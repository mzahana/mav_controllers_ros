#!/usr/bin/env python3
"""Persist live controller gains to disk ON THE VEHICLE.

Where: see launch/config_dir.py. On the Jetson the deployed stack runs in a
container, so the default resolves to a host-mounted shared volume -- the
container's own filesystem does not survive being recreated, and neither the
installed package (wiped by colcon) nor the source tree (must stay clean in
git) is an acceptable place to write.

A ROS parameter write changes a running node's memory and nothing else: it is
lost on the next restart or power cycle. That volatility is useful in flight
(a bad gain cannot survive a reboot) but it means a tuning session evaporates
unless something writes it down. RViz runs on a laptop and cannot touch the
vehicle's filesystem, so this node -- which runs ON the vehicle -- does it,
on request.

It reads the CURRENT values from the running controller and mavros nodes and
writes them as override YAML files that the launch files load if present.
Writes are atomic (temp file + rename) and the previous file is kept as a
timestamped backup, so a save can never leave a half-written config that
would fail to parse at the next boot.

Services
  ~/save (std_srvs/SetBool)
      data = false : save the gains as they are now.
      data = true  : additionally correct max_thrust by the online thrust-scale
                     estimate (max_thrust * scale), capturing what the vehicle
                     learned in flight. REFUSED WHILE ARMED -- max_thrust
                     linearly scales every position gain, so changing it
                     retunes the whole controller at once.

Only the parameters listed in CONTROLLER_KEYS / MAVROS_KEYS are written, so an
override file stays small and reviewable rather than a dump of every parameter
a node happens to declare.
"""

import os
import sys
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from mavros_msgs.msg import State
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

# The controller calls the position gain kx and the velocity gain kv in its
# maths, but declares them as gains.pos.* / gains.vel.*.
# Same resolver the launch files use, so the writer and the readers cannot
# disagree about where an override lives (see launch/config_dir.py).
sys.path.insert(0, os.path.join(get_package_share_directory("mav_controllers_ros"), "launch"))
from config_dir import ENV_VAR, resolve_config_dir  # noqa: E402

CONTROLLER_KEYS = [
    "gains.pos.x", "gains.pos.y", "gains.pos.z",
    "gains.vel.x", "gains.vel.y", "gains.vel.z",
    "gains.ki.x", "gains.ki.y", "gains.ki.z",
    "attctrl_tau", "yawctrl_tau",
]

MAVROS_KEYS = ["max_thrust", "mass"]


def unflatten(flat: dict) -> dict:
    """{'gains.kx.x': 1.0} -> {'gains': {'kx': {'x': 1.0}}}, matching the
    nesting of the shipped config files so an override reads like them."""
    out: dict = {}
    for key, value in flat.items():
        node = out
        parts = key.split(".")
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return out


class GainSaver(Node):
    def __init__(self):
        super().__init__("gain_saver")

        self.declare_parameter("controller_node", "geometric_controller_node")
        self.declare_parameter("mavros_node", "geometric_mavros_node")
        # Empty means "resolve it the same way the launch files do", which
        # on the Jetson lands on the host-mounted shared volume: the
        # container filesystem is scratch, and an override written there is
        # gone the next time the container is recreated.
        self.declare_parameter("output_dir", "")
        self.declare_parameter("keep_backups", 10)

        self.controller_node = str(self.get_parameter("controller_node").value).strip("/")
        self.mavros_node = str(self.get_parameter("mavros_node").value).strip("/")
        self.output_dir = str(self.get_parameter("output_dir").value) or resolve_config_dir()
        self.keep_backups = int(self.get_parameter("keep_backups").value)

        self.armed = False
        self.thrust_scale = None

        self.create_subscription(State, "mavros/state", self._state_cb, 10)
        self.create_subscription(Float64, "geometric_mavros/thrust_scale_estimate",
                                 self._scale_cb, 10)

        # The save handler blocks waiting on these clients. With the default
        # mutually-exclusive callback group the responses could never be
        # delivered while the handler waits -- a deadlock that shows up as a
        # timeout. A reentrant group lets both run.
        self.cb_group = ReentrantCallbackGroup()
        self.ctrl_cli = self.create_client(
            GetParameters, f"/{self.controller_node}/get_parameters",
            callback_group=self.cb_group)
        self.mavros_cli = self.create_client(
            GetParameters, f"/{self.mavros_node}/get_parameters",
            callback_group=self.cb_group)

        self.create_service(SetBool, "~/save", self._save_cb,
                            callback_group=self.cb_group)

        self.get_logger().info(
            f"gain_saver up. Writing overrides to {self.output_dir} "
            f"(override with the output_dir parameter or ${ENV_VAR}); "
            f"reading {self.controller_node} and {self.mavros_node}. "
            "Call ~/save (SetBool) to persist the live gains.")

    # ------------------------------------------------------------------
    def _state_cb(self, msg):
        self.armed = msg.armed

    def _scale_cb(self, msg):
        self.thrust_scale = msg.data

    def _read(self, client, node_name, keys):
        """Fetch parameters synchronously. Called from a service callback, so
        the node is spun by a multi-threaded executor (see main)."""
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"{node_name} parameter service not available")
        req = GetParameters.Request()
        req.names = list(keys)
        future = client.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"timed out reading parameters from {node_name}")
        values = future.result().values
        if len(values) != len(keys):
            raise RuntimeError(f"{node_name} returned {len(values)} of {len(keys)} parameters")

        out = {}
        for key, pv in zip(keys, values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                # The controller declares its gains as float32, so a value
                # read back lands as 7.400000095367432. Rounding keeps the
                # written file readable and diffable; the discarded digits
                # are below float32 precision anyway.
                out[key] = round(float(pv.double_value), 6)
            elif pv.type == ParameterType.PARAMETER_INTEGER:
                out[key] = int(pv.integer_value)
            elif pv.type == ParameterType.PARAMETER_BOOL:
                out[key] = bool(pv.bool_value)
            elif pv.type == ParameterType.PARAMETER_NOT_SET:
                # An older build without yawctrl_tau, say: skip rather than
                # writing a null that would fail to load.
                continue
            else:
                raise RuntimeError(f"parameter {key} has unsupported type {pv.type}")
        return out

    def _write_atomic(self, path, node_name, flat):
        os.makedirs(os.path.dirname(path), exist_ok=True)

        if os.path.exists(path):
            stamp = time.strftime("%Y%m%d-%H%M%S")
            backup = f"{path}.{stamp}.bak"
            os.replace(path, backup)
            self._prune_backups(path)

        doc = {node_name: {"ros__parameters": unflatten(flat)}}
        header = (
            "# Written by gain_saver from the LIVE parameters of "
            f"{node_name}.\n"
            f"# {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
            "# Loaded after the shipped config, so these values win. Delete\n"
            "# this file to fall back to the package defaults.\n")

        # Atomic: a partial file here would fail to parse at the next boot,
        # which is the one moment nobody is watching.
        directory = os.path.dirname(path)
        fd, tmp = tempfile.mkstemp(dir=directory, suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as f:
                f.write(header)
                yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)
                f.flush()
                os.fsync(f.fileno())
            # mkstemp creates 0600, but the stack may be built by one
            # account and run by another (user vs root in a container), and
            # a config the launch cannot read fails the node at boot.
            os.chmod(tmp, 0o644)
            os.replace(tmp, path)
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def _prune_backups(self, path):
        directory = os.path.dirname(path)
        base = os.path.basename(path)
        backups = sorted(f for f in os.listdir(directory)
                         if f.startswith(base + ".") and f.endswith(".bak"))
        for old in backups[:max(0, len(backups) - self.keep_backups)]:
            try:
                os.unlink(os.path.join(directory, old))
            except OSError:
                pass

    # ------------------------------------------------------------------
    def _save_cb(self, request, response):
        correct_thrust = bool(request.data)

        if correct_thrust and self.armed:
            response.success = False
            response.message = ("Refused: max_thrust correction requires the vehicle to be "
                                "DISARMED. It scales every position gain, so it retunes the "
                                "whole controller at once.")
            return response

        try:
            ctrl = self._read(self.ctrl_cli, self.controller_node, CONTROLLER_KEYS)
            mav = self._read(self.mavros_cli, self.mavros_node, MAVROS_KEYS)
        except RuntimeError as exc:
            response.success = False
            response.message = f"Save failed: {exc}"
            return response

        notes = []
        if correct_thrust:
            if self.thrust_scale is None:
                response.success = False
                response.message = ("Refused: no thrust-scale estimate received on "
                                    "geometric_mavros/thrust_scale_estimate.")
                return response
            old = mav.get("max_thrust")
            if not old:
                response.success = False
                response.message = "Refused: max_thrust could not be read."
                return response
            mav["max_thrust"] = old * self.thrust_scale
            notes.append(f"max_thrust {old:.2f} -> {mav['max_thrust']:.2f} N "
                         f"(scale {self.thrust_scale:.3f})")

        ctrl_path = os.path.join(self.output_dir, "geometric_controller.override.yaml")
        mav_path = os.path.join(self.output_dir, "geometric_mavros.override.yaml")
        try:
            self._write_atomic(ctrl_path, self.controller_node, ctrl)
            self._write_atomic(mav_path, self.mavros_node, mav)
        except OSError as exc:
            response.success = False
            response.message = f"Save failed writing files: {exc}"
            return response

        response.success = True
        response.message = f"Saved to {ctrl_path} and {mav_path}."
        if notes:
            response.message += " " + "; ".join(notes) + "."
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GainSaver()
    # The save callback blocks on parameter calls to other nodes, so it must
    # not run on the same single-threaded executor that has to service them.
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
