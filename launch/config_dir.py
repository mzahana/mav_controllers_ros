"""Where the on-vehicle parameter overrides live.

Single source of truth, imported by the launch files and by gain_saver so the
writer and the readers cannot disagree -- an override written somewhere the
launch files do not read is worse than no persistence at all, because it looks
like it worked.

The deployed system runs inside a container on the Jetson, so the container's
own filesystem is scratch: anything written to ~/.ros is gone when the
container is recreated. The default therefore prefers a shared volume mounted
from the host, and never the installed package (a colcon rebuild wipes
install/) nor the source tree (which must stay clean in git).

Resolution order:
  1. $MAV_CONTROLLERS_CONFIG_DIR, if set -- the explicit answer for any
     deployment that does not match the conventions below.
  2. The first shared-volume mount from SHARED_VOLUME_CANDIDATES that exists,
     with 'mav_controllers_config' underneath it.
  3. The mount point this package is installed under, if it is a mount other
     than '/'. The workspace itself normally lives on the shared volume, so
     this finds the right place whichever account runs the stack, without
     depending on $HOME.
  4. ~/.ros/mav_controllers_ros -- development fallback, container-local.
"""

import os

ENV_VAR = "MAV_CONTROLLERS_CONFIG_DIR"

# Host-mounted directories, in the order they are preferred. Add to this list
# rather than teaching each launch file a new path.
SHARED_VOLUME_CANDIDATES = (
    "~/shared_volume",
    "~/d2dtracker_cuda_shared_volume",
)

SUBDIR = "mav_controllers_config"


def resolve_config_dir():
    """Absolute path of the override directory. Does not create it."""
    env = os.environ.get(ENV_VAR)
    if env:
        return os.path.abspath(os.path.expanduser(env))

    for candidate in SHARED_VOLUME_CANDIDATES:
        path = os.path.expanduser(candidate)
        # Only a real mount counts: a stale symlink or a missing mount would
        # silently put the file back on the container's scratch filesystem.
        if os.path.isdir(path) and os.path.ismount(path):
            return os.path.join(path, SUBDIR)

    for candidate in SHARED_VOLUME_CANDIDATES:
        path = os.path.expanduser(candidate)
        if os.path.isdir(path):
            return os.path.join(path, SUBDIR)

    mount = _install_mount_point()
    if mount:
        return os.path.join(mount, SUBDIR)

    return os.path.join(os.path.expanduser("~"), ".ros", "mav_controllers_ros")


def _install_mount_point():
    """Mount point this package is installed under, or None.

    $HOME differs between the account that builds the workspace and one that
    happens to run a node (root in a `docker exec`, say), so a $HOME-relative
    guess can silently put the override on the container's scratch
    filesystem. The installed package's own path cannot lie about where the
    workspace lives.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.realpath(get_package_share_directory("mav_controllers_ros"))
    except Exception:
        return None

    while path and path != "/":
        if os.path.ismount(path):
            return path
        path = os.path.dirname(path)
    return None


def override_path(name):
    """Path of one override file, or None when it does not exist yet.

    A ROS parameter file list cannot express "load this if present", and
    passing a missing path makes the node fail to start, so this is resolved
    at launch time.
    """
    path = os.path.join(resolve_config_dir(), name)
    return path if os.path.isfile(path) else None
