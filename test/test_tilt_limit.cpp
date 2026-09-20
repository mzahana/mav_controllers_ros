// Regression tests for the tilt limiter in GeometricAttitudeControl.
//
// Field flight 2026-09-20: whenever the commanded tilt exceeded
// max_tilt_angle the limiter returned lambda*acc + gravity_vec_ with
// gravity_vec_ = (0,0,-g), i.e. a thrust vector pointing DOWN. PX4 received
// zero throttle and body rates up to 330 deg/s, 15 times in OFFBOARD. The
// limiter had never engaged in earlier flights and nothing tested it.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "mav_controllers_ros/GeometricAttitudeControl.h"

namespace
{
constexpr float kMass = 2.3f;
constexpr float kG = 9.81f;
constexpr float kMaxTilt = 0.52f;  // rad, the field value

const Eigen::Vector3f kZero = Eigen::Vector3f::Zero();

float tiltOf(const Eigen::Vector3f & f)
{
  return std::atan2(std::hypot(f.x(), f.y()), f.z());
}

GeometricAttitudeControl makeController(float max_tilt)
{
  GeometricAttitudeControl c;
  c.setMass(kMass);
  c.setGravity(kG);
  c.setMaxAcceleration(5.0f);
  if (max_tilt > 0.0f) {c.setMaxTiltAngle(max_tilt);}
  c.setPosition(kZero);
  c.setVelocity(kZero);
  c.setCurrentOrientation(Eigen::Quaternionf::Identity());
  return c;
}

// Zero gains and zero errors: acc_control is exactly the feedforward, so the
// limiter's input is known in closed form.
void runFeedforward(GeometricAttitudeControl & c, const Eigen::Vector3f & acc, float yaw = 0.0f)
{
  c.calculateControl(kZero, kZero, acc, kZero, yaw, 0.0f, kZero, kZero, kZero, kZero, 0.3f, 0.02f);
}
}  // namespace

// The exact acceleration demand of the first OFFBOARD burst that reached 30
// deg in the bag (E4, t = 744.15 s): F = (13.7, 5.9, -19.1) N was published.
TEST(TiltLimit, FlightSampleCommandsUpwardThrustAtTheLimit)
{
  auto c = makeController(kMaxTilt);
  runFeedforward(c, Eigen::Vector3f(5.96f, 2.57f, -1.5f));
  const Eigen::Vector3f f = c.getComputedForce();
  EXPECT_GT(f.z(), 0.0f) << "thrust vector points down: F = " << f.transpose();
  EXPECT_NEAR(tiltOf(f), kMaxTilt, 1e-3f);
  EXPECT_TRUE(c.isSaturated());
}

TEST(TiltLimit, SweepNeverInvertsAndNeverExceedsTheLimit)
{
  int limited = 0, free_cases = 0;
  for (float r = 0.5f; r <= 14.0f; r += 0.75f) {
    for (float az = -8.0f; az <= 6.0f; az += 1.0f) {
      for (float psi = 0.0f; psi < 6.28f; psi += 0.7f) {
        auto c = makeController(kMaxTilt);
        const Eigen::Vector3f acc(r * std::cos(psi), r * std::sin(psi), az);
        runFeedforward(c, acc, psi * 0.5f);
        const Eigen::Vector3f f = c.getComputedForce();
        const Eigen::Vector3f unlimited = kMass * (acc + Eigen::Vector3f(0, 0, kG));
        const std::string where = "acc = (" + std::to_string(acc.x()) + ", " + std::to_string(acc.y()) +
          ", " + std::to_string(acc.z()) + ")";

        ASSERT_TRUE(f.allFinite()) << where;
        ASSERT_GT(f.z(), 0.0f) << where << " -> F = " << f.transpose();
        ASSERT_LE(tiltOf(f), kMaxTilt + 1e-3f) << where;

        if (tiltOf(unlimited) > kMaxTilt + 1e-4f) {
          ++limited;
          EXPECT_NEAR(tiltOf(f), kMaxTilt, 1e-3f) << where;
          EXPECT_TRUE(c.isSaturated()) << where;
          // Same horizontal direction, never a larger horizontal force.
          EXPECT_NEAR(std::atan2(f.y(), f.x()), std::atan2(acc.y(), acc.x()), 1e-3f) << where;
          EXPECT_LE(std::hypot(f.x(), f.y()), std::hypot(unlimited.x(), unlimited.y()) + 1e-3f) << where;
        } else {
          ++free_cases;
          EXPECT_TRUE(f.isApprox(unlimited, 1e-4f)) << where;
          EXPECT_FALSE(c.isSaturated()) << where;
        }

        // The commanded attitude must agree with the commanded force.
        const Eigen::Vector3f zb = c.getComputedOrientation().toRotationMatrix().col(2);
        EXPECT_GT(zb.z(), 0.0f) << where;
        EXPECT_NEAR(zb.dot(f.normalized()), 1.0f, 1e-4f) << where;
      }
    }
  }
  // Guard against a sweep that silently stops exercising the limiter.
  EXPECT_GT(limited, 500);
  EXPECT_GT(free_cases, 100);
}

// The limiter is reached through feedback too, not just feedforward: a large
// velocity error is what actually tipped it over in flight.
TEST(TiltLimit, FeedbackPlusFeedforwardIsLimited)
{
  auto c = makeController(kMaxTilt);
  c.setVelocity(Eigen::Vector3f(-0.46f, -0.19f, 0.48f));
  const Eigen::Vector3f kx(2.40f, 2.37f, 2.79f), kv(2.85f, 3.30f, 3.15f);
  c.calculateControl(Eigen::Vector3f(0.12f, 0.07f, 0.11f), kZero, Eigen::Vector3f(4.62f, 1.91f, 0.0f), kZero,
    1.2f, 0.0f, kx, kv, kZero, kZero, 0.3f, 0.02f);
  const Eigen::Vector3f f = c.getComputedForce();
  EXPECT_GT(f.z(), 0.0f) << "F = " << f.transpose();
  EXPECT_NEAR(tiltOf(f), kMaxTilt, 1e-3f);
}

// max_tilt_angle defaults to pi (limiter off). That path must be untouched.
TEST(TiltLimit, DisabledByDefault)
{
  auto c = makeController(-1.0f);
  const Eigen::Vector3f acc(8.0f, -3.0f, -2.0f);
  runFeedforward(c, acc);
  EXPECT_TRUE(c.getComputedForce().isApprox(kMass * (acc + Eigen::Vector3f(0, 0, kG)), 1e-4f));
  EXPECT_FALSE(c.isSaturated());
}

// Matches the reference implementation the formula was ported from.
TEST(TiltLimit, HoverIsUnaffected)
{
  auto c = makeController(kMaxTilt);
  runFeedforward(c, kZero);
  EXPECT_TRUE(c.getComputedForce().isApprox(Eigen::Vector3f(0, 0, kMass * kG), 1e-4f));
  EXPECT_FALSE(c.isSaturated());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
