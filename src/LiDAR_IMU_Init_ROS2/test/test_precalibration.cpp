#include <gtest/gtest.h>
#include <ceres/ceres.h>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include "LI_init/runtime_safety.h"
#include "LI_init/LI_init.h"
#include "preprocess.h"

TEST(Scheduling, OncePerOriginalSecond) {
  for (int cuts : {1, 2, 5}) {
    for (int i = 1; i <= 100; ++i)
      EXPECT_EQ(li_init_check_due(i, 10, cuts), i % (10 * cuts) == 0);
  }
  EXPECT_FALSE(li_init_check_due(10, 0, 5));
}

TEST(Jacobian, GrowsPastFormerBoundary) {
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(30000, 3);
  jacobian(3, 0) = 7.0;
  li_init_ensure_jacobian_rows(jacobian, 9999);
  EXPECT_EQ(jacobian.rows(), 30000);
  li_init_ensure_jacobian_rows(jacobian, 10000);
  EXPECT_GE(jacobian.rows(), 30003);
  EXPECT_EQ(jacobian(3, 0), 7.0);
  EXPECT_EQ(jacobian(30000, 0), 0.0);
  EXPECT_THROW(li_init_ensure_jacobian_rows(jacobian, -1), std::out_of_range);
}

TEST(Imu, ValidatesFiniteSIInputsAndStamp) {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = 123;
  msg.header.stamp.nanosec = 250000000;
  msg.linear_acceleration.z = 9.805;
  msg.angular_velocity.z = M_PI / 2;
  EXPECT_TRUE(li_init_valid_imu(msg));
  EXPECT_NEAR(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 123.25, 1e-12);
  EXPECT_NEAR(msg.angular_velocity.z * 2.0, M_PI, 1e-12);
  msg.angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(li_init_valid_imu(msg));
  msg.angular_velocity.x = 0;
  msg.header.stamp.nanosec = 1000000000u;
  EXPECT_FALSE(li_init_valid_imu(msg));
}

TEST(Quaternion, CeresManifoldAndResidualJacobian) {
  ceres::QuaternionManifold manifold;
  const Eigen::Vector3d lidar(0.3, -0.2, 0.7), imu(-0.1, 0.4, 0.2);
  std::unique_ptr<ceres::CostFunction> cost(Angular_Vel_Cost_only_Rot::Create(imu, lidar));
  for (double angle : {0.0, 1e-6, 0.7, 3.0, M_PI / 2, -M_PI / 2, M_PI - 1e-6}) {
    for (const Eigen::Vector3d axis : std::vector<Eigen::Vector3d>{Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ(), Eigen::Vector3d(1, 2, 3).normalized()}) {
    Eigen::Quaterniond base(Eigen::AngleAxisd(angle, axis));
    double q[4] = {base.w(), base.x(), base.y(), base.z()};
    double delta[3] = {0.02, -0.01, 0.03}, updated[4], recovered[3];
    ASSERT_TRUE(manifold.Plus(q, delta, updated));
    ASSERT_TRUE(manifold.Minus(updated, q, recovered));
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(recovered[j], delta[j], 1e-12);
    Eigen::Quaterniond expected(Eigen::AngleAxisd(2 * Eigen::Vector3d(0.02, -0.01, 0.03).norm(), Eigen::Vector3d(0.02, -0.01, 0.03).normalized()));
    expected = expected * base;
    EXPECT_NEAR(updated[0], expected.w(), 1e-12);
    EXPECT_NEAR(updated[1], expected.x(), 1e-12);
    EXPECT_NEAR(Eigen::Quaterniond(updated[0], updated[1], updated[2], updated[3]).norm(), 1.0, 1e-12);
    double residual[3], ambient[12], plus_jacobian[12];
    double *jac[] = {ambient};
    const double *q_ptr[] = {q};
    ASSERT_TRUE(cost->Evaluate(q_ptr, residual, jac));
    ASSERT_TRUE(manifold.PlusJacobian(q, plus_jacobian));
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> Jambient(ambient);
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> Jplus(plus_jacobian);
    Eigen::Matrix3d tangent = Jambient * Jplus;
    for (int k = 0; k < 3; ++k) {
      double step[3] = {}, qp[4], qm[4], rp[3], rm[3];
      step[k] = 1e-7; manifold.Plus(q, step, qp);
      step[k] = -1e-7; manifold.Plus(q, step, qm);
      const double *qp_ptr[] = {qp}, *qm_ptr[] = {qm};
      cost->Evaluate(qp_ptr, rp, nullptr); cost->Evaluate(qm_ptr, rm, nullptr);
      for (int r = 0; r < 3; ++r)
        EXPECT_NEAR(tangent(r, k), (rp[r] - rm[r]) / 2e-7, 1e-7);
    }
    }
  }
}

TEST(LivoxSplit, RetainsAllPointsAndTimeOrder) {
  Preprocess parser;
  parser.set(false, AVIA, 0.0, 1);
  auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  msg->header.stamp.sec = 100;
  msg->point_num = 11;
  for (unsigned i = 0; i < 11; ++i) {
    livox_ros_driver2::msg::CustomPoint p;
    p.x = 2.0f + i; p.y = 0; p.z = 0;
    p.line = 0; p.tag = 0x10; p.offset_time = i * 1000000;
    msg->points.push_back(p);
  }
  std::deque<PointCloudXYZI::Ptr> clouds;
  std::deque<double> stamps;
  parser.process_cut_frame_livox(msg, clouds, stamps, 5, 6);
  ASSERT_EQ(clouds.size(), 5u);
  ASSERT_EQ(stamps.size(), clouds.size());
  size_t count = 0;
  double last_time = -1;
  for (size_t k = 0; k < clouds.size(); ++k) {
    ASSERT_GE(clouds[k]->size(), 2u);
    for (const auto &p : *clouds[k]) {
      const double t = stamps[k] + p.curvature;
      EXPECT_GE(t, last_time);
      last_time = t;
      ++count;
    }
  }
  EXPECT_EQ(count, 11u);
  EXPECT_NEAR(last_time, 100010.0, 1e-3);
}

TEST(LivoxSplit, NoSplitAndInsufficientData) {
  Preprocess parser;
  parser.set(false, AVIA, 0.0, 1);
  auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
  msg->header.stamp.sec = 123;
  msg->header.stamp.nanosec = 999000000;
  for (unsigned i = 0; i < 3; ++i) {
    livox_ros_driver2::msg::CustomPoint p;
    p.x = 2.0f + i; p.line = 0; p.tag = 0x10;
    p.offset_time = (2 - i) * 1000000;
    msg->points.push_back(p);
  }
  msg->point_num = 3;
  std::deque<PointCloudXYZI::Ptr> clouds;
  std::deque<double> stamps;
  parser.process_cut_frame_livox(msg, clouds, stamps, 1, 6);
  ASSERT_EQ(clouds.size(), 1u);
  EXPECT_EQ(clouds[0]->size(), 3u);
  EXPECT_NEAR(stamps[0] + clouds[0]->points.front().curvature, 123999.0, 1e-3);
  EXPECT_NEAR(stamps[0] + clouds[0]->points.back().curvature, 124001.0, 1e-3);
  clouds.clear(); stamps.clear();
  msg->point_num = 1;
  parser.process_cut_frame_livox(msg, clouds, stamps, 5, 6);
  EXPECT_TRUE(clouds.empty());
  msg->point_num = 0;
  parser.process_cut_frame_livox(msg, clouds, stamps, 5, 6);
  EXPECT_TRUE(clouds.empty());
}

TEST(LivoxSplit, ConsecutiveScansDoNotDuplicateBoundaryPoints) {
  Preprocess parser;
  parser.set(false, AVIA, 0.0, 1);
  double previous_end = -1;
  for (int scan = 0; scan < 2; ++scan) {
    auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
    msg->header.stamp.sec = 200;
    msg->header.stamp.nanosec = scan * 100000000;
    msg->point_num = 10;
    for (unsigned i = 0; i < 10; ++i) {
      livox_ros_driver2::msg::CustomPoint p;
      p.x = 2 + i; p.line = 0; p.tag = 0x10;
      p.offset_time = i * 10000000;
      msg->points.push_back(p);
    }
    std::deque<PointCloudXYZI::Ptr> clouds;
    std::deque<double> stamps;
    parser.process_cut_frame_livox(msg, clouds, stamps, 5, 6 + scan);
    ASSERT_EQ(clouds.size(), 5u);
    size_t n = 0;
    for (size_t k = 0; k < clouds.size(); ++k) {
      for (const auto &p : *clouds[k]) {
        const double point_time = stamps[k] + p.curvature;
        EXPECT_GT(point_time, previous_end);
        previous_end = point_time;
        ++n;
      }
    }
    EXPECT_EQ(n, 10u);
  }
}
