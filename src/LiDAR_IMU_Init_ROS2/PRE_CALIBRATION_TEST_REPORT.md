# Stage 2A validation

Environment: ROS 2 Kilted, Ceres 2.2, Ubuntu 24.04. Only source and recorded data were used. No motor commands or live calibration were run.

| Check | Status | Evidence |
|---|---|---|
| Focused source audit | PASS | Source paths and findings in `PRE_CALIBRATION_ISSUES.md` |
| Quaternion equivalence and finite differences | PASS | `test_precalibration` gtest |
| Excitation schedule and Jacobian boundary | PASS | `test_precalibration` gtest |
| Synthetic Livox splitting/timestamps | PASS | `test_precalibration` gtest |
| Synthetic IMU validity and units | PASS | `test_precalibration` gtest |
| Package build and test | PASS | `colcon build` completed; `colcon test-result` reported 8 tests, 0 errors, 0 failures (7 gtest cases plus CTest wrapper) |
| Short recorded-bag ingestion | PASS with environment warning | Isolated `ROS_DOMAIN_ID=231` replay reached LiDAR-only odometry and logged acceleration norm 9.80329 m/s². Sandbox emitted FastDDS transport/getifaddrs errors; process was stopped by short timeout. No result was generated. |
| Extrinsic accuracy and convergence | NOT TESTED | Requires Stage 2B experiment and independent checks |

Commands:

```bash
cd /home/cuong/lm_calibr
source /opt/ros/kilted/setup.bash
source /home/cuong/ws_livox/install/setup.bash
colcon build --packages-select lidar_imu_init --symlink-install
colcon test --packages-select lidar_imu_init
colcon test-result --verbose
```

**GO for a separately supervised Stage 2B data-collection plan**, subject to checking the compensated cloud's frame label and physical IMU/LiDAR time basis. This is not a claim of calibrated extrinsics or timing. The sandbox's DDS transport warnings mean the smoke result is limited to demonstrated ingestion, not a clean sustained runtime test.
