# ROS 2 Kilted audit — 2026-11-11

## Environment

Ubuntu 24.04, ROS 2 Kilted, GCC on x86_64, Ceres 2.2.0, PCL 1.14, Eigen 3, Python 3.12, system NumPy, `livox_ros_driver2` from `/home/cuong/ws_livox/install`. The package is named `lidar_imu_init` and its executable is `li_init`. Build dependencies are present locally; no system packages were installed.

## Confirmed findings

| Severity | Finding and evidence | Resolution |
| --- | --- | --- |
| High | Six `launch/*.py` files named `lidar_imu_init2`, but `package.xml:3` declares `lidar_imu_init`. | Corrected launch references. |
| High | PCL injected bare `EIGEN_MPL2_ONLY` compiler tokens; C compilation failed (`CMakeLists.txt:55-56`). | Removed the malformed definition after `find_package(PCL)`. |
| High | Original C++14 flags conflict with Kilted/Ceres 2.2 (`CMakeLists.txt:11`). | Set C++17 and removed conflicting `-std=c++14` flags. |
| High | Ceres 2.2 removed `LocalParameterization`/`QuaternionParameterization` (`include/LI_init/LI_init.cpp:327,364,427`). | Use the equivalent scalar-first `QuaternionManifold`; residuals, Jacobians, states and solver options are unchanged. |
| High | NumPy header was missing from CMake includes although system NumPy was present (`CMakeLists.txt:64`). | Find Python3 NumPy and include its headers. |
| High | Four cloud outputs and the post-initialization IMU buffer constructed `rclcpp::Time(double_seconds)`, treating seconds as nanoseconds (`src/laserMapping.cpp:616,655,670,679,1349`). | Use existing `get_ros_time(double_seconds)`. |
| Medium | Results and debug logs wrote into the package source tree (`src/laserMapping.cpp:919-923`, `include/common_lib.h:36-45`). | `result_dir` parameter; debug directory controlled by `LIDAR_IMU_INIT_LOG_DIR`, default `/tmp/lidar_imu_init/Log`. |
| Medium | `process_cut_frame_livox` indexed `points` using `point_num` and divided by an unchecked filter value (`src/preprocess.cpp:50-60`). | Bound point count and reject invalid settings or too-short frames. |
| Medium | Duplicate-point check read `pl_full[i-1]` even when the previous received point had been filtered (`src/preprocess.cpp:80-85`). | Compare against the preceding received point directly; no frame transform or time scaling changed. |
| Low | `package.xml:13-16` included ROS 1 `rospy` and omitted direct ROS 2 build dependencies. | Removed `rospy`; added `rclcpp_components`, `pcl_conversions`, `ament_index_cpp`. |
| Low | `include/matplotlibcpp.h:153` calls deprecated `Py_SetProgramName` under Python 3.12. | Left unchanged; warning only. |

## Input path and timestamp checks

`src/laserMapping.cpp::livox_pcl_cbk` subscribes to `livox_ros_driver2/msg/CustomMsg`, then calls `src/preprocess.cpp::process_cut_frame_livox`. XYZ and reflectivity are copied directly; no extra transform is applied by the parser. It accepts points with `line < scan_line` and tag upper bits `0x00` or `0x10`, then uses `offset_time / 1e6` as milliseconds. Frame splitting stores a sub-frame start in milliseconds; `sync_packages` converts the final curvature back to seconds. IMU callback consumes `sensor_msgs/msg/Imu` angular velocity and acceleration and uses `header.stamp`, without requiring orientation quaternion. A hard one-time offset is applied only if the two latest stamps differ by more than 1 s; later estimates subtract offset from IMU stamps.

The existing 55-second bag includes approximately 10 Hz `/calibrated_livox` and 100 Hz `/imu/data_raw`. One inspected corrected frame had 19,964 points, lines 0–3, accepted tag patterns only, and a 100.22 ms point-offset span. Five software-computed partitions were monotonic after range/tag/filter selection. The corrected cloud preserves Livox stamps, `timebase`, and point offsets. A two-second isolated replay reached LiDAR-only processing and measured IMU norm 9.79986 m/s². This verifies short ingestion, not calibration accuracy.

## Remaining risks and suspected issues

- The compensated CustomMsg still reports `header.frame_id = livox_frame`, although its XYZ values are in the fixed motor-base frame. The LI-Init parser ignores `frame_id`; downstream frame semantics should be corrected in a future LM-Calibr change, outside Stage 1 scope.
- WT901C uses host-reception timestamps. A small nearest-stamp difference does not establish the physical LiDAR–IMU offset. Its 100 Hz rate is below the source warning recommending over 150 Hz; this may reduce interpolation quality.
- The original code assumes normal monotonic streams. It clears buffers on strictly backward timestamps but does not reject equal stamps. `sync_packages` reads buffers without taking the callback mutex; the current `rclcpp::spin` uses a single-threaded executor, so callback/timer serialization prevents this race in the tested launch. A future multi-threaded executor would require locking.
- The original parser drops the first point of each frame and filters near points; whether its thresholds retain enough scene structure for this MID-360 mounting requires a longer bag test.
- `data_sufficiency_assess` and the refinement progress gate use `frame_num % orig_odom_freq * cut_frame_num == 0`. C++ parses this as `(frame_num % orig_odom_freq) * cut_frame_num == 0`, so checks occur every 10 subframes, not every 50 with the current settings. This is inherited algorithm/control-flow behavior; it was documented, not changed. The fixed-size `Jaco_rot(30000,3)` can also be exceeded if more than 10,000 subframes accumulate. Both require separate algorithm review before long calibration runs.
- Full numerical equivalence with the ROS 1 upstream optimization has not been proved. The visible rotation, gyro bias, time-correlation, translation, and gravity pipeline and residual calls match the original structure; a full source-level mathematical diff and paired-dataset comparison remain for later review.
- The node's SIGINT shutdown is slow in a timed smoke launch; the test runner escalated after a short timeout. Normal shutdown behavior needs a longer interactive test.

Stage 1 build and interface foundation: **PASS**. Full calibration and time-offset accuracy: **NOT TESTED**.
