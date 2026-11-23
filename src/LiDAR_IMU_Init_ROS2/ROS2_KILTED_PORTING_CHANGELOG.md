# Porting changelog

- `CMakeLists.txt`: C++17 for Kilted/Ceres 2.2; remove malformed PCL `EIGEN_MPL2_ONLY` token; include installed NumPy headers. Build-only, no estimation equation changes.
- `package.xml`: remove ROS 1 `rospy`; declare ROS 2 dependencies already used by CMake. Manifest-only.
- `launch/*.py` (six original files): correct package lookup from `lidar_imu_init2` to `lidar_imu_init`. Launch-only.
- `config/motorized_mid360_wt901c.yaml`: copy original MID-360 settings, selecting `/calibrated_livox` and `/imu/data_raw`. Existing `cut_frame_num=5`, `orig_odom_freq=10`, `mean_acc_norm=9.805` retained. Configuration-only.
- `launch/motorized_mid360_wt901c.launch.py`: dedicated launch and writable `result_dir` argument; no static TF or calibration value. Launch-only.
- `include/LI_init/LI_init.cpp`: replace removed Ceres scalar-first quaternion local-parameterization API with Ceres 2.2 `QuaternionManifold` at three parameter blocks. Same quaternion ordering and 3D tangent representation; no residual, Jacobian, state or solver changes. Numerical equivalence is expected but not experimentally proved.
- `include/LI_init/LI_init.h`, `include/common_lib.h`: route debug logs to a writable runtime directory. I/O-only.
- `src/preprocess.cpp`: bound `point_num` by actual message array size; ignore invalid filter/cut settings and frames with fewer than two points. Duplicate-point comparison now uses the preceding received point when a filtered `pl_full` slot was never populated. Frame transform and split-time mathematics unchanged.
- `src/laserMapping.cpp`: use the existing seconds-to-ROS-time helper for four cloud output stamps and post-initialization IMU buffer; write results under `result_dir`; export numeric 3×3 rotation, translation, 4×4 transform, biases, gravity, and the sign convention. Timestamp/I/O changes only.

Validation: `colcon build --packages-select lidar_imu_init --symlink-install` passed; `ros2 pkg executables` listed `li_init`; isolated ROS graph showed the two intended input subscriptions; a two-second bag replay entered LiDAR-only processing. No actual calibration run was performed.

## Stage 2A

- `include/LI_init/runtime_safety.h`: shared scheduling, dynamic Jacobian capacity, and finite IMU validation helpers.
- `include/LI_init/LI_init.cpp`, `src/laserMapping.cpp`: correct excitation scheduling at both call sites and grow Jacobian before writes; reject malformed IMU samples with a message. Calibration residuals and thresholds unchanged.
- `src/preprocess.cpp`: retain the first valid Livox point, distribute every valid point across subframes, require at least two points per chunk, and report rejected undersized scans. XYZ and offset units unchanged.
- `CMakeLists.txt`, `package.xml`, `test/test_precalibration.cpp`: deterministic Ceres, scheduling, memory boundary, Livox timing, and IMU tests.
- `PRE_CALIBRATION_ISSUES.md`, `NUMERICAL_EQUIVALENCE_TESTS.md`, `PRE_CALIBRATION_TEST_REPORT.md`: audit and validation record.
