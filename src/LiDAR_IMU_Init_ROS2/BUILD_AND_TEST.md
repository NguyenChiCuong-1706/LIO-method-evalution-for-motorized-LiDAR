# Build and Stage 1 checks

```bash
cd /home/cuong/lm_calibr
source /opt/ros/kilted/setup.bash
source /home/cuong/ws_livox/install/setup.bash
colcon build --packages-select lidar_imu_init --symlink-install
source install/setup.bash
ros2 pkg executables lidar_imu_init
ros2 launch lidar_imu_init motorized_mid360_wt901c.launch.py --show-args
```

Build **PASS** on 2026-11-11. Executable check returned `lidar_imu_init li_init`. Launch-argument check returned the expected writable result directory. In an isolated ROS domain, `ros2 node info /laser_mapping` showed `/calibrated_livox` (`CustomMsg`) and `/imu/data_raw` (`Imu`); parameter reads matched those names. The node started without an immediate crash. A two-second replay of `data/time_sync_20261110_112506` reached LiDAR-only processing and printed acceleration norm 9.79986 m/s². Result file remained empty, as expected without initialization. The test did not send motor commands or execute calibration.

Launch when ready for a separately planned calibration experiment:

```bash
ros2 launch lidar_imu_init motorized_mid360_wt901c.launch.py \
  result_dir:=/home/cuong/lm_calibr/data/li_init_results
```

The result path is `<result_dir>/Initialization_result.txt`. Debug logs default to `/tmp/lidar_imu_init/Log`; set `LIDAR_IMU_INIT_LOG_DIR` before launch to keep them elsewhere. Empty result files mean **no calibration result**, not a zero-valued calibration.

Limits: no actual extrinsic/time calibration, no independent transformation ground truth, no full upstream numerical equivalence test. The short timeout forced the smoke process to stop because SIGINT shutdown was slow. A sandboxed ROS launch initially failed to create UDP sockets; graph and replay checks were then run in an isolated domain with network access. Remaining Python 3.12 matplotlib deprecation warnings do not block the build.
