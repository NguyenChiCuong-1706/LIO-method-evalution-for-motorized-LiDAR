# LI-Init pipeline and frame convention

1. `src/laserMapping.cpp::livox_pcl_cbk` receives `/calibrated_livox`. `src/preprocess.cpp::process_cut_frame_livox` selects MID-360 points, retains corrected XYZ, uses Livox nanosecond `offset_time` as millisecond `curvature`, and splits each ~100 ms frame into up to five subframes after startup.
2. `src/laserMapping.cpp::sync_packages` pairs each subframe with IMU samples by `header.stamp`. `src/laserMapping.cpp::timer_callback`, `map_incremental`, and the filter's LiDAR-only propagation build odometry and the initial map. The initial map and movement threshold produce LiDAR pose/velocity samples.
3. `src/laserMapping.cpp::imu_cbk` receives `/imu/data_raw`, reads acceleration (m/s²) and gyro (rad/s), subtracts currently estimated time shifts from the IMU stamp, and feeds `LI_Init::push_ALL_IMU_CalibState`. Orientation quaternion is not an input to these estimates.
4. `LI_Init::data_sufficiency_assess` determines whether excitation and accumulation are adequate. `LI_Init::LI_Initialization` then interpolates/filters IMU samples, runs `xcorr_temporal_init`, `solve_Rotation_only`, `solve_Rot_bias_gyro` (including second time-delay term), `acc_interpolate`, and `solve_trans_biasacc_grav`.
5. `src/laserMapping.cpp::timer_callback` transfers the estimates into `state`, changes to IMU-enabled propagation and online refinement, then `fileout_calib_result` writes initialization and refinement results.

The transform is **from the corrected cloud's motor-base coordinate system M into WT901C measurement coordinates I**:

`p_I = R_IM p_M + t_IM`.

This direction follows the actual `pointBodyToWorld` use of `state.offset_R_L_I * p_body + state.offset_T_L_I`, and `LI_init.cpp`'s conversion from its solved opposite translation: `Trans_Lidar_wrt_IMU = -Rot_Lidar_wrt_IMU * Trans_IL_vec`. FAST-LIO2 `mapping.extrinsic_R` is the row-major 3×3 `R_IM`; `mapping.extrinsic_T` is the three-element `t_IM` in metres. Do not use the inverse transform.

Reported time lag is `time_lag_IMU_wtr_lidar + timediff_imu_wrt_lidar`, in seconds. The implementation **subtracts that value from IMU `header.stamp`** to align it with LiDAR time; positive reported lag therefore means shifting IMU stamps earlier. Host-reception WT901C stamps make the estimate a combined sensor/driver/transport delay, not a hardware clock calibration.

Only the obsolete Ceres quaternion API changed in optimization construction. Rotation residuals, accelerometer residuals, Jacobians, observability thresholds, time correlation and estimation states were not edited. [Original upstream](https://github.com/hku-mars/LiDAR_IMU_Init) uses the old Ceres API and the same named estimation stages; a full numerical comparison has not been run.
