# Quaternion numerical equivalence

The former Ceres `QuaternionParameterization` and current `QuaternionManifold` both use scalar-first `[w,x,y,z]` and left perturbation `q(δ)=[cos(|δ|), sin(|δ|)δ/|δ|] ⊗ q`. Eigen's constructor explicitly receives those four scalars in the same order; its internal coefficient storage order is not used as the Ceres parameter block.

`test/test_precalibration.cpp` checks identity, ±90° on X/Y/Z, near-zero, arbitrary axes, and near-180° rotations. It checks `Minus(Plus(q,δ),q)=δ`, the independently formed Eigen left product, unit norm, and the angular-velocity residual's `3×4` autodiff Jacobian multiplied by Ceres' `4×3` Plus Jacobian against central differences with step `1e-7`. Plus/Minus and norm tolerances are `1e-12`; tangent Jacobian tolerance is `1e-7`.

These tests establish equivalence of the tested manifold and rotation residual. They do not prove every optimization residual or final estimated extrinsic is equivalent to an original ROS 1 run. No solver equation or threshold was changed.
