# RC5 + Right Hand URDFs

`urdf_with_simple_collisions.urdf` is the canonical full robot URDF for runtime use.
It includes the RC5 arm, prehand D405 camera mesh, right hand, simplified collision boxes,
and the nominal D405 depth frames:

- `d405_depth_frame`
- `d405_depth_optical_frame`

The other URDFs are kept as variants:

- `Robot_with_right_hand_cor_fixed.urdf`: full arm, camera, and hand with mesh-heavy collisions.
- `Robot_with_right_hand_cor.urdf`: legacy full arm, camera, and hand variant.
- `robot_one_joint.urdf`: hand-only floating-wrist model for isolating hand/tracker behavior.
