# Validation Report: marav3best

## Status: PASS

## Summary

| Metric | Value |
|--------|-------|
| Links | 12 |
| Joints | 11 |
| Assemblies | 4 |
| Root | `base_link` |
| Errors | 0 |
| Warnings | 0 |

## Kinematic Tree

```
base_link [BAKE] [visual_reuse]
  └─ J1_yaw_base_shoulderLink [continuous]
    RG_shoulder_link [BAKE] [visual_reuse]
      └─ J2_pitch_shoulder [continuous]
        RG_shoulder_arm [BAKE] [visual_reuse]
          └─ J3_rigid_shoulderArm_elbowLink [fixed]
            RG_elbow_link [visual_reuse]
              └─ J4_pitch_elbow [continuous]
                RG_elbow_arm [BAKE] [visual_reuse]
                  └─ J5_rigid_elbowArm_wristLink [fixed]
                    RG_wrist_link [visual_reuse]
                      └─ J6_pitch_wrist [continuous]
                        RG_wrist_arm [BAKE] [visual_reuse]
                          └─ J9_horn_drive [continuous]
                            RG_gripper_horn [BAKE] [visual_reuse]
                              └─ J10_horn_link1 [continuous]
                                marav3best_gripper_2nd_link [BAKE] [visual_reuse]
                                  └─ J12_link1_tooth1 [continuous]
                                    marav3best_gripper [BAKE] [visual_reuse]
                              └─ J11_horn_link2 [continuous]
                                marav3best_gripper_2nd_link_2 [BAKE] [visual_reuse]
                                  └─ J13_link2_tooth2 [continuous]
                                    marav3best_gripper_2 [BAKE] [visual_reuse]
```

## Collision Geometry

| Link | Source | Shape/File |
|------|--------|------------|
| `RG_elbow_arm` | visual_reuse | — |
| `RG_elbow_link` | visual_reuse | — |
| `RG_gripper_horn` | visual_reuse | — |
| `RG_shoulder_arm` | visual_reuse | — |
| `RG_shoulder_link` | visual_reuse | — |
| `RG_wrist_arm` | visual_reuse | — |
| `RG_wrist_link` | visual_reuse | — |
| `base_link` | visual_reuse | — |
| `marav3best_gripper` | visual_reuse | — |
| `marav3best_gripper_2` | visual_reuse | — |
| `marav3best_gripper_2nd_link` | visual_reuse | — |
| `marav3best_gripper_2nd_link_2` | visual_reuse | — |

## Mesh Bake Offsets

Links where joint frame ≠ component origin. Visual/inertial/collision origins shifted.

| Link | Offset (mm) |
|------|-------------|
| `base_link` | (0.0, 0.0, 0.0) |
| `RG_shoulder_link` | (-50.0, 0.0, -65.5) |
| `RG_gripper_horn` | (0.0, 0.0, 18.0) |
| `marav3best_gripper_2nd_link` | (13.0, -0.2, -0.3) |
| `marav3best_gripper_2nd_link_2` | (-13.0, 0.2, -0.3) |
| `marav3best_gripper` | (19.0, -16.0, -6.2) |
| `marav3best_gripper_2` | (-19.0, 16.0, -6.2) |
| `RG_wrist_arm` | (0.0, 0.0, -30.0) |
| `RG_shoulder_arm` | (0.0, 0.0, -29.0) |
| `RG_elbow_arm` | (0.0, 0.0, -30.0) |
