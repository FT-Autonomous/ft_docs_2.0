# Mechanical

Mechanical handled the CAD, printed parts and physical packaging for Hardware. There were two closely related jobs: rebuilding the competition sensor mount and making Rigby easier to assemble, maintain and test.

The competition mount project is recorded here as completed work. [Rigby](../../rigby/overview.md) continues separately, including its newer control software and drivetrain work.

## Competition Mount

The mount was designed around a simple split: Brainy held the equipment, Lunchy sheltered it, and the sensors had their own mounting points on the main plate. That let us remove the shell without dismantling the compute package, and replace a damaged print without remaking the entire mount.

| Part | Job |
| --- | --- |
| [Lunchy](mount/lunchy.md) | Weather shell, removable lid and rear cable cover |
| [Brainy](mount/brainy.md) | Internal rack for the compute and navigation hardware |
| [Zeddy](mount/camera_module.md) | Supported stand for the Zed2i camera |
| [LiDAR mount](mount/lidar.md) | Direct attachment of the VLP-16 to the plate |
| [SwitchyJr](mount/power_control.md) | Power-control enclosure and its underside clamp |

Start with the [mount overview](mount/mount_overview.md) for the overall arrangement. The [reference sketches](mount/reference_sketches.md) explain the earlier concepts, while the [2026 DDT checks](mount/fs_ai_2026_mount_checks.md) record what needed demonstrating for the competition installation.

## Working From the Files

The [CAD setup](solidworks_setup.md) covers Git LFS and the OneDrive warning. SolidWorks itself is taught in person. The [printing reference](3dprinting.md) explains the source/export split and what to keep when reproducing a part.

The submitted competition assembly is a snapshot, not a file to quietly update. Later working assemblies and print releases need to be compared with it when tracing what changed. File names such as `Final` or `ReleaseCandidate` are not, on their own, evidence that a part was fitted or tested.
