# Hardware

Hardware brought together the mechanical and electronics work behind Rigby and our competition sensor mount. The department ran during the 2025/26 season and finished its work as a separate team in August 2026. These pages keep the designs, build notes and lessons from that work together.

**Rigby is still being developed.** Its current [mechanical and software documentation](../rigby/overview.md) now has its own section. Start there if you are working on the test rig rather than looking back at the Hardware project.

## What We Built

The [competition mount](mechanical/mount/mount_overview.md) carried our sensors and compute hardware on the shared DDT car. The main changes were a removable internal rack, a printed weather shell, a supported camera stand and a compact switch enclosure. Brainy and the camera holder were printed for competition, followed by Lunchy's finished shell (July 2026).

Rigby's work covered the steering feedback and wheel encoder mounts, Platey and its electronics holders, the power and control hardware, and the assembled and exploded CAD. Drive and steering were demonstrated independently; work then continued on the vehicle interface, managed firmware and remote control in [FT-Rigby](https://github.com/FT-Autonomous/FT-Rigby). That ongoing work is not being marked complete just because Hardware has finished.

## Finding Things

| Looking for | Start here |
| --- | --- |
| Competition mount layout and component names | [Mechanical overview](mechanical/mechanical_overview.md) |
| Lunchy, Brainy, Zeddy or SwitchyJr | [Competition mount](mechanical/mount/mount_overview.md) |
| Original wiring, controller and power-system notes | [Electronics archive](electronics/electronics_overview.md) |
| CAD checkout and Git LFS | [FT-Hardware CAD setup](mechanical/solidworks_setup.md) |
| Current Rigby hardware, software and testing | [Rigby](../rigby/overview.md) |

The CAD remains in [FT-Hardware](https://github.com/FT-Autonomous/FT-Hardware). The local working copy contains later, unpushed files, so a fresh GitHub clone does not yet reproduce every layout shown here. In particular, Rigby's later exploded assembly and its full assembly still need reconciling. The [CAD guide](../rigby/cad.md) explains which files to compare.

## Earlier Rigby Work

Rigby began in summer 2023, before Hardware became a department. The 2023/24 work included a steering motor change and early steering-angle feedback. The following season focused on steering geometry, rewiring, feedback sensors and separate controllers for the motors. The 2025/26 work built on that rather than starting with a new vehicle.

These archived pages describe the arrangements used during the project. Older board names and wiring examples remain useful history, but are not instructions to replace the current Rigby installation.
