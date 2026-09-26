# CAD and Printing

The CAD remains in [FT-Hardware](https://github.com/FT-Autonomous/FT-Hardware/tree/main/CAD/Rigby), even though Rigby's maintained software now has its own repository.

**The local files are not fully reconciled or pushed.** In particular, the later exploded model includes changes that have not all been carried back into the full assembly. Do not discard it as "just a presentation" or assume a fresh clone is more current than the working copy.

## Assemblies to Open

Paths are relative to `FT-Hardware/CAD/Rigby/` and reflect the local checkout checked on 26 September 2026.

| File | What it is useful for |
| --- | --- |
| `Rigby.SLDASM` | Full assembled layout; still needs changes brought back from the later exploded work |
| `RigbyDrawings.SLDDRW` / `RigbySketch.SLDDRW` | Drawing references |
| `Front Steering System/Front Steering.SLDASM` | Steering mechanism |
| `Platey - Electronic Mounting/PlateyAssembly.SLDASM` | Electronics plate and holders |
| `Platey - Electronic Mounting/Platey Exploded Assembly.SLDASM` | Platey assembly relationships |
| `Rigby Exploded Visual/Rigby Exploded Cleavon Adjustments WIP .SLDASM` | Later exploded overview to compare with the full model |

There is a space before `.SLDASM` in the last filename. The matching drawing and image live beside it.

The exploded folders contain many intermediate copies, presentation parts and duplicated component trees. They were not cleaned up for this documentation update. A duplicate filename does not tell you whether its geometry is identical.

## Folder Map

| Folder | Contents |
| --- | --- |
| `Designed Parts (reproducable)` | Our mechanism parts and brackets |
| `Designed Parts (reproducable)/Part Print Files` | STL files and Bambu A1/Prusa projects and G-code |
| `Bought Parts (to be replaced)` | Purchased/reference geometry, including the golf-trolley motor model |
| `Front Steering System` | Screw, nut, support and linkage assembly |
| `Platey - Electronic Mounting` | Plate, electronics holders, SwitchySr and their print files |
| `Drawings` | Drawing exports and supporting references |
| `Rigby Exploded Parts` / `Rigby Exploded Visual` | Exploded-assembly work and presentation copies |
| `Historical Files` | Older builds kept for comparison |

The spelling `reproducable` is retained because that is the actual folder name.

## Before Editing or Printing

1. Follow the [Git LFS setup](../hardware/mechanical/solidworks_setup.md), and keep the CAD clone outside OneDrive.
2. Back up unpushed work before pulling or reorganising anything. The latest working files are not all on GitHub.
3. Open the full and later exploded assemblies together. Note which referenced part each one actually uses.
4. Compare the relevant part with the real vehicle. Check mates, scale, clearances and whether the exploded copy includes a genuine design change or only a display change.
5. Make the change in the intended source, then update its assembly and export. Record the relationship rather than silently overwriting every duplicate.

The [printing reference](../hardware/mechanical/3dprinting.md) covers materials, slicer projects and retained test prints. Old G-code is machine-specific, not a universal shortcut.

## Gearbox Prototype Files

The September R4 gearbox design is currently outside the published CAD collection. It is in the local FT-Rigby checkout under:

`local-artifacts/outputs/gear-replacement-20260920/`

That directory is ignored by Git. A clone of FT-Rigby will not contain it, and a push does not back it up. The [drivetrain page](drivetrain.md) lists the exact R4 files and what was checked. Do not force-add the entire artifacts folder just to share one design.

## Drawings in These Docs

The overview and chassis images are original exports from `Rigby Pictures Disassembled`, used without cropping. They help explain the assembly, but retain their original title blocks and older presentation geometry. They are not a new as-built survey.

{Add a reconciled assembled view once the latest exploded-model changes have been carried back into Rigby.SLDASM.}
