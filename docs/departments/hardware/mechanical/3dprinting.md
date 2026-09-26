# 3D Printing Reference

The Hardware build used a mixture of test prints, release candidates and saved printer projects. Keeping those files was useful, but it also left several similar-looking parts. Before printing, work out which assembly the part belongs to and whether the export actually matches it.

## Finding the Files

| Project | Source CAD | Print files |
| --- | --- | --- |
| Rigby mechanisms | `CAD/Rigby/Designed Parts (reproducable)/` | `Part Print Files/` beneath that folder |
| Platey and holders | `CAD/Rigby/Platey - Electronic Mounting/` | `Part Prints/`, plus SwitchySr's own print folder |
| Competition mount | `CAD/Mount/` component folders | Exports beside the parts, prototype folders and `Switchy Jr/PrintParts/` |

Rigby's mechanism exports are split into general `STL`, and `Bambu A1` / `Prusa` folders containing `3mf` and `gcode`.

The competition mount was adjusted through its final prints, so its exports remained spread across the working folders. A standalone mount print folder, matching Rigby's arrangement, is still a useful handover task. It has **not** been released yet; do not mistake an old prototype folder for the complete competition print pack.

The newer [R4 gearbox fit prototype](../../rigby/drivetrain.md#r4-flat-stop-gear-prototype) is separate local work in FT-Rigby's ignored artifacts, not part of the published Hardware print collection.

## Reproducing a Part

1. Identify the fitted part and its matching SolidWorks source. Check the [Rigby CAD caveat](../../rigby/cad.md) before choosing an exploded-model copy.
2. Export in millimetres and check the dimensions before slicing. Do not scale an entire mechanical part to fix one clearance.
3. Save the STL and the printer project together. The 3MF should preserve material, orientation, supports and local modifiers.
4. Record the source revision and the outcome: failed print, dry fit, fitted part or tested part.
5. Keep machine-specific G-code only with enough information to identify its printer, nozzle and material. Re-slice when that setup changes.

A successful print is not the same as a successful fit, and a successful fit is not a load test.

## Competition Prints

Lunchy was designed for PETG around warm electronics and outdoor exposure. Its removable panels, sheltered vents and rear cover are described on the [Lunchy page](mount/lunchy.md). No blanket waterproofing or temperature rating follows from choosing PETG.

Have the M6 nuts and other insert hardware ready before printing interfaces that capture them. Check the pockets, layer adhesion and bolt bearing areas before fitting. Tightening a bolt against damaged layers can hide a problem until the part is loaded.

Zeddy and SwitchyJr were smaller prints, but their orientation and fit still affected camera stiffness, clamp support and cable access. Use the same assembly checks as for a larger part.

## Keeping the Archive Useful

Keep useful design iterations, but label what they were for. Do not delete local CAD or assume that an ignored folder is backed up merely because the repository has been pushed. The eventual print pack should contain a short parts list and matched CAD/export revisions, not every intermediate slicer file.

