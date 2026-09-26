# FT-Hardware CAD Setup

SolidWorks setup is normally handled in person. This page is just the repo-side setup that people forget before they touch `FT-Hardware`.

## Git LFS
Before pulling or editing the CAD side of `FT-Hardware`, install Git LFS:

1. Go to [git-lfs.com](https://git-lfs.com/).
2. Install Git LFS.
3. Open PowerShell and run `git lfs install`.
4. Only then pull or clone the CAD repo.

This matters because the SolidWorks parts, assemblies, `STEP` files, and ZIP snapshots are stored through Git LFS. If you skip it, the CAD side of the repo will look broken or half-missing.

## Repo Location Warning
Do not keep your `FT-Hardware` clone inside a OneDrive-backed folder if you can avoid it.

Large CAD binaries, renames, deletes, and folder reshuffles do not mix well with background sync, and it turns normal CAD problems into much weirder ones.

If you already have local unpushed work, back it up before doing a big pull.

## Practical Notes
- Keep final `.SLDPRT`, `.SLDASM`, `.SLDDRW`, `STEP`, and print-export files in the repo structure rather than leaving them in personal folders.
- If Git LFS is not installed yet, stop and do that first.
- If the repo setup is fighting you, ask someone on the team before trying random workarounds.
