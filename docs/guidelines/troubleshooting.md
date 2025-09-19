# General Troubleshooting Guide

- Make sure all of your packages are up to date. Navigate to `FT-FSAI-23/FT-FastSLAM2023/` and `git switch main` and `git pull` on every repository unless you are explicitly working with a specific package version. For debugging purposes, when communicating an issue with somebody, you will want to provide people with the versions of each package in your current setup so that they can replicate your setup exactly locally.

- If you are having issues visualising cones or way points from path planning. It is likely that you're install of `eufs_rviz_plugins` is not configured correctly. Make sure that you've compiled `eufs_rviz_plugins` and sourced it.

Be sure to make a PR adding anything to this list if you have any practices that help you troubleshoot things!