# Formula Trinity Windows Setup

## HOW TO USE

- Download [FormulaTrinitySetup.bat](/media/downloads/FormulaTrinitySetup.bat), run it, and follow the on-screen instructions.
- Please contact Muirinn if there are any issues.

## Target

- Windows 10/11
- WSL2
- Ubuntu 22.04

## Important

- Existing Formula Trinity installations are checked **before anything is downloaded or modified**.
- Windows Administrator access is requested only when WSL or Ubuntu needs to be installed.

## Git issue

- There is currently a reported Git issue with Windows 11 and WSL2. Therefore, the `FT-FSAI-Core` repository is not cloned as part of setup and must be cloned manually partway through the setup process.
- An SSH key must be configured. See the following guides:
  - [Windows 10: Git Credentials](https://docs.formulatrinity.ie/resources/git/)
  - [Windows 11: GitHub CLI guide](https://www.freecodecamp.org/news/github-cli-wsl2-guide/)
- These steps are also described in the program instructions.
