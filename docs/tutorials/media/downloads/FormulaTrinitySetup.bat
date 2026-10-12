@echo off
setlocal EnableExtensions EnableDelayedExpansion

title Formula Trinity Autonomous Setup

:: ============================================================
:: Formula Trinity Windows / WSL Bootstrapper
::
:: Target:
::   Windows 10/11
::   WSL2
::   Ubuntu 22.04
::
:: IMPORTANT:
::   - Existing FT installations are checked BEFORE anything
::     is downloaded or modified.
::   - Windows Administrator access is requested only when
::     WSL / Ubuntu needs to be installed.
:: HOW TO USE:
::   - Simply download, run the program and follow all the 
::     instructions
::
:: Please contact Muirinn if there are any issues
:: ============================================================


:: ------------------------------------------------------------
:: Configuration
:: ------------------------------------------------------------

set "DISTRO=Ubuntu-22.04"

:: ------------------------------------------------------------
:: Header
:: ------------------------------------------------------------

cls

echo.
echo ============================================================
echo              FORMULA TRINITY AUTONOMOUS
echo                    SETUP INSTALLER
echo ============================================================
echo.
echo This installer will check your computer before making
echo any changes.
echo.
echo If the Formula Trinity suit is already installed, you will 
echo be given the option to exit without modifying anything.
echo.
echo ============================================================
echo.


:: ============================================================
:: STEP 1 - Check WSL
:: ============================================================

echo [1/6] Checking WSL...
echo.

where wsl.exe >nul

if %errorlevel% neq 0 (

    echo [INFO] WSL is not installed.
    echo.

    goto :INSTALL_WSL
)

echo [OK] WSL is available.


:: ============================================================
:: STEP 2 - Check WSL configuration
:: ============================================================

echo.
echo [2/6] Checking WSL configuration...
echo.

wsl --status >nul

if %errorlevel% neq 0 (

    echo [WARNING] WSL exists but does not appear to be
    echo fully configured.
    echo.

    goto :INSTALL_WSL
)

echo [OK] WSL is configured.


:: ============================================================
:: STEP 3 - Check Ubuntu 22.04
:: ============================================================

echo.
echo [3/6] Checking for %DISTRO%...
echo.

:: ------------------------------------------------------------
:: Rather than parsing "wsl --list", directly ask WSL to run
:: a harmless command inside the requested distribution.
::
:: This avoids encoding problems with the output of wsl -l.
:: Nothing is installed or modified by this test.
:: ------------------------------------------------------------

wsl.exe -d %DISTRO% -- /bin/true >nul

if %errorlevel% equ 0 (
    echo [OK] %DISTRO% is installed.
    goto :UBUNTU_FOUND
)

echo [INFO] %DISTRO% was not found.
echo.
echo Installed WSL distributions:
echo.
wsl.exe --list --verbose
echo.
echo The installer expected a distribution named:
echo.
echo     %DISTRO%
echo.
echo If Ubuntu 22.04 appears above under a different name,
echo the installer should not install another copy.
echo.

pause
goto :INSTALL_UBUNTU


:UBUNTU_FOUND
:: ============================================================
:: STEP 4 - Check Ubuntu
:: ============================================================

echo.
echo [4/6] Checking Ubuntu Setup...
echo.

set "CHECK_FILE=%TEMP%\ft_wsl_check_%RANDOM%.txt"

wsl -d %DISTRO% -- bash -lc "printf 'FT_WSL_READY\n'" > "%CHECK_FILE%" 

findstr /C:"FT_WSL_READY" "%CHECK_FILE%" >nul 

if %errorlevel% neq 0 (

    del "%CHECK_FILE%" >nul

    echo [WARNING] Ubuntu is installed but could not be
    echo started normally.
    echo.
    echo Ubuntu may not have completed its first-time setup.
    echo.
    echo An Ubuntu terminal will now be opened.
    echo.
    echo If prompted, create your Linux username and password.
    echo.
    echo When finished, close Ubuntu and run this installer
    echo again.
    echo.

    pause

    start "" wsl.exe -d %DISTRO%

    exit /b 0
)

del "%CHECK_FILE%" >nul 

echo [OK] Ubuntu is ready.

:: ============================================================
:: STEP 5 - Check if ~/ft exists in WSL
:: ============================================================
echo.
echo [5/6] Checking Formula Trinity directory...
echo.

wsl.exe -d %DISTRO% -- bash -c "test -d ~/ft"

if not errorlevel 1 goto :FT_FOLDER_EXISTS

:: Directory does not exist - create it
wsl.exe -d %DISTRO% -- bash -c "mkdir -p ~/ft"

if errorlevel 1 goto :ERROR_EXIT

echo [OK] ~/ft directory created
goto :FT_FOLDER_READY

:FT_FOLDER_EXISTS
echo [OK] ~/ft directory found

:FT_FOLDER_READY

:: ============================================================
:: STEP 6 - READ-ONLY Formula Trinity installation check
:: ============================================================

echo.
echo [5/6] Checking existing Formula Trinity suite installation...
echo.

:: ------------------------------------------------------------
:: IMPORTANT:
:: These checks are READ ONLY.
:: Nothing is downloaded, installed or modified.
::
:: Each component is checked separately. This avoids complex
:: Bash commands being parsed incorrectly by Windows CMD.
:: ------------------------------------------------------------

set "ROS_OK=0"
set "REPO_OK=0"
set "WORKSPACE_OK=0"
set "EUFS_OK=0"
set "SYSTEM_OK=0"


:: ------------------------------------------------------------
:: Check ROS2 Humble
:: ------------------------------------------------------------

wsl.exe -d %DISTRO% -- test -f /opt/ros/humble/setup.bash >nul 

if not errorlevel 1 (
    set "ROS_OK=1"
)

:: ------------------------------------------------------------
:: Check FT-FSAI-Core repository
:: ------------------------------------------------------------

wsl.exe -d %DISTRO% -- bash -c "test -d \"$HOME/ft/FT-FSAI-Core/.git\"" >nul 

if not errorlevel 1 (
    set "REPO_OK=1"
)


:: ------------------------------------------------------------
:: Check built ROS workspace
:: ------------------------------------------------------------

wsl.exe -d %DISTRO% -- bash -c "test -f ~/ft-wsl/install/setup.bash" >nul
if not errorlevel 1 (
    set "WORKSPACE_OK=1"
)

:: ------------------------------------------------------------
:: Check EUFS package
::
:: Only attempt this if ROS and the workspace exist.
:: ------------------------------------------------------------

if "!ROS_OK!"=="1" if "!WORKSPACE_OK!"=="1" (

    wsl.exe -d %DISTRO% -- bash -c "source ~/ft-wsl/environment.bash && ros2 pkg prefix eufs_launcher" >nul
    if not errorlevel 1 (
        set "EUFS_OK=1"
    )
)


:: ------------------------------------------------------------
:: Check FT simulation package
:: ------------------------------------------------------------

if "!WORKSPACE_OK!"=="0" goto :SYSTEM_CHECK_DONE

wsl.exe -d %DISTRO% -- bash -c "source ~/ft-wsl/environment.bash && ros2 pkg prefix ft_launch" >nul 2>&1

if errorlevel 1 goto :SYSTEM_CHECK_DONE

set "SYSTEM_OK=1"

:SYSTEM_CHECK_DONE

:: ------------------------------------------------------------
:: Display results
:: ------------------------------------------------------------

echo.
echo Formula Trinity components:
echo.

if "!ROS_OK!"=="1" (
    echo     [OK] ROS2 Humble
) else (
    echo     [MISSING] ROS2 Humble
)

if "!REPO_OK!"=="1" (
    echo     [OK] FT-FSAI-Core repository
) else (
    echo     [MISSING] FT-FSAI-Core repository
)

if "!WORKSPACE_OK!"=="1" (
    echo     [OK] ROS workspace build
) else (
    echo     [MISSING] ROS workspace build
)

if "!EUFS_OK!"=="1" (
    echo     [OK] EUFS
) else (
    echo     [MISSING] EUFS
)

if "!SYSTEM_OK!"=="1" (
    echo     [OK] FT system launch
) else (
    echo     [MISSING] FT system launch
)
echo.

:: ============================================================
:: Determine whether installation is complete
:: ============================================================

if "!ROS_OK!"=="1" if "!REPO_OK!"=="1" if "!WORKSPACE_OK!"=="1" if "!EUFS_OK!"=="1" if "!SYSTEM_OK!"=="1" (

    goto :ALREADY_INSTALLED
)


:: ============================================================
:: Installation incomplete
:: ============================================================

echo.
echo ============================================================
echo              INSTALLATION INCOMPLETE
echo ============================================================
echo.
echo One or more Formula Trinity components are missing.
echo.
echo Unfortunatly there is an issue with git (windows 11 and WSL2) 
echo so please follow these instructions for Windows 11
echo https://www.freecodecamp.org/news/github-cli-wsl2-guide/
echo to set up an SSH key and then clone the FT-FSAI-Core repo.
echo.
echo Follow these instructions if you have Windows 10
echo https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent
echo.
echo How to clone FT-FSAI-Core repo
echo wsl
echo cd ~/ft
echo git clone --recurse-submodules git@github.com:FT-Autonomous/FT-FSAI-Core.git
echo.
echo Options:
echo.
echo    [1] FT-FSAI-Core is cloned and I am ready for the next step
echo    [2] Exit without making changes

choice /C 12 /N /M "Select an option [1-2]: "

if errorlevel 2 goto :SAFE_EXIT
if errorlevel 1 goto :ASK_INSTALLER

:ASK_INSTALLER
echo ============================================================
echo              INSTALLATION INCOMPLETE
echo ============================================================
echo The setup process can run install.sh which will install 
echo missing components of the suite.
echo.
echo No Formula Trinity files have been changed yet.
echo.
echo Options:
echo.
echo     [1] Continue setup
echo     [2] Exit without making changes
echo.

choice /C 12 /N /M "Select an option [1-2]: "

if errorlevel 2 goto :SAFE_EXIT
if errorlevel 1 goto :RUN_INSTALLER


:: ============================================================
:: Already installed
:: ============================================================

:ALREADY_INSTALLED

echo.
echo ============================================================
echo         FORMULA TRINITY SUITE IS ALREADY INSTALLED
echo ============================================================
echo.
echo All checked components were found.
echo No files have been downloaded or modified.
echo.
echo Options:
echo.
echo     [1] Exit
echo.
echo     [2] Run install.sh anyway
echo.

choice /C 12 /N /M "Select an option [1-2]: "

if errorlevel 2 goto :RUN_INSTALLER
if errorlevel 1 goto :SAFE_EXIT


:: ============================================================
:: Safe exit
:: ============================================================

:SAFE_EXIT

echo.
echo ============================================================
echo                  NO CHANGES MADE
echo ============================================================
echo.
echo Formula Trinity setup has been cancelled.
echo.

pause
exit /b 0


:: ============================================================
:: Run Linux installer
:: ============================================================
:RUN_INSTALLER

echo.
echo ============================================================
echo                FORMULA TRINITY SUITE
echo ============================================================
echo The Formula Trinity install.sh script will now be run within
echo the wsl and it will download the Formula Trinity suite.
echo.

:: uses Jakubs code assuming its in FT-FSAI-Core
wsl.exe -d Ubuntu-22.04 -- bash -c "cd ~/ft/FT-FSAI-Core && ls && bash setup.sh"

:: ============================================================
:: Installation successful
:: ============================================================

echo.
echo ============================================================
echo              FORMULA TRINITY SETUP COMPLETE
echo ============================================================
echo.
echo Formula Trinity has been installed successfully.
echo.
echo ============================================================
echo                        HOW TO RUN
echo ============================================================
echo.
echo Open cmd and send the command wsl to open a Ubuntu terminal
echo.
echo Open 3 wsl terminals
echo Run the following command in them all
echo.
echo source ~/ft-wsl/environment.bash
echo.
echo Then enter the following commands in each terminal
echo Terminal 1:
echo ros2 launch eufs_launcher eufs_launcher.launch.py
echo.
echo Choose and launch a circuit in EUFS. Keep that terminal running.
echo.
echo Termial 2:
echo ros2 launch ft_launch system_sim.launch.py
echo.
echo Terminal 3, to select Autocross on a suitable circuit:
echo ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState '{ami_state: 13}'
echo ros2 topic echo /ros_can/state --once
echo.
echo Autocross is mission 13; driving is AS state 2. This uses the simulator's
echo mission service rather than publishing a fake driving state.
echo.
echo ============================================================
echo.

pause
exit /b 0



:: ============================================================
:: Install WSL
:: ============================================================

:INSTALL_WSL

echo ============================================================
echo                       WSL SETUP
echo ============================================================
echo.
echo WSL needs to be installed or configured.
echo.
echo Windows Administrator privileges are required for this
echo Windows-level operation.
echo.
echo The Formula Trinity installer itself will NOT run as
echo Windows Administrator.
echo.
echo Options:
echo.
echo     [1] Continue setup
echo     [2] Exit without making changes
echo.

choice /C 12 /N /M "Select an option [1-2]: "

if errorlevel 2 goto :SAFE_EXIT
if errorlevel 1 goto :WSL_INSTALLER
echo.

:WSL_INSTALLER
powershell -NoProfile -ExecutionPolicy Bypass ^
    -Command "Start-Process -FilePath 'wsl.exe' -ArgumentList '--install','-d','%DISTRO%' -Verb RunAs -Wait"

if %errorlevel% neq 0 (

    echo.
    echo [ERROR] WSL installation did not complete successfully.
    echo.

    pause
    exit /b 1
)

:: ============================================================
:: Install Ubuntu
:: ============================================================

:INSTALL_UBUNTU

echo ============================================================
echo                     UBUNTU SETUP
echo ============================================================
echo.
echo %DISTRO% needs to be installed.
echo.
echo Windows may request Administrator privileges for this
echo operation.
echo.
echo This terminal may close after installation. If this happens
echo please run the program again.
echo.
echo Press any key to continue or close this window to cancel.
echo.
powershell wsl --install %DISTRO%

if %errorlevel% neq 0 (

    echo.
    echo [ERROR] Ubuntu installation did not complete
    echo successfully.
    echo.

    pause
    exit /b 1
)


echo.
echo ============================================================
echo                 UBUNTU INSTALLATION COMPLETE
echo ============================================================
echo.
echo Ubuntu now needs its first-time setup.
echo.
echo Create your Linux username and password when prompted.
echo.
echo Once finished:
echo.
echo     1. Close Ubuntu
echo     2. Run this installer again
echo.

pause

start "" wsl.exe -d %DISTRO%

exit /b 0

:: ============================================================
:: FAIL
:: ============================================================
:fail
echo.
echo ================================================================
echo [FAILED] Formula Trinity setup did not complete.
echo ================================================================
echo.
echo This window will remain open so you can read the error.
pause
exit /b 1
