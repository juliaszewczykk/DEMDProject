@echo off
REM ==============================================================================
REM Reorganization Script for DEMDProject
REM Automatically organizes files into clean modular directories.
REM ==============================================================================

echo Creating project directories...
if not exist "src" mkdir src
if not exist "models" mkdir models
if not exist "docs" mkdir docs
if not exist "scripts" mkdir scripts

REM 1. Move and rename the academic PDF report
if exist "Electrical_machines_report (3).pdf" (
    echo Moving academic report to docs\Electrical_machines_report.pdf...
    move /Y "Electrical_machines_report (3).pdf" "docs\Electrical_machines_report.pdf" >nul
)

REM 2. Extract and place the Simulink model
if exist "Simulink_final.slx.zip" (
    echo Extracting Simulink_final.slx.zip into models\...
    powershell -command "Expand-Archive -Path 'Simulink_final.slx.zip' -DestinationPath 'models' -Force"
    move /Y "Simulink_final.slx.zip" "models\Simulink_final.slx.zip" >nul
)

REM 3. Clean up legacy / redundant files
if exist "Report_final.m.txt" (
    echo Removing Report_final.m.txt (source code is now located in src\Report_final.m)...
    del "Report_final.m.txt"
)
if exist "src\test.txt" (
    del "src\test.txt"
)

echo Reorganization completed successfully!
pause
