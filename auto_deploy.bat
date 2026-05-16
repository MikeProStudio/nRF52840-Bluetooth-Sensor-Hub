@echo off
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%SCRIPT_DIR%
set UF2_PATH=
set TARGET_DRIVE=D:\

echo ==============================================
echo  Skynet AI - Auto Deploy to XIAO-SENSE
echo ==============================================
echo.

:: ── Step 1: Build ──
echo [1/3] Building firmware...
cd /d "%PROJECT_ROOT%"

call build.bat
if %errorlevel% neq 0 (
    echo [ERROR] Build failed with errorlevel=%errorlevel%
    pause
    exit /b %errorlevel%
)
echo [OK] Build successful
echo.

:: ── Step 2: Find .uf2 file ──
echo [2/3] Looking for .uf2 output...
for /r "%PROJECT_ROOT%\build" %%f in (*.uf2) do (
    set UF2_PATH=%%f
)
if "!UF2_PATH!"=="" (
    echo [ERROR] No .uf2 file found in build directory
    pause
    exit /b 1
)
echo [OK] Found: !UF2_PATH!
echo.

:: ── Step 3: Wait for XIAO-SENSE (D:\) and deploy ──
echo [3/3] Waiting for XIAO-SENSE on %TARGET_DRIVE%...
echo.
echo Put the board into UF2 bootloader mode (double-tap reset),
echo then press any key to continue...
echo.
pause >nul

:: Wait for the drive to appear (up to 30 seconds)
set WAIT_COUNT=0
:wait_drive
if exist "%TARGET_DRIVE%" (
    dir "%TARGET_DRIVE%" >nul 2>&1
    if !errorlevel! equ 0 goto found_drive
)
timeout /t 1 /nobreak >nul
set /a WAIT_COUNT+=1
if !WAIT_COUNT! geq 30 (
    echo [ERROR] %TARGET_DRIVE% not found after 30 seconds.
    echo         Make sure the board is in UF2 bootloader mode:
    echo         1. Connect the board via USB
    echo         2. Double-tap the RST button
    echo         3. A drive named XIAO-SENSE should appear
    pause
    exit /b 1
)
goto wait_drive

:found_drive
echo [OK] XIAO-SENSE detected on %TARGET_DRIVE%

:: Copy the .uf2 file
copy /y "!UF2_PATH!" "%TARGET_DRIVE%"
if %errorlevel% neq 0 (
    echo [ERROR] Failed to copy .uf2 to %TARGET_DRIVE%
    pause
    exit /b %errorlevel%
)

echo.
echo ==============================================
echo  [DONE] Firmware deployed successfully!
echo  Board will reboot and flash automatically.
echo ==============================================
echo.

endlocal
