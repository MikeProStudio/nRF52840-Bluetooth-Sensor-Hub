@echo off
setlocal

:: Set the path to the nRF Connect SDK toolchain
set NCS_TOOLCHAIN_PATH=C:\ncs\toolchains\c717907b94
set ZEPHYR_BASE=C:\ncs\v3.2.2\zephyr
set GNUARMEMB_TOOLCHAIN_PATH=%NCS_TOOLCHAIN_PATH%\opt

:: Add toolchain binaries to PATH
set PATH=%NCS_TOOLCHAIN_PATH%\opt\bin;%NCS_TOOLCHAIN_PATH%\opt\bin\Scripts;%PATH%

:: Set environment variables for Zephyr
set ZEPHYR_TOOLCHAIN_VARIANT=zephyr
set ZEPHYR_SDK_INSTALL_DIR=%NCS_TOOLCHAIN_PATH%\opt\zephyr-sdk

:: Activate the python environment
call %NCS_TOOLCHAIN_PATH%\cmd\env.cmd

:: Check if west is available now
where west
if %errorlevel% neq 0 (
    echo "West not found, trying to run cmake/ninja directly"
    
    :: Clean build directory
    if exist build rmdir /s /q build
    
    :: Run CMake
    cmake -S . -B build -GNinja -DBOARD=xiao_ble/nrf52840/sense
    if %errorlevel% neq 0 exit /b %errorlevel%
    
    :: Run Ninja
    ninja -C build
) else (
    echo "West found, building with west"
    west build -b xiao_ble/nrf52840/sense
)

endlocal