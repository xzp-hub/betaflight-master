@echo off
chcp 65001 >nul
cls

echo ==========================================
echo Betaflight Firmware Build Script
echo Target: SPEEDYBEEF405V4
echo ==========================================
echo.

set "ARM_GCC_PATH=E:\飞塔V4\arm-gnu-toolchain\arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi\bin"

if not exist "%ARM_GCC_PATH%\arm-none-eabi-gcc.exe" (
    echo [ERROR] ARM GCC compiler not found
    echo Path: %ARM_GCC_PATH%
    pause
    exit /b 1
)

echo [1/4] ARM GCC Path OK
echo.

if exist "C:\Program Files\Git\mingw64\bin\mingw32-make.exe" (
    set "MAKE_EXE=C:\Program Files\Git\mingw64\bin\mingw32-make.exe"
    goto :found_make
)

if exist "C:\Program Files\Git\bin\make.exe" (
    set "MAKE_EXE=C:\Program Files\Git\bin\make.exe"
    goto :found_make
)

echo [ERROR] Make tool not found
echo Please install Git for Windows
pause
exit /b 1

:found_make
echo [2/4] Make Tool: %MAKE_EXE%
echo.

set "PATH=%ARM_GCC_PATH%;%PATH%"

echo [3/4] Environment configured
echo.

cd /d "%~dp0"

echo [4/4] Building SPEEDYBEEF405V4 firmware...
echo This may take several minutes...
echo.

"%MAKE_EXE%" clean
"%MAKE_EXE%" SPEEDYBEEF405V4

if errorlevel 1 (
    echo.
    echo [ERROR] Build failed!
    pause
    exit /b 1
)

echo.
echo ==========================================
echo [SUCCESS] Firmware compiled successfully!
echo ==========================================
echo.

if exist "obj\*.hex" (
    dir /b "obj\*.hex"
)

echo.
pause
