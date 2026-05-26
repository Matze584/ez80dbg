@echo off
REM Setup script for ez80dbg on Windows
setlocal

set VENV=venv

echo === ez80dbg setup ===

where python >/dev/null 2>&1
if errorlevel 1 (
    echo ERROR: python not found. Install Python 3.9+ from python.org
    pause
    exit /b 1
)

if not exist "%VENV%\" (
    echo Creating virtual environment...
    python -m venv %VENV%
)

echo Installing dependencies...
%VENV%\Scripts\pip install --upgrade pip -q
%VENV%\Scripts\pip install -r requirements.txt

echo.
echo === Windows USB driver note ===
echo pyftdi uses libusb, NOT the default FTDI D2XX driver.
echo You must replace the FTDI driver with WinUSB using Zadig:
echo   1. Download Zadig from https://zadig.akeo.ie/
echo   2. Connect the FT4232H board
echo   3. In Zadig: Options -^> List All Devices
echo   4. Select "FT4232H" interface B (or your device)
echo   5. Choose driver: WinUSB  -^> Install Driver
echo.
echo === Done ===
echo Run the tool:
echo   %VENV%\Scripts\python ez80dbg.py --help
echo   %VENV%\Scripts\python ez80dbg.py flash firmware.hex
echo   %VENV%\Scripts\python ez80dbg.py --gui
pause
