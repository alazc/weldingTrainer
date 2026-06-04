@echo off
REM Flash the inert "do nothing" sketch to BOTH boards (COM6 primary, COM7 secondary).
REM Parks every board in a safe state: empty setup/loop -> all GPIO high-Z input,
REM so nothing drives the motors, ERM, or I2C bus.
REM
REM Pre-flight: close the of-app and any Serial Monitor first, or upload fails
REM "access is denied" (the port is held open).
setlocal
set "CLI=C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "SKETCH=%~dp0arduino\welding_blank"

echo === Compiling welding_blank ===
"%CLI%" compile --fqbn arduino:avr:uno "%SKETCH%"
if errorlevel 1 (
  echo COMPILE FAILED — aborting, no boards flashed.
  exit /b 1
)

echo === Uploading to COM6 (primary) ===
"%CLI%" upload -p COM6 --fqbn arduino:avr:uno "%SKETCH%"
if errorlevel 1 echo WARNING: COM6 upload failed.

echo === Uploading to COM7 (secondary) ===
"%CLI%" upload -p COM7 --fqbn arduino:avr:uno "%SKETCH%"
if errorlevel 1 echo WARNING: COM7 upload failed.

echo === Done ===
endlocal
