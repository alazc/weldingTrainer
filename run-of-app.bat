@echo off
REM Launch the welding trainer of-app against the primary on COM6.
REM cwd is forced to of-app\bin so openFrameworks finds bin\data\ (fonts, paths).
REM No args  -> trainer mode, serial:COM6 (the primary).
REM With args-> passed straight through, e.g.:
REM   run-of-app.bat --source=mouse
REM   run-of-app.bat --mode=linkage --source=replay:..\..\session.csv
setlocal
cd /d "%~dp0of-app\bin"
if "%~1"=="" (
  of-app.exe --mode=trainer --source=serial:COM6
) else (
  of-app.exe %*
)
endlocal
