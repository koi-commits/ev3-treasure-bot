@echo off
rem pbrun.cmd - one-shot: start the service, connect the EV3 over USB, run a
rem script, then stop the service again.
rem
rem Why one shot: this sandbox tears down the whole process tree when a shell
rem command ends, so the Pybricks background service cannot outlive a command
rem the way it normally would. Doing everything in one invocation is the
rem workable equivalent.
rem
rem Usage:  pbrun.cmd src\diag.py
setlocal
set "PROJ=%~dp0"
set "PB_RUNTIME_DIR=%PROJ%.pb\runtime"
set "PB_STATE_DIR=%PROJ%.pb\state"
set "USERPROFILE=%PROJ%.pb\home"
set "APPDATA=%USERPROFILE%\AppData\Roaming"
set "LOCALAPPDATA=%USERPROFILE%\AppData\Local"
set "PYTHONIOENCODING=utf-8"
set "PB_ENDPOINT=tcp://127.0.0.1:47823"
set "PB_SERVICE_CMD=D:\python\python.exe -m pb_service --idle-timeout 60"
set "PB_HUB_NAME=Hamster"
if not exist "%PB_RUNTIME_DIR%" mkdir "%PB_RUNTIME_DIR%"
if not exist "%PB_STATE_DIR%" mkdir "%PB_STATE_DIR%"
if not exist "%APPDATA%" mkdir "%APPDATA%"
if not exist "%LOCALAPPDATA%" mkdir "%LOCALAPPDATA%"

set "PBEXE=D:\python\Scripts\pb.exe"

"%PBEXE%" service start >nul 2>&1
"%PBEXE%" connect --hub %PB_HUB_NAME% --transport usb
if errorlevel 1 (
  echo [pbrun] connect failed
  exit /b 1
)

rem Clear any program left running by a previous (interrupted) session: the hub
rem refuses to store a new program while one is still resident ("The hub would
rem not store the program").
rem
rem Pass --hub explicitly: `pb stop` without it can leave the "hub in use"
rem selection cleared, which made the following `pb run` fail with
rem "No hub selected". Reconnecting afterwards is belt and braces.
"%PBEXE%" stop --hub %PB_HUB_NAME% >nul 2>&1
rem NB: `timeout` needs a console on stdin; under a redirected stdin it prints
rem "Input redirection is not supported". ping is the portable 1-second sleep.
ping -n 2 127.0.0.1 >nul
"%PBEXE%" connect --hub %PB_HUB_NAME% --transport usb >nul 2>&1

"%PBEXE%" run %* --hub %PB_HUB_NAME% --wait
set "RUNRC=%ERRORLEVEL%"

"%PBEXE%" service stop >nul 2>&1
exit /b %RUNRC%
