@echo off
rem pb.cmd - run the Pybricks CLI with every runtime/log/state path kept inside
rem the project tree, and over TCP loopback instead of a named pipe.
rem
rem Why: the DSH sandbox only allows writes under the workspace, and it refuses
rem to open Windows named pipes. Both are worked around here; nothing about the
rem hub connection itself changes.
rem
rem PB_SERVICE_CMD also disables the service's idle timeout, so the hub stays
rem connected between separate pb invocations.
setlocal
set "PROJ=%~dp0"
set "PB_RUNTIME_DIR=%PROJ%.pb\runtime"
set "PB_STATE_DIR=%PROJ%.pb\state"
set "USERPROFILE=%PROJ%.pb\home"
set "APPDATA=%USERPROFILE%\AppData\Roaming"
set "LOCALAPPDATA=%USERPROFILE%\AppData\Local"
set "PYTHONIOENCODING=utf-8"
set "PB_ENDPOINT=tcp://127.0.0.1:47823"
set "PB_SERVICE_CMD=D:\python\python.exe -m pb_service --idle-timeout 0"
if not exist "%PB_RUNTIME_DIR%" mkdir "%PB_RUNTIME_DIR%"
if not exist "%PB_STATE_DIR%" mkdir "%PB_STATE_DIR%"
if not exist "%APPDATA%" mkdir "%APPDATA%"
if not exist "%LOCALAPPDATA%" mkdir "%LOCALAPPDATA%"
"D:\python\Scripts\pb.exe" %*
exit /b %ERRORLEVEL%
