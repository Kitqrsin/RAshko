@echo off
setlocal
pushd "%~dp0"
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0raspberry.ps1" %*
set "RASHKO_EXIT_CODE=%ERRORLEVEL%"
popd
if not "%RASHKO_EXIT_CODE%"=="0" (
    echo.
    echo Raspberry connection console exited with code %RASHKO_EXIT_CODE%.
    pause
)
exit /b %RASHKO_EXIT_CODE%
