@echo off
rem FIC Studio in Docker: double-click to start. Arguments are passed on to
rem fic-studio.ps1, e.g.  fic-studio.cmd stop   or   fic-studio.cmd update
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0fic-studio.ps1" %*
if errorlevel 1 pause
