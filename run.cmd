@echo off
rem Shortcut: run any script with this project's venv Python.
rem   .\run tutorials\01_basics.py --headless
rem %~dp0 = the folder this file is in, so it works from any working directory.
"%~dp0.venv\Scripts\python.exe" %*
