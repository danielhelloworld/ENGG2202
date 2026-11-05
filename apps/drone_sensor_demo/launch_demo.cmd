@echo off
cd /d "%~dp0"
python demo.py
if errorlevel 1 pause
