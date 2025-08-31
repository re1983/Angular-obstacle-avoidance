@echo off
echo Starting Monte Carlo Analysis Pipeline...
echo.

REM 切換到腳本目錄
cd /d "%~dp0"

REM 執行 Python 管道腳本
python run_complete_analysis.py

echo.
echo Pipeline completed. Press any key to exit...
pause > nul
