@echo off
echo Starting Monte Carlo Analysis Pipeline...
echo.

REM ������}���ؿ�
cd /d "%~dp0"

REM ���� Python �޹D�}��
python run_complete_analysis.py

echo.
echo Pipeline completed. Press any key to exit...
pause > nul
