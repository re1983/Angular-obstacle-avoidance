@echo off
REM Compile LaTeX to PDF
REM Requires pdflatex (MiKTeX, TeX Live, or similar)

set TEXFILE=%1
if "%TEXFILE%"=="" set TEXFILE=rigorous_proof.tex

echo Compiling %TEXFILE% to PDF...

REM Check if pdflatex is available
pdflatex --version >nul 2>&1
if errorlevel 1 (
    echo.
    echo ❌ pdflatex not found!
    echo Please install MiKTeX or TeX Live:
    echo - MiKTeX: https://miktex.org/download
    echo - TeX Live: https://www.tug.org/texlive/
    echo.
    pause
    exit /b 1
)

REM Compile twice for cross-references
echo First pass...
pdflatex -interaction=nonstopmode %TEXFILE%

if errorlevel 1 (
    echo.
    echo ❌ LaTeX compilation failed!
    echo Check %TEXFILE:.tex=.log% for errors
    pause
    exit /b 1
)

echo Second pass...
pdflatex -interaction=nonstopmode %TEXFILE%

REM Clean up auxiliary files
del *.aux *.log *.out *.toc 2>nul

set PDFFILE=%TEXFILE:.tex=.pdf%

if exist "%PDFFILE%" (
    echo.
    echo ✅ Successfully compiled to %PDFFILE%
    echo 📖 Opening PDF...
    start "" "%PDFFILE%"
) else (
    echo ❌ PDF generation failed
)

pause
