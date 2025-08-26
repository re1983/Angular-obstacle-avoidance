# LaTeX Documents

This directory contains all LaTeX-related files for the Lyapunov stability analysis.

## 📄 Main Documents

- **`rigorous_proof.tex`** - Complete LaTeX source document
- **`rigorous_proof.pdf`** - Compiled PDF (275KB, 9 pages)
- **`rigorous_proof_backup.tex`** - Backup copy before fixes

## 🔧 Tools and Scripts

### Compilation Tools
- **`compile_latex.bat`** - One-click compilation script for Windows
- **`convert_to_latex.py`** - Markdown to LaTeX converter

### Preview and Fixing Tools
- **`latex_preview.py`** - Generate HTML preview from LaTeX
- **`simple_fix.py`** - Fix common LaTeX math environment errors
- **`fix_latex.py`** - Advanced LaTeX error fixer

### Documentation
- **`LaTeX_Guide.md`** - Complete guide for LaTeX compilation options

## 🚀 Quick Start

### Compile to PDF
```bash
# Windows
.\compile_latex.bat

# Or manually
pdflatex rigorous_proof.tex
```

### Generate HTML Preview
```bash
python latex_preview.py rigorous_proof.tex
```

### Online Compilation (Recommended)
1. Upload `rigorous_proof.tex` to [Overleaf](https://www.overleaf.com/)
2. Click "Recompile" to generate PDF

## 📊 Document Statistics
- **Pages**: 9
- **Equations**: 50+ mathematical expressions
- **Sections**: 5 main sections with proofs
- **File size**: ~275KB PDF

## 🎯 Features
- ✅ Perfect mathematical equation rendering
- ✅ Academic paper formatting
- ✅ Automatic cross-references and numbering
- ✅ Professional PDF output
- ✅ Complete bibliography

## 📝 Notes
- All math environments have been fixed and compile without errors
- The document uses standard LaTeX packages for maximum compatibility
- Content is identical to the Markdown version but with proper LaTeX formatting
