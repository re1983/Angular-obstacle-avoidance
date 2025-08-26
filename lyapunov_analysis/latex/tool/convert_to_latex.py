#!/usr/bin/env python3
"""
Convert Markdown to LaTeX document with proper mathematical formatting
Perfect for academic papers and mathematical documents
"""

import re
import os
import sys
from pathlib import Path

def convert_markdown_to_latex(md_file_path, output_file_path=None):
    """Convert markdown file to LaTeX with proper math formatting"""
    
    # Read the markdown file
    with open(md_file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # LaTeX document template
    latex_template = r"""\documentclass[11pt,a4paper]{article}
\usepackage[utf8]{inputenc}
\usepackage[T1]{fontenc}
\usepackage{lmodern}
\usepackage{amsmath,amssymb,amsfonts}
\usepackage{mathtools}
\usepackage{geometry}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{array}
\usepackage{hyperref}
\usepackage{cleveref}
\usepackage{enumitem}
\usepackage{float}

% Page setup
\geometry{margin=1in}
\setlength{\parindent}{0pt}
\setlength{\parskip}{6pt plus 2pt minus 1pt}

% Math environments
\numberwithin{equation}{section}

% Custom commands
\newcommand{\norm}[1]{\left\|#1\right\|}
\newcommand{\abs}[1]{\left|#1\right|}
\newcommand{\set}[1]{\left\{#1\right\}}
\newcommand{\Real}{\mathbb{R}}
\newcommand{\eps}{\varepsilon}
\newcommand{\To}{\rightarrow}
\newcommand{\BX}{\mathbf{B}(X)}
\newcommand{\A}{\mathcal{A}}

% Title and author
\title{Rigorous Lyapunov Stability Analysis for Angle-Only Collision Avoidance System}
\author{Mathematical Analysis}
\date{\today}

\begin{document}

\maketitle

\tableofcontents
\newpage

{CONTENT}

\end{document}"""
    
    # Convert Markdown elements to LaTeX
    latex_content = content
    
    # Headers
    latex_content = re.sub(r'^# (.+)$', r'\\section{\1}', latex_content, flags=re.MULTILINE)
    latex_content = re.sub(r'^## (.+)$', r'\\subsection{\1}', latex_content, flags=re.MULTILINE)
    latex_content = re.sub(r'^### (.+)$', r'\\subsubsection{\1}', latex_content, flags=re.MULTILINE)
    latex_content = re.sub(r'^#### (.+)$', r'\\paragraph{\1}', latex_content, flags=re.MULTILINE)
    
    # Bold text
    latex_content = re.sub(r'\*\*(.+?)\*\*', r'\\textbf{\1}', latex_content)
    
    # Italic text
    latex_content = re.sub(r'\*(.+?)\*', r'\\textit{\1}', latex_content)
    
    # Code inline
    latex_content = re.sub(r'`([^`]+)`', r'\\texttt{\1}', latex_content)
    
    # Lists
    latex_content = re.sub(r'^- (.+)$', r'\\item \1', latex_content, flags=re.MULTILINE)
    
    # Convert bullet lists to itemize environment
    lines = latex_content.split('\n')
    new_lines = []
    in_list = False
    
    for line in lines:
        if line.strip().startswith('\\item '):
            if not in_list:
                new_lines.append('\\begin{itemize}')
                in_list = True
            new_lines.append(line)
        else:
            if in_list:
                new_lines.append('\\end{itemize}')
                in_list = False
            new_lines.append(line)
    
    if in_list:
        new_lines.append('\\end{itemize}')
    
    latex_content = '\n'.join(new_lines)
    
    # Display math equations (keep as is - they're already in LaTeX format)
    # Just ensure proper spacing
    latex_content = re.sub(r'\n\$\$\n', r'\n\\[\n', latex_content)
    latex_content = re.sub(r'\n\$\$', r'\n\\]\n', latex_content)
    
    # Inline math (keep as is)
    
    # Special characters
    latex_content = latex_content.replace('&', '\\&')
    latex_content = latex_content.replace('%', '\\%')
    latex_content = latex_content.replace('#', '\\#')
    
    # Fix some common issues
    latex_content = latex_content.replace('\\text{atan2}', '\\mathrm{atan2}')
    latex_content = latex_content.replace('\\text{max}', '\\mathrm{max}')
    latex_content = latex_content.replace('\\text{min}', '\\mathrm{min}')
    latex_content = latex_content.replace('\\text{second}', '\\mathrm{second}')
    
    # Create complete LaTeX document
    complete_latex = latex_template.replace('{CONTENT}', latex_content)
    
    # Determine output file name
    if output_file_path is None:
        md_path = Path(md_file_path)
        output_file_path = md_path.with_suffix('.tex')
    
    # Write LaTeX file
    with open(output_file_path, 'w', encoding='utf-8') as f:
        f.write(complete_latex)
    
    print(f"✅ Converted to LaTeX: {md_file_path}")
    print(f"📄 Output: {output_file_path}")
    print(f"🔨 Compile with: pdflatex {output_file_path}")
    
    return str(output_file_path)

def main():
    """Main function to handle command line usage"""
    if len(sys.argv) < 2:
        print("Usage: python convert_to_latex.py <markdown_file> [output_file]")
        print("Example: python convert_to_latex.py rigorous_proof_github.md proof.tex")
        sys.exit(1)
    
    md_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(md_file):
        print(f"❌ Error: File '{md_file}' not found")
        sys.exit(1)
    
    try:
        convert_markdown_to_latex(md_file, output_file)
    except Exception as e:
        print(f"❌ Error converting file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
