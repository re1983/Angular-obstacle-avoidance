#!/usr/bin/env python3
"""
LaTeX to HTML converter for quick preview
Converts basic LaTeX to HTML with MathJax for mathematical content
"""

import re
import sys
import os
from pathlib import Path

def latex_to_html_preview(tex_file_path, output_file_path=None):
    """Convert LaTeX file to HTML for quick preview"""
    
    with open(tex_file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract content between \begin{document} and \end{document}
    doc_match = re.search(r'\\begin\{document\}(.*?)\\end\{document\}', content, re.DOTALL)
    if doc_match:
        latex_content = doc_match.group(1)
    else:
        latex_content = content
    
    # Convert LaTeX commands to HTML equivalents
    html_content = latex_content
    
    # Sections
    html_content = re.sub(r'\\maketitle\s*', '', html_content)
    html_content = re.sub(r'\\tableofcontents\s*', '<div id="toc">Table of Contents will be generated automatically</div>', html_content)
    html_content = re.sub(r'\\newpage\s*', '<div style="page-break-before: always;"></div>', html_content)
    
    # Headers
    html_content = re.sub(r'\\section\{([^}]+)\}', r'<h2>\1</h2>', html_content)
    html_content = re.sub(r'\\subsection\{([^}]+)\}', r'<h3>\1</h3>', html_content)
    html_content = re.sub(r'\\subsubsection\{([^}]+)\}', r'<h4>\1</h4>', html_content)
    html_content = re.sub(r'\\paragraph\{([^}]+)\}', r'<h5>\1</h5>', html_content)
    
    # Text formatting
    html_content = re.sub(r'\\textbf\{([^}]+)\}', r'<strong>\1</strong>', html_content)
    html_content = re.sub(r'\\textit\{([^}]+)\}', r'<em>\1</em>', html_content)
    html_content = re.sub(r'\\texttt\{([^}]+)\}', r'<code>\1</code>', html_content)
    
    # Lists
    html_content = re.sub(r'\\begin\{itemize\}', '<ul>', html_content)
    html_content = re.sub(r'\\end\{itemize\}', '</ul>', html_content)
    html_content = re.sub(r'\\begin\{enumerate\}', '<ol>', html_content)
    html_content = re.sub(r'\\end\{enumerate\}', '</ol>', html_content)
    html_content = re.sub(r'\\item\s+', '<li>', html_content)
    
    # Math environments (keep LaTeX syntax for MathJax)
    # Display math
    html_content = re.sub(r'\\begin\{equation\}(.*?)\\end\{equation\}', r'$$\1$$', html_content, flags=re.DOTALL)
    html_content = re.sub(r'\\begin\{align\}(.*?)\\end\{align\}', r'$$\\begin{align}\1\\end{align}$$', html_content, flags=re.DOTALL)
    html_content = re.sub(r'\\begin\{equation\*\}(.*?)\\end\{equation\*\}', r'$$\1$$', html_content, flags=re.DOTALL)
    html_content = re.sub(r'\\begin\{align\*\}(.*?)\\end\{align\*\}', r'$$\\begin{align*}\1\\end{align*}$$', html_content, flags=re.DOTALL)
    
    # Convert \[ \] to $$ $$ (fix the regex)
    html_content = re.sub(r'\\\\\\\[(.*?)\\\\\\\]', r'$$\1$$', html_content, flags=re.DOTALL)
    
    # Clean up extra whitespace and newlines
    html_content = re.sub(r'\n\s*\n\s*\n', '\n\n', html_content)
    html_content = html_content.strip()
    
    # Convert newlines to HTML
    html_content = html_content.replace('\n\n', '</p><p>')
    html_content = f'<p>{html_content}</p>'
    
    # Fix empty paragraphs and nested lists
    html_content = re.sub(r'<p>\s*</p>', '', html_content)
    html_content = re.sub(r'<p>(\s*<[hou][1-6lil])', r'\1', html_content)
    html_content = re.sub(r'(<\/[hou][1-6lil]>\s*)</p>', r'\1', html_content)
    html_content = re.sub(r'<p>(\s*</?[uo]l)', r'\1', html_content)
    html_content = re.sub(r'(</?[uo]l>\s*)</p>', r'\1', html_content)
    
    # Complete HTML document
    html_template = f'''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>LaTeX Preview</title>
    
    <!-- MathJax Configuration -->
    <script>
        MathJax = {{
            tex: {{
                inlineMath: [['$', '$'], ['\\\\(', '\\\\)']],
                displayMath: [['$$', '$$'], ['\\\\[', '\\\\]']],
                processEscapes: true,
                processEnvironments: true,
                tags: 'ams',
                macros: {{
                    norm: ["\\\\left\\\\|#1\\\\right\\\\|", 1],
                    abs: ["\\\\left|#1\\\\right|", 1],
                    Real: "\\\\mathbb{{R}}",
                    eps: "\\\\varepsilon"
                }}
            }},
            options: {{
                ignoreHtmlClass: 'tex2jax_ignore',
                processHtmlClass: 'tex2jax_process'
            }}
        }};
    </script>
    <script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
    <script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>
    
    <!-- Styling -->
    <style>
        body {{
            font-family: 'Times New Roman', Times, serif;
            line-height: 1.6;
            max-width: 900px;
            margin: 0 auto;
            padding: 40px 20px;
            background-color: #ffffff;
            color: #000000;
        }}
        
        h2, h3, h4, h5, h6 {{
            margin-top: 24px;
            margin-bottom: 12px;
            font-weight: bold;
        }}
        
        h2 {{ font-size: 1.5em; }}
        h3 {{ font-size: 1.3em; }}
        h4 {{ font-size: 1.1em; }}
        h5 {{ font-size: 1em; font-weight: bold; }}
        
        p {{
            margin-bottom: 12px;
            text-align: justify;
        }}
        
        ul, ol {{
            margin: 12px 0;
            padding-left: 30px;
        }}
        
        li {{
            margin-bottom: 6px;
        }}
        
        code {{
            background-color: #f5f5f5;
            padding: 2px 4px;
            border-radius: 3px;
            font-family: 'Courier New', Courier, monospace;
        }}
        
        strong {{
            font-weight: bold;
        }}
        
        em {{
            font-style: italic;
        }}
        
        /* Math styling */
        .MathJax {{
            font-size: 1em !important;
        }}
        
        mjx-container[jax="CHTML"][display="true"] {{
            margin: 1.2em 0;
        }}
        
        #toc {{
            background-color: #f9f9f9;
            padding: 15px;
            border-left: 4px solid #007acc;
            margin: 20px 0;
        }}
        
        /* Print styles */
        @media print {{
            body {{
                font-size: 12pt;
                line-height: 1.4;
            }}
            
            h2, h3, h4, h5, h6 {{
                page-break-after: avoid;
            }}
        }}
    </style>
</head>
<body>
    {html_content}
    
    <footer style="margin-top: 50px; padding-top: 20px; border-top: 1px solid #ccc; color: #666; text-align: center; font-size: 0.9em;">
        <p>LaTeX Preview • Generated from {os.path.basename(tex_file_path)} • Mathematical equations rendered with MathJax</p>
    </footer>
</body>
</html>'''
    
    # Determine output file name
    if output_file_path is None:
        tex_path = Path(tex_file_path)
        output_file_path = tex_path.with_name(f"{tex_path.stem}_preview.html")
    
    # Write HTML file
    with open(output_file_path, 'w', encoding='utf-8') as f:
        f.write(html_template)
    
    print(f"✅ LaTeX preview generated: {output_file_path}")
    print(f"🌐 Open in browser: file://{os.path.abspath(output_file_path)}")
    
    return str(output_file_path)

def main():
    if len(sys.argv) < 2:
        print("Usage: python latex_preview.py <tex_file> [output_file]")
        print("Example: python latex_preview.py rigorous_proof.tex")
        sys.exit(1)
    
    tex_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(tex_file):
        print(f"❌ Error: File '{tex_file}' not found")
        sys.exit(1)
    
    try:
        latex_to_html_preview(tex_file, output_file)
    except Exception as e:
        print(f"❌ Error converting file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
