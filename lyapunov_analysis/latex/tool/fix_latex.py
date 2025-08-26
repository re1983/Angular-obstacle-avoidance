#!/usr/bin/env python3
"""
Fix LaTeX compilation errors in rigorous_proof.tex
- Fix math environment delimiters
- Replace Unicode characters
- Fix other common LaTeX issues
"""

import re
import sys

def fix_latex_file(filepath):
    """Fix common LaTeX compilation errors"""
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    print("🔧 Fixing LaTeX errors...")
    
    # Fix math environment delimiters - replace consecutive \[ \[ with \[ \]
    content = re.sub(r'\\\\\\[\s*\n\s*\\\\\\[', r'\\\\\\\[\n\\\\\\\]', content)
    
    # Fix missing \] after \[
    # Find \[ followed by content and another \[, replace second \[ with \]
    content = re.sub(r'(\\\\\\[[^\\]*?)\\\\\\[', r'\1\\\\\\\]', content)
    
    # Replace Unicode Greek letters with LaTeX commands
    unicode_replacements = {
        'θ': r'$\theta$',
        'β': r'$\beta$',
        'Δ': r'$\Delta$',
        'ψ': r'$\psi$',
        'α': r'$\alpha$',
        'π': r'$\pi$',
        'ω': r'$\omega$',
        'φ': r'$\phi$',
        'λ': r'$\lambda$',
        '°': r'$^\circ$'
    }
    
    for unicode_char, latex_cmd in unicode_replacements.items():
        content = content.replace(unicode_char, latex_cmd)
    
    # Fix common special characters
    content = content.replace('&', '\\&')
    content = content.replace('%', '\\%')
    # Don't escape # in math mode or if already escaped
    content = re.sub(r'(?<!\\)#(?![a-zA-Z])', r'\\#', content)
    
    # Fix incomplete math environments
    # Find \[ without matching \]
    lines = content.split('\n')
    fixed_lines = []
    in_display_math = False
    
    for i, line in enumerate(lines):
        # Count \[ and \] in this line
        open_count = line.count('\\[')
        close_count = line.count('\\]')
        
        if open_count > close_count:
            in_display_math = True
        elif close_count > open_count:
            in_display_math = False
        elif in_display_math and line.strip() and not line.strip().startswith('%'):
            # We're in display math but this line doesn't close it
            # Check if next few lines contain another \[ - if so, close this one
            next_lines = lines[i+1:i+3] if i+1 < len(lines) else []
            if any('\\[' in next_line for next_line in next_lines):
                line += '\n\\]'
                in_display_math = False
        
        fixed_lines.append(line)
    
    content = '\n'.join(fixed_lines)
    
    # Final cleanup for common patterns
    # Fix double display math delimiters
    content = re.sub(r'\\\\\\]\s*\\\\\\[', '', content)
    
    # Remove empty display math environments
    content = re.sub(r'\\\\\\[\s*\\\\\\]', '', content)
    
    # Fix spacing issues
    content = re.sub(r'\n{3,}', '\n\n', content)
    
    # Write the fixed content
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"✅ Fixed LaTeX file: {filepath}")
    print("📝 Main fixes applied:")
    print("   - Fixed math environment delimiters")
    print("   - Replaced Unicode characters with LaTeX commands")
    print("   - Fixed special character escaping")
    print("   - Cleaned up formatting")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = "rigorous_proof.tex"
    
    fix_latex_file(filepath)
