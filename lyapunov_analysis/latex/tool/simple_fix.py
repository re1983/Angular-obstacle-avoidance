#!/usr/bin/env python3
"""
Simple LaTeX fixer for display math environments
"""

def fix_latex_simple(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    fixed_lines = []
    i = 0
    
    while i < len(lines):
        line = lines[i].rstrip()
        
        # Check if this line starts a display math environment
        if line.strip() == '\\[':
            fixed_lines.append(line + '\n')
            i += 1
            
            # Look for the content and closing
            while i < len(lines):
                current_line = lines[i].rstrip()
                
                # If we find another \[, replace it with \]
                if current_line.strip() == '\\[':
                    fixed_lines.append('\\]\n')
                    break
                # If we find a proper \], keep it
                elif current_line.strip() == '\\]':
                    fixed_lines.append(current_line + '\n')
                    break
                else:
                    fixed_lines.append(current_line + '\n')
                
                i += 1
        else:
            fixed_lines.append(line + '\n')
        
        i += 1
    
    # Write the fixed content
    with open(filepath, 'w', encoding='utf-8') as f:
        f.writelines(fixed_lines)
    
    print(f"✅ Fixed math environments in {filepath}")

if __name__ == "__main__":
    fix_latex_simple("rigorous_proof.tex")
