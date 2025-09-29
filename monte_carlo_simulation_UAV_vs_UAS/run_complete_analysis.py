"""
Complete Monte Carlo Analysis Pipeline
完整的蒙地卡羅分析管道
"""

import os
import sys
from pathlib import Path
import subprocess
from datetime import datetime

def run_monte_carlo_pipeline():
    """執行完整的蒙地卡羅分析流程"""
    
    print("="*70)
    print("Monte Carlo Collision Avoidance Analysis Pipeline")
    print("="*70)
    
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    
    # 步驟 1: 執行蒙地卡羅模擬
    print("\n🚀 Step 1: Running Monte Carlo simulations...")
    print("-" * 50)
    
    try:
        result = subprocess.run([sys.executable, "monte_carlo_runner.py"], 
                              capture_output=False, check=True)
        print("✅ Monte Carlo simulation completed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"❌ Monte Carlo simulation failed with error: {e}")
        return False
    except FileNotFoundError:
        print("❌ monte_carlo_runner.py not found!")
        return False
    
    # 步驟 2: 分析結果並生成圖表
    print("\n📊 Step 2: Analyzing results and generating plots...")
    print("-" * 50)
    
    try:
        result = subprocess.run([sys.executable, "analyze_results.py"], 
                              capture_output=False, check=True)
        print("✅ Results analysis completed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"❌ Results analysis failed with error: {e}")
        return False
    except FileNotFoundError:
        print("❌ analyze_results.py not found!")
        return False
    
    # 步驟 3: 列出生成的文件
    print("\n📁 Step 3: Generated files summary...")
    print("-" * 50)
    
    results_dir = Path("results")
    if results_dir.exists():
        # 找到最新的結果目錄
        result_dirs = list(results_dir.glob("results_*"))
        if result_dirs:
            latest_dir = max(result_dirs, key=lambda x: x.name)
            print(f"📂 Results directory: {latest_dir}")
            
            # 列出主要文件
            main_files = [
                "report_*.txt",
                "overview_*.png", 
                "trajectories_*.png",
                "simulation_results_*.npz"
            ]
            
            print("\n📄 Main output files:")
            for pattern in main_files:
                files = list(latest_dir.glob(pattern))
                for file in files:
                    print(f"   • {file.name}")
            
            # 統計個別結果
            subdirs = ["successful", "collision", "timeout"]
            print("\n📊 Individual results:")
            for subdir in subdirs:
                subdir_path = latest_dir / subdir
                if subdir_path.exists():
                    txt_files = list(subdir_path.glob("*.txt"))
                    png_files = list(subdir_path.glob("*.png"))
                    print(f"   • {subdir}: {len(txt_files)} parameter files, {len(png_files)} trajectory plots")
    
    print("\n🎉 Monte Carlo analysis pipeline completed!")
    print("="*70)
    return True

def create_batch_file():
    """創建批次執行文件 (Windows .bat)"""
    batch_content = '''@echo off
echo Starting Monte Carlo Analysis Pipeline...
echo.

REM 切換到腳本目錄
cd /d "%~dp0"

REM 執行 Python 管道腳本
python run_complete_analysis.py

echo.
echo Pipeline completed. Press any key to exit...
pause > nul
'''
    
    batch_file = Path("run_analysis.bat")
    with open(batch_file, 'w') as f:
        f.write(batch_content)
    
    print(f"✅ Batch file created: {batch_file}")
    print("   You can double-click 'run_analysis.bat' to run the complete analysis!")

def main():
    """主函數"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Monte Carlo Analysis Pipeline")
    parser.add_argument("--create-batch", action="store_true", 
                       help="Create batch file for easy execution")
    
    args = parser.parse_args()
    
    if args.create_batch:
        create_batch_file()
        return
    
    # 執行完整分析流程
    success = run_monte_carlo_pipeline()
    
    if success:
        print("\n💡 Tip: You can create a batch file for easy future runs:")
        print(f"   python {Path(__file__).name} --create-batch")

if __name__ == "__main__":
    main()
