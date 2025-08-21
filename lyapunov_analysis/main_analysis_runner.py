"""
Main Analysis Runner for Lyapunov Stability Verification
=======================================================

This script coordinates all phases of the Lyapunov stability analysis:
1. Mathematical stability analysis with visualization
2. Numerical verification through Monte Carlo testing
3. Parameter sensitivity analysis
4. Comprehensive reporting and documentation

Usage:
    python main_analysis_runner.py

Author: Auto-generated analysis for Angular-obstacle-avoidance project
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stability_analyzer import LyapunovAnalyzer, run_comprehensive_analysis
from numerical_verification import NumericalVerifier, run_full_numerical_verification

def main():
    """
    Run complete Lyapunov stability analysis suite
    """
    print("="*80)
    print("LYAPUNOV STABILITY ANALYSIS FOR ANGLE-ONLY COLLISION AVOIDANCE")
    print("="*80)
    print(f"Analysis started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # Create results directory
    results_dir = "analysis_results"
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
        print(f"Created results directory: {results_dir}")
    
    print("This analysis will perform:")
    print("• Mathematical Lyapunov stability analysis")
    print("• Numerical verification with Monte Carlo testing")
    print("• Parameter sensitivity analysis")
    print("• Comprehensive stability reporting")
    print()
    
    try:
        # Phase 1: Core Lyapunov Analysis
        print("PHASE 1: CORE LYAPUNOV STABILITY ANALYSIS")
        print("="*50)
        print("Analyzing both absolute and relative bearing control systems...")
        
        analyzer = LyapunovAnalyzer(R_safe=2.0, alpha_nav=1.0)
        
        # Test both control types
        for use_absolute in [True, False]:
            control_type = "Absolute Bearing" if use_absolute else "Relative Bearing"
            print(f"\nAnalyzing {control_type} Control...")
            
            # Run simulation and analysis
            sim_data = analyzer.run_simulation_with_analysis(
                use_absolute=use_absolute, steps=5000
            )
            analysis_results = analyzer.analyze_stability_conditions(sim_data)
            
            # Generate plots
            fig = analyzer.plot_lyapunov_analysis(analysis_results, sim_data)
            fig.suptitle(f'Lyapunov Stability Analysis - {control_type} Control', fontsize=16)
            
            # Save results
            filename = f"{results_dir}/lyapunov_analysis_{control_type.lower().replace(' ', '_')}.png"
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Analysis plot saved: {filename}")
            
            # Print stability report
            print(f"\n{control_type.upper()} CONTROL - STABILITY REPORT:")
            print("-" * 40)
            analyzer.print_stability_report(analysis_results)
            
            plt.close(fig)  # Close to save memory
        
        print("\nPhase 1 completed successfully!")
        
        # Phase 2: Numerical Verification
        print("\n\nPHASE 2: NUMERICAL VERIFICATION")
        print("="*50)
        print("Running comprehensive Monte Carlo verification...")
        
        verifier = NumericalVerifier()
        
        # Monte Carlo testing
        print("Running Monte Carlo analysis (this may take a few minutes)...")
        mc_results = verifier.monte_carlo_test(n_trials=50)  # Reduced for demonstration
        
        # Plot and save Monte Carlo results
        fig_mc = verifier.plot_monte_carlo_results(mc_results)
        mc_filename = f"{results_dir}/monte_carlo_verification.png"
        plt.savefig(mc_filename, dpi=300, bbox_inches='tight')
        print(f"Monte Carlo results saved: {mc_filename}")
        plt.close(fig_mc)
        
        print("Phase 2 completed successfully!")
        
        # Phase 3: Sensitivity Analysis
        print("\n\nPHASE 3: PARAMETER SENSITIVITY ANALYSIS")
        print("="*50)
        
        sensitivity_results = []
        
        # R_safe sensitivity
        print("Analyzing R_safe parameter sensitivity...")
        sens_R_safe = verifier.sensitivity_analysis('R_safe', (1.0, 4.0), n_points=10)
        sensitivity_results.append(sens_R_safe)
        
        fig_R_safe, optimal_R_safe = verifier.plot_sensitivity_results(sens_R_safe)
        r_safe_filename = f"{results_dir}/sensitivity_R_safe.png"
        plt.savefig(r_safe_filename, dpi=300, bbox_inches='tight')
        print(f"R_safe sensitivity results saved: {r_safe_filename}")
        plt.close(fig_R_safe)
        
        # alpha_nav sensitivity
        print("Analyzing alpha_nav parameter sensitivity...")
        sens_alpha = verifier.sensitivity_analysis('alpha_nav', (0.3, 2.5), n_points=10)
        sensitivity_results.append(sens_alpha)
        
        fig_alpha, optimal_alpha = verifier.plot_sensitivity_results(sens_alpha)
        alpha_filename = f"{results_dir}/sensitivity_alpha_nav.png"
        plt.savefig(alpha_filename, dpi=300, bbox_inches='tight')
        print(f"alpha_nav sensitivity results saved: {alpha_filename}")
        plt.close(fig_alpha)
        
        print("Phase 3 completed successfully!")
        
        # Phase 4: Final Report Generation
        print("\n\nPHASE 4: COMPREHENSIVE REPORT GENERATION")
        print("="*50)
        
        final_report = verifier.generate_comprehensive_report(mc_results, sensitivity_results)
        
        # Save numerical results
        import json
        
        results_summary = {
            'analysis_timestamp': datetime.now().isoformat(),
            'monte_carlo': {
                'n_trials': mc_results['n_trials'],
                'safety_rate': mc_results['safety_rate'],
                'boundedness_rate': mc_results['boundedness_rate'],
                'ultimate_boundedness_rate': mc_results['ultimate_boundedness_rate'],
                'min_distance_stats': mc_results['min_distance_stats'],
                'max_angular_size_stats': mc_results['max_angular_size_stats'],
                'cbdr_ratio_stats': mc_results['cbdr_ratio_stats']
            },
            'sensitivity_analysis': [
                {
                    'parameter_name': sens['parameter_name'],
                    'parameter_values': sens['parameter_values'].tolist(),
                    'safety_rates': sens['safety_rates'].tolist(),
                    'boundedness_rates': sens['boundedness_rates'].tolist(),
                    'ultimate_boundedness_rates': sens['ultimate_boundedness_rates'].tolist(),
                    'min_distances': sens['min_distances'].tolist()
                }
                for sens in sensitivity_results
            ],
            'optimal_parameters': {
                'R_safe': float(optimal_R_safe),
                'alpha_nav': float(optimal_alpha)
            },
            'final_assessment': {
                'overall_success_rate': final_report['overall_success_rate'],
                'assessment': final_report['assessment']
            }
        }
        
        results_file = f"{results_dir}/complete_analysis_results.json"
        with open(results_file, 'w') as f:
            json.dump(results_summary, f, indent=2)
        print(f"Complete analysis results saved: {results_file}")
        
        # Generate summary report file
        summary_file = f"{results_dir}/analysis_summary.txt"
        with open(summary_file, 'w') as f:
            f.write("LYAPUNOV STABILITY ANALYSIS - EXECUTIVE SUMMARY\n")
            f.write("="*60 + "\n\n")
            f.write(f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Analysis Duration: Comprehensive multi-phase verification\n\n")
            
            f.write("KEY FINDINGS:\n")
            f.write("-" * 20 + "\n")
            f.write(f"• Safety Success Rate: {mc_results['safety_rate']:.1%}\n")
            f.write(f"• System Boundedness: {mc_results['boundedness_rate']:.1%}\n")
            f.write(f"• Ultimate Boundedness: {mc_results['ultimate_boundedness_rate']:.1%}\n")
            f.write(f"• Overall System Performance: {final_report['overall_success_rate']:.1%}\n\n")
            
            f.write("OPTIMAL PARAMETERS:\n")
            f.write("-" * 20 + "\n")
            f.write(f"• Recommended R_safe: {optimal_R_safe:.2f} meters\n")
            f.write(f"• Recommended alpha_nav: {optimal_alpha:.2f} degrees\n\n")
            
            f.write("SYSTEM ASSESSMENT:\n")
            f.write("-" * 20 + "\n")
            f.write(f"• {final_report['assessment']}\n\n")
            
            f.write("GENERATED FILES:\n")
            f.write("-" * 20 + "\n")
            f.write("• lyapunov_analysis_absolute_bearing.png - Core stability analysis\n")
            f.write("• lyapunov_analysis_relative_bearing.png - Alternative control analysis\n")
            f.write("• monte_carlo_verification.png - Statistical verification results\n")
            f.write("• sensitivity_R_safe.png - Safety parameter sensitivity\n")
            f.write("• sensitivity_alpha_nav.png - Navigation parameter sensitivity\n")
            f.write("• complete_analysis_results.json - Raw numerical results\n")
            f.write("• analysis_summary.txt - This summary file\n")
        
        print(f"Analysis summary saved: {summary_file}")
        
        print("Phase 4 completed successfully!")
        
        # Final Summary
        print("\n\n" + "="*80)
        print("ANALYSIS COMPLETED SUCCESSFULLY")
        print("="*80)
        print(f"Results saved in: {os.path.abspath(results_dir)}/")
        print()
        print("KEY OUTCOMES:")
        print(f"• Safety Success Rate: {mc_results['safety_rate']:.1%}")
        print(f"• Overall System Performance: {final_report['overall_success_rate']:.1%}")
        print(f"• System Assessment: {final_report['assessment']}")
        print()
        print("RECOMMENDED PARAMETERS:")
        print(f"• R_safe = {optimal_R_safe:.2f} meters")
        print(f"• alpha_nav = {optimal_alpha:.2f} degrees")
        print()
        print("Review the generated plots and results files for detailed analysis.")
        print("="*80)
        
        return True
        
    except Exception as e:
        print(f"\nERROR: Analysis failed with exception: {str(e)}")
        print("Check input files and dependencies.")
        import traceback
        traceback.print_exc()
        return False

def quick_analysis():
    """
    Run a quick version of the analysis for testing
    """
    print("Running Quick Analysis Version...")
    print("This performs basic stability verification with reduced computational load.")
    print()
    
    # Basic stability analysis
    analyzer = LyapunovAnalyzer(R_safe=2.0, alpha_nav=1.0)
    
    # Run single simulation
    sim_data = analyzer.run_simulation_with_analysis(use_absolute=True, steps=3000)
    analysis_results = analyzer.analyze_stability_conditions(sim_data)
    
    # Print results
    print("QUICK ANALYSIS RESULTS:")
    print("-" * 30)
    analyzer.print_stability_report(analysis_results)
    
    # Generate basic plot
    fig = analyzer.plot_lyapunov_analysis(analysis_results, sim_data)
    plt.savefig('quick_analysis_result.png', dpi=200, bbox_inches='tight')
    print("Quick analysis plot saved: quick_analysis_result.png")
    plt.show()
    
    return analysis_results

if __name__ == "__main__":
    # Check command line arguments
    if len(sys.argv) > 1 and sys.argv[1] == "--quick":
        quick_analysis()
    else:
        main()
