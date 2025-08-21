"""
Numerical Verification and Monte Carlo Analysis
==============================================

This module performs extensive numerical verification of the Lyapunov stability
analysis through Monte Carlo simulations and sensitivity analysis.

Features:
- Monte Carlo testing with randomized initial conditions
- Parameter sensitivity analysis
- Comprehensive statistical reporting
- Visualization of verification results

Author: Auto-generated analysis for Angular-obstacle-avoidance project
"""

import numpy as np
import matplotlib.pyplot as plt
from stability_analyzer import LyapunovAnalyzer
import matplotlib.font_manager as fm
from scipy import stats
import json

class NumericalVerifier:
    """
    Numerical verification system for Lyapunov stability analysis
    """
    
    def __init__(self, R_safe_range=(1.5, 3.0), alpha_nav_range=(0.5, 2.0)):
        """
        Initialize the verifier with parameter ranges
        
        Args:
            R_safe_range: Range for safety distance testing (min, max)
            alpha_nav_range: Range for navigation threshold testing (min, max)
        """
        self.R_safe_range = R_safe_range
        self.alpha_nav_range = alpha_nav_range
        self.results_cache = {}
    
    def monte_carlo_test(self, n_trials=100, steps=3000):
        """
        Perform Monte Carlo testing with randomized initial conditions
        
        Args:
            n_trials: Number of random trials to run
            steps: Number of simulation steps per trial
        
        Returns:
            dict: Monte Carlo results with statistics
        """
        print(f"Running Monte Carlo analysis with {n_trials} trials...")
        
        # Initialize result storage
        safety_results = []
        boundedness_results = []
        ultimate_boundedness_results = []
        min_distances = []
        max_angular_sizes = []
        cbdr_ratios = []
        
        successful_trials = 0
        
        for trial in range(n_trials):
            if trial % 10 == 0:
                print(f"  Trial {trial+1}/{n_trials}")
            
            try:
                # Randomize parameters within reasonable ranges
                R_safe = np.random.uniform(*self.R_safe_range)
                alpha_nav = np.random.uniform(*self.alpha_nav_range)
                
                # Randomize initial conditions
                own_velocity = np.random.uniform(0.5, 2.0)
                target_velocity = np.random.uniform(0.5, 3.0)
                target_heading = np.random.uniform(0, 360)
                target_rate_of_turn = np.random.uniform(-2, 2)
                
                # Random initial positions (avoiding immediate collision)
                distance_range = (R_safe + 2, 25)
                angle_range = (-180, 180)
                
                initial_distance = np.random.uniform(*distance_range)
                initial_angle = np.radians(np.random.uniform(*angle_range))
                
                target_x = initial_distance * np.cos(initial_angle)
                target_y = initial_distance * np.sin(initial_angle)
                
                # Create analyzer and run simulation
                analyzer = LyapunovAnalyzer(R_safe=R_safe, alpha_nav=alpha_nav)
                
                # Custom simulation with randomized parameters
                sim_data = self._run_custom_simulation(
                    analyzer, own_velocity, target_velocity, target_heading, 
                    target_rate_of_turn, [target_x, target_y, 0], steps
                )
                
                if sim_data is None:
                    continue
                
                # Analyze stability
                analysis = analyzer.analyze_stability_conditions(sim_data)
                
                # Store results
                safety_results.append(analysis['safety_maintained'])
                boundedness_results.append(analysis['bounded'])
                ultimate_boundedness_results.append(analysis['ultimate_bounded'])
                min_distances.append(analysis['min_distance'])
                max_angular_sizes.append(analysis['max_angular_size'])
                cbdr_ratios.append(analysis['cbdr_activation_ratio'])
                
                successful_trials += 1
                
            except Exception as e:
                print(f"    Trial {trial+1} failed: {str(e)}")
                continue
        
        # Compute statistics
        safety_rate = np.mean(safety_results)
        boundedness_rate = np.mean(boundedness_results)
        ultimate_boundedness_rate = np.mean(ultimate_boundedness_results)
        
        results = {
            'n_trials': successful_trials,
            'safety_rate': safety_rate,
            'boundedness_rate': boundedness_rate,
            'ultimate_boundedness_rate': ultimate_boundedness_rate,
            'min_distance_stats': {
                'mean': np.mean(min_distances),
                'std': np.std(min_distances),
                'min': np.min(min_distances),
                'max': np.max(min_distances),
                'percentile_5': np.percentile(min_distances, 5),
                'percentile_95': np.percentile(min_distances, 95)
            },
            'max_angular_size_stats': {
                'mean': np.mean(max_angular_sizes),
                'std': np.std(max_angular_sizes),
                'min': np.min(max_angular_sizes),
                'max': np.max(max_angular_sizes)
            },
            'cbdr_ratio_stats': {
                'mean': np.mean(cbdr_ratios),
                'std': np.std(cbdr_ratios),
                'min': np.min(cbdr_ratios),
                'max': np.max(cbdr_ratios)
            },
            'raw_data': {
                'safety_results': safety_results,
                'boundedness_results': boundedness_results,
                'ultimate_boundedness_results': ultimate_boundedness_results,
                'min_distances': min_distances,
                'max_angular_sizes': max_angular_sizes,
                'cbdr_ratios': cbdr_ratios
            }
        }
        
        return results
    
    def _run_custom_simulation(self, analyzer, own_velocity, target_velocity, 
                             target_heading, target_rate_of_turn, target_position, steps):
        """
        Run custom simulation with specified parameters
        """
        try:
            from BearingRateGraph_comparison import (
                ShipStatus, get_distance_3d, get_bearing, get_absolute_bearing,
                get_angular_diameter, angle_difference_in_deg,
                adj_ownship_heading_absolute, adj_ownship_heading_relative
            )
            
            dt = 0.01
            
            # Initialize ships
            ownship = ShipStatus("Ownship", velocity=own_velocity, acceleration=0, 
                               heading=90, rate_of_turn=0, position=[0, 0, 0], size=0.5)
            target_ship = ShipStatus("Ship A", velocity=target_velocity, acceleration=0, 
                                   heading=target_heading, rate_of_turn=target_rate_of_turn, 
                                   position=target_position, size=0.5)
            goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0, 
                            rate_of_turn=0, position=[0, 50, 0])
            
            # Data storage
            distances = []
            bearings = []
            absolute_bearings = []
            angular_sizes = []
            bearings_difference = []
            absolute_bearings_difference = []
            
            for i in range(steps):
                # Current measurements
                bearing = get_bearing(ownship, target_ship)
                absolute_bearing = get_absolute_bearing(ownship, target_ship)
                angular_size = get_angular_diameter(ownship, target_ship)
                distance = get_distance_3d(ownship.position, target_ship.position)
                
                # Store data
                bearings.append(bearing)
                absolute_bearings.append(absolute_bearing)
                angular_sizes.append(angular_size)
                distances.append(distance)
                
                # Check for collision (simulation failed)
                if distance < 0.1:
                    return None
                
                # Apply control (use absolute bearing control)
                ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_absolute(
                    absolute_bearings, absolute_bearings_difference, angular_sizes, 
                    ownship, goal, target_ship, dt)
                
                # Update positions
                ownship.update(dt)
                target_ship.update(dt)
                
                # Calculate bearing rate
                if i > 0:
                    new_absolute_bearing = get_absolute_bearing(ownship, target_ship)
                    abs_bearing_diff = angle_difference_in_deg(absolute_bearing, new_absolute_bearing) / dt
                    absolute_bearings_difference.append(abs_bearing_diff)
            
            return {
                'distances': np.array(distances),
                'bearings': np.array(bearings),
                'absolute_bearings': np.array(absolute_bearings),
                'angular_sizes': np.array(angular_sizes),
                'absolute_bearings_difference': np.array(absolute_bearings_difference)
            }
            
        except Exception as e:
            print(f"Simulation failed: {str(e)}")
            return None
    
    def sensitivity_analysis(self, parameter_name, param_range, n_points=20):
        """
        Perform sensitivity analysis for a specific parameter
        
        Args:
            parameter_name: Name of parameter to vary ('R_safe' or 'alpha_nav')
            param_range: (min, max) range for parameter
            n_points: Number of points to test
        
        Returns:
            dict: Sensitivity analysis results
        """
        print(f"Running sensitivity analysis for {parameter_name}...")
        
        param_values = np.linspace(param_range[0], param_range[1], n_points)
        
        safety_rates = []
        boundedness_rates = []
        ultimate_boundedness_rates = []
        min_distances = []
        
        for i, param_value in enumerate(param_values):
            print(f"  Testing {parameter_name} = {param_value:.3f} ({i+1}/{n_points})")
            
            # Set parameters
            if parameter_name == 'R_safe':
                R_safe = param_value
                alpha_nav = 1.0  # Fixed
            elif parameter_name == 'alpha_nav':
                R_safe = 2.0  # Fixed
                alpha_nav = param_value
            else:
                raise ValueError(f"Unknown parameter: {parameter_name}")
            
            # Run multiple trials for this parameter value
            n_trials = 20  # Reduced for sensitivity analysis
            trial_results = {
                'safety': [],
                'boundedness': [],
                'ultimate_boundedness': [],
                'min_distance': []
            }
            
            for trial in range(n_trials):
                try:
                    analyzer = LyapunovAnalyzer(R_safe=R_safe, alpha_nav=alpha_nav)
                    sim_data = analyzer.run_simulation_with_analysis(use_absolute=True, steps=3000)
                    analysis = analyzer.analyze_stability_conditions(sim_data)
                    
                    trial_results['safety'].append(analysis['safety_maintained'])
                    trial_results['boundedness'].append(analysis['bounded'])
                    trial_results['ultimate_boundedness'].append(analysis['ultimate_bounded'])
                    trial_results['min_distance'].append(analysis['min_distance'])
                    
                except Exception as e:
                    continue
            
            # Compute averages for this parameter value
            if trial_results['safety']:  # If we have valid results
                safety_rates.append(np.mean(trial_results['safety']))
                boundedness_rates.append(np.mean(trial_results['boundedness']))
                ultimate_boundedness_rates.append(np.mean(trial_results['ultimate_boundedness']))
                min_distances.append(np.mean(trial_results['min_distance']))
            else:
                # No valid results
                safety_rates.append(0.0)
                boundedness_rates.append(0.0)
                ultimate_boundedness_rates.append(0.0)
                min_distances.append(0.0)
        
        return {
            'parameter_name': parameter_name,
            'parameter_values': param_values,
            'safety_rates': np.array(safety_rates),
            'boundedness_rates': np.array(boundedness_rates),
            'ultimate_boundedness_rates': np.array(ultimate_boundedness_rates),
            'min_distances': np.array(min_distances)
        }
    
    def plot_monte_carlo_results(self, mc_results):
        """
        Plot Monte Carlo test results
        """
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('Monte Carlo Verification Results', fontsize=16, weight='bold')
        
        # Success rates bar plot
        success_rates = [
            mc_results['safety_rate'],
            mc_results['boundedness_rate'],
            mc_results['ultimate_boundedness_rate']
        ]
        categories = ['Safety', 'Boundedness', 'Ultimate\nBoundedness']
        colors = ['green', 'blue', 'purple']
        
        bars = axes[0,0].bar(categories, success_rates, color=colors, alpha=0.7)
        axes[0,0].set_ylabel('Success Rate')
        axes[0,0].set_title('Stability Property Success Rates')
        axes[0,0].set_ylim(0, 1.1)
        axes[0,0].grid(True, alpha=0.3)
        
        # Add percentage labels on bars
        for bar, rate in zip(bars, success_rates):
            height = bar.get_height()
            axes[0,0].text(bar.get_x() + bar.get_width()/2., height + 0.01,
                         f'{rate:.1%}', ha='center', va='bottom', fontweight='bold')
        
        # Minimum distance distribution
        min_dists = mc_results['raw_data']['min_distances']
        axes[0,1].hist(min_dists, bins=30, alpha=0.7, color='orange', edgecolor='black')
        axes[0,1].axvline(mc_results['min_distance_stats']['mean'], color='red', 
                         linestyle='--', label=f"Mean: {mc_results['min_distance_stats']['mean']:.2f}")
        axes[0,1].set_xlabel('Minimum Distance (m)')
        axes[0,1].set_ylabel('Frequency')
        axes[0,1].set_title('Distribution of Minimum Distances')
        axes[0,1].legend()
        axes[0,1].grid(True, alpha=0.3)
        
        # Angular size distribution
        max_angles = mc_results['raw_data']['max_angular_sizes']
        axes[0,2].hist(max_angles, bins=30, alpha=0.7, color='cyan', edgecolor='black')
        axes[0,2].axvline(mc_results['max_angular_size_stats']['mean'], color='red', 
                         linestyle='--', label=f"Mean: {mc_results['max_angular_size_stats']['mean']:.2f}")
        axes[0,2].set_xlabel('Maximum Angular Size (degrees)')
        axes[0,2].set_ylabel('Frequency')
        axes[0,2].set_title('Distribution of Maximum Angular Sizes')
        axes[0,2].legend()
        axes[0,2].grid(True, alpha=0.3)
        
        # CBDR activation ratio distribution
        cbdr_ratios = mc_results['raw_data']['cbdr_ratios']
        axes[1,0].hist(cbdr_ratios, bins=30, alpha=0.7, color='magenta', edgecolor='black')
        axes[1,0].axvline(mc_results['cbdr_ratio_stats']['mean'], color='red', 
                         linestyle='--', label=f"Mean: {mc_results['cbdr_ratio_stats']['mean']:.3f}")
        axes[1,0].set_xlabel('CBDR Activation Ratio')
        axes[1,0].set_ylabel('Frequency')
        axes[1,0].set_title('Distribution of CBDR Activation Ratios')
        axes[1,0].legend()
        axes[1,0].grid(True, alpha=0.3)
        
        # Statistical summary table
        axes[1,1].axis('off')
        table_data = [
            ['Metric', 'Mean', 'Std Dev', '5th %ile', '95th %ile'],
            ['Min Distance (m)', 
             f"{mc_results['min_distance_stats']['mean']:.2f}",
             f"{mc_results['min_distance_stats']['std']:.2f}",
             f"{mc_results['min_distance_stats']['percentile_5']:.2f}",
             f"{mc_results['min_distance_stats']['percentile_95']:.2f}"],
            ['Max Angular Size (°)', 
             f"{mc_results['max_angular_size_stats']['mean']:.2f}",
             f"{mc_results['max_angular_size_stats']['std']:.2f}",
             f"{mc_results['max_angular_size_stats']['min']:.2f}",
             f"{mc_results['max_angular_size_stats']['max']:.2f}"],
            ['CBDR Ratio', 
             f"{mc_results['cbdr_ratio_stats']['mean']:.3f}",
             f"{mc_results['cbdr_ratio_stats']['std']:.3f}",
             f"{mc_results['cbdr_ratio_stats']['min']:.3f}",
             f"{mc_results['cbdr_ratio_stats']['max']:.3f}"]
        ]
        
        table = axes[1,1].table(cellText=table_data[1:], colLabels=table_data[0],
                              cellLoc='center', loc='center', bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 2)
        axes[1,1].set_title('Statistical Summary', pad=20)
        
        # Correlation analysis
        axes[1,2].scatter(min_dists, max_angles, alpha=0.6, color='navy')
        
        # Calculate correlation
        correlation = np.corrcoef(min_dists, max_angles)[0,1]
        axes[1,2].set_xlabel('Minimum Distance (m)')
        axes[1,2].set_ylabel('Maximum Angular Size (degrees)')
        axes[1,2].set_title(f'Distance vs Angular Size\n(Correlation: {correlation:.3f})')
        axes[1,2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def plot_sensitivity_results(self, sensitivity_results):
        """
        Plot parameter sensitivity analysis results
        """
        param_name = sensitivity_results['parameter_name']
        param_values = sensitivity_results['parameter_values']
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle(f'Parameter Sensitivity Analysis: {param_name}', fontsize=16, weight='bold')
        
        # Safety rate vs parameter
        axes[0,0].plot(param_values, sensitivity_results['safety_rates'], 'go-', 
                      linewidth=2, markersize=6, label='Safety Rate')
        axes[0,0].set_xlabel(f'{param_name}')
        axes[0,0].set_ylabel('Safety Success Rate')
        axes[0,0].set_title('Safety vs Parameter Value')
        axes[0,0].grid(True, alpha=0.3)
        axes[0,0].set_ylim(0, 1.1)
        
        # Boundedness rates
        axes[0,1].plot(param_values, sensitivity_results['boundedness_rates'], 'bo-', 
                      linewidth=2, markersize=6, label='Boundedness')
        axes[0,1].plot(param_values, sensitivity_results['ultimate_boundedness_rates'], 'ro-', 
                      linewidth=2, markersize=6, label='Ultimate Boundedness')
        axes[0,1].set_xlabel(f'{param_name}')
        axes[0,1].set_ylabel('Success Rate')
        axes[0,1].set_title('Boundedness Properties vs Parameter')
        axes[0,1].legend()
        axes[0,1].grid(True, alpha=0.3)
        axes[0,1].set_ylim(0, 1.1)
        
        # Minimum distances
        axes[1,0].plot(param_values, sensitivity_results['min_distances'], 'co-', 
                      linewidth=2, markersize=6)
        axes[1,0].set_xlabel(f'{param_name}')
        axes[1,0].set_ylabel('Average Minimum Distance (m)')
        axes[1,0].set_title('Minimum Distance vs Parameter')
        axes[1,0].grid(True, alpha=0.3)
        
        # Combined success rate (all properties)
        combined_success = (sensitivity_results['safety_rates'] * 
                          sensitivity_results['boundedness_rates'] * 
                          sensitivity_results['ultimate_boundedness_rates'])
        axes[1,1].plot(param_values, combined_success, 'mo-', 
                      linewidth=2, markersize=6, label='Combined Success')
        axes[1,1].set_xlabel(f'{param_name}')
        axes[1,1].set_ylabel('Combined Success Rate')
        axes[1,1].set_title('Overall System Performance')
        axes[1,1].grid(True, alpha=0.3)
        axes[1,1].set_ylim(0, 1.1)
        
        # Find optimal parameter value
        optimal_idx = np.argmax(combined_success)
        optimal_value = param_values[optimal_idx]
        axes[1,1].axvline(optimal_value, color='red', linestyle='--', alpha=0.7,
                         label=f'Optimal: {optimal_value:.2f}')
        axes[1,1].legend()
        
        plt.tight_layout()
        return fig, optimal_value
    
    def generate_comprehensive_report(self, mc_results, sensitivity_results_list):
        """
        Generate comprehensive verification report
        """
        print("\n" + "="*80)
        print("COMPREHENSIVE NUMERICAL VERIFICATION REPORT")
        print("="*80)
        
        # Monte Carlo Results
        print("\nMONTE CARLO ANALYSIS RESULTS:")
        print("-" * 50)
        print(f"Total Successful Trials: {mc_results['n_trials']}")
        print(f"Safety Success Rate: {mc_results['safety_rate']:.1%}")
        print(f"Boundedness Success Rate: {mc_results['boundedness_rate']:.1%}")
        print(f"Ultimate Boundedness Success Rate: {mc_results['ultimate_boundedness_rate']:.1%}")
        print()
        
        print("DISTANCE STATISTICS:")
        dist_stats = mc_results['min_distance_stats']
        print(f"  Mean Minimum Distance: {dist_stats['mean']:.3f} ± {dist_stats['std']:.3f} m")
        print(f"  Range: [{dist_stats['min']:.3f}, {dist_stats['max']:.3f}] m")
        print(f"  95% of trials: [{dist_stats['percentile_5']:.3f}, {dist_stats['percentile_95']:.3f}] m")
        print()
        
        # Sensitivity Analysis Results
        print("PARAMETER SENSITIVITY ANALYSIS:")
        print("-" * 50)
        
        for sens_result in sensitivity_results_list:
            param_name = sens_result['parameter_name']
            safety_rates = sens_result['safety_rates']
            
            print(f"\n{param_name.upper()} SENSITIVITY:")
            print(f"  Safety Rate Range: [{np.min(safety_rates):.1%}, {np.max(safety_rates):.1%}]")
            print(f"  Most Stable Range: {sens_result['parameter_values'][np.argmax(safety_rates)]:.2f}")
            
            # Find critical values where performance drops
            critical_threshold = 0.95  # 95% success rate
            stable_indices = safety_rates >= critical_threshold
            if np.any(stable_indices):
                stable_range = [
                    np.min(sens_result['parameter_values'][stable_indices]),
                    np.max(sens_result['parameter_values'][stable_indices])
                ]
                print(f"  Recommended Range (≥95% success): [{stable_range[0]:.2f}, {stable_range[1]:.2f}]")
            else:
                print(f"  Warning: No parameter values achieved ≥95% success rate")
        
        print()
        print("OVERALL ASSESSMENT:")
        print("-" * 50)
        
        # Overall system robustness
        overall_success = (mc_results['safety_rate'] * 
                         mc_results['boundedness_rate'] * 
                         mc_results['ultimate_boundedness_rate'])
        
        if overall_success >= 0.9:
            assessment = "EXCELLENT - System demonstrates robust stability"
        elif overall_success >= 0.8:
            assessment = "GOOD - System is generally stable with minor issues"
        elif overall_success >= 0.7:
            assessment = "ACCEPTABLE - System needs parameter tuning"
        else:
            assessment = "NEEDS IMPROVEMENT - Significant stability issues detected"
        
        print(f"Combined Success Rate: {overall_success:.1%}")
        print(f"System Assessment: {assessment}")
        
        # Recommendations
        print("\nRECOMMENDations:")
        print("-" * 50)
        
        if mc_results['safety_rate'] < 0.95:
            print("• Consider increasing R_safe for better safety margins")
        
        if mc_results['min_distance_stats']['percentile_5'] < 1.0:
            print("• Some scenarios result in very close approaches - review control gains")
        
        if mc_results['cbdr_ratio_stats']['mean'] < 0.3:
            print("• Low CBDR activation suggests aggressive maneuvering - consider tuning")
        
        print("• Regular Monte Carlo testing recommended for parameter validation")
        print("• Consider adaptive parameter adjustment based on scenario conditions")
        
        print("\n" + "="*80)
        
        return {
            'overall_success_rate': overall_success,
            'assessment': assessment,
            'mc_results': mc_results,
            'sensitivity_results': sensitivity_results_list
        }

def run_full_numerical_verification():
    """
    Run complete numerical verification suite
    """
    print("Starting Full Numerical Verification Suite...")
    print("This may take several minutes to complete.")
    print()
    
    # Initialize verifier
    verifier = NumericalVerifier()
    
    # Phase 1: Monte Carlo Testing
    print("Phase 1: Monte Carlo Analysis")
    print("-" * 40)
    mc_results = verifier.monte_carlo_test(n_trials=100)
    
    # Plot Monte Carlo results
    fig_mc = verifier.plot_monte_carlo_results(mc_results)
    plt.savefig('monte_carlo_results.png', dpi=300, bbox_inches='tight')
    print("Monte Carlo results saved as: monte_carlo_results.png")
    plt.show()
    
    # Phase 2: Sensitivity Analysis
    print("\nPhase 2: Parameter Sensitivity Analysis")
    print("-" * 40)
    
    sensitivity_results = []
    
    # Test R_safe sensitivity
    sens_R_safe = verifier.sensitivity_analysis('R_safe', (1.0, 4.0), n_points=15)
    sensitivity_results.append(sens_R_safe)
    
    fig_sens_R, optimal_R_safe = verifier.plot_sensitivity_results(sens_R_safe)
    plt.savefig('sensitivity_R_safe.png', dpi=300, bbox_inches='tight')
    print("R_safe sensitivity results saved as: sensitivity_R_safe.png")
    plt.show()
    
    # Test alpha_nav sensitivity
    sens_alpha = verifier.sensitivity_analysis('alpha_nav', (0.2, 3.0), n_points=15)
    sensitivity_results.append(sens_alpha)
    
    fig_sens_alpha, optimal_alpha = verifier.plot_sensitivity_results(sens_alpha)
    plt.savefig('sensitivity_alpha_nav.png', dpi=300, bbox_inches='tight')
    print("alpha_nav sensitivity results saved as: sensitivity_alpha_nav.png")
    plt.show()
    
    # Phase 3: Comprehensive Report
    print("\nPhase 3: Generating Comprehensive Report")
    print("-" * 40)
    
    final_report = verifier.generate_comprehensive_report(mc_results, sensitivity_results)
    
    # Save results to file
    results_summary = {
        'monte_carlo': mc_results,
        'sensitivity_analysis': [
            {k: v.tolist() if isinstance(v, np.ndarray) else v 
             for k, v in sens_result.items()} 
            for sens_result in sensitivity_results
        ],
        'final_assessment': final_report,
        'optimal_parameters': {
            'R_safe': optimal_R_safe,
            'alpha_nav': optimal_alpha
        }
    }
    
    with open('verification_results.json', 'w') as f:
        json.dump(results_summary, f, indent=2)
    
    print("Complete results saved as: verification_results.json")
    print("\nNumerical verification completed successfully!")
    
    return final_report

if __name__ == "__main__":
    run_full_numerical_verification()
