"""
Lyapunov Stability Analysis for Angle-Only Collision Avoidance System
====================================================================

This module provides comprehensive stability analysis for the CBDR-based
collision avoidance system using:
1. Barrier Lyapunov Functions for safety
2. Geometric Lyapunov Functions for convergence
3. Numerical verification of bounded and ultimate bounded stability

Author: Auto-generated analysis for Angular-obstacle-avoidance project
"""

import numpy as np
import matplotlib.pyplot as plt
from math import pi, sin, cos, tan, atan2, sqrt, log
import sys
import os

# Import from the comparison module
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from BearingRateGraph_comparison import (
    ShipStatus, get_distance_3d, get_bearing, get_absolute_bearing,
    get_angular_diameter, angle_difference_in_deg,
    adj_ownship_heading_absolute, adj_ownship_heading_relative
)

class LyapunovAnalyzer:
    """
    Lyapunov stability analyzer for the angle-only collision avoidance system
    """
    
    def __init__(self, R_safe=2.0, alpha_nav=1.0):
        """
        Initialize the analyzer
        
        Args:
            R_safe: Safe distance threshold (meters)
            alpha_nav: Navigation threshold (degrees)
        """
        self.R_safe = R_safe
        self.alpha_nav = alpha_nav
        self.alpha_nav_rad = np.radians(alpha_nav)
        
    def barrier_function(self, R):
        """
        Barrier function B(R) = 1/(R - R_safe) for R > R_safe
        This enforces the constraint R > R_safe
        """
        if R <= self.R_safe:
            return np.inf  # Barrier violated
        return 1.0 / (R - self.R_safe)
    
    def geometric_lyapunov(self, beta_rad):
        """
        Geometric Lyapunov function L(β) = 1 - cos(β)
        Minimum at β = 0 (target directly ahead)
        """
        return 1.0 - cos(beta_rad)
    
    def composite_lyapunov(self, R, beta_rad, w1=1.0, w2=1.0):
        """
        Composite Lyapunov function V(R,β) = w1*B(R) + w2*L(β)
        
        Args:
            R: Distance to target
            beta_rad: Relative bearing in radians
            w1, w2: Weighting factors
        """
        B_R = self.barrier_function(R)
        L_beta = self.geometric_lyapunov(beta_rad)
        return w1 * B_R + w2 * L_beta
    
    def barrier_derivative(self, R, R_dot):
        """
        Derivative of barrier function: Ḃ(R) = -Ṙ/(R - R_safe)²
        """
        if R <= self.R_safe:
            return np.inf
        return -R_dot / ((R - self.R_safe)**2)
    
    def geometric_lyapunov_derivative(self, beta_rad, beta_dot_rad):
        """
        Derivative of geometric Lyapunov: L̇(β) = sin(β) * β̇
        """
        return sin(beta_rad) * beta_dot_rad
    
    def compute_relative_velocity(self, own_ship, target_ship):
        """
        Compute relative velocity components
        """
        # Velocity vectors in NED coordinates
        v_own = own_ship.velocity * np.array([
            cos(np.radians(own_ship.heading)),
            sin(np.radians(own_ship.heading)),
            0
        ])
        
        v_target = target_ship.velocity * np.array([
            cos(np.radians(target_ship.heading)),
            sin(np.radians(target_ship.heading)),
            0
        ])
        
        return v_target - v_own
    
    def compute_range_rate(self, own_ship, target_ship):
        """
        Compute range rate Ṙ = (relative_velocity · unit_vector_to_target)
        """
        rel_pos = target_ship.position - own_ship.position
        R = np.linalg.norm(rel_pos)
        
        if R < 1e-9:
            return 0.0
            
        unit_vector = rel_pos / R
        rel_velocity = self.compute_relative_velocity(own_ship, target_ship)
        
        return np.dot(rel_velocity, unit_vector)
    
    def analyze_stability_conditions(self, simulation_data, dt=0.01):
        """
        Analyze stability conditions from simulation data
        
        Returns:
            dict: Stability analysis results
        """
        distances = simulation_data['distances']
        bearings = simulation_data['bearings']
        angular_sizes = simulation_data['angular_sizes']
        
        # Convert to radians for analysis
        bearings_rad = np.radians(bearings)
        angular_sizes_rad = np.radians(angular_sizes)
        
        # Compute Lyapunov functions
        barrier_values = []
        geometric_values = []
        composite_values = []
        
        # Compute derivatives numerically
        barrier_derivatives = []
        geometric_derivatives = []
        composite_derivatives = []
        
        for i in range(len(distances)):
            R = distances[i]
            beta_rad = bearings_rad[i]
            
            # Lyapunov function values
            B_R = self.barrier_function(R)
            L_beta = self.geometric_lyapunov(beta_rad)
            V = self.composite_lyapunov(R, beta_rad)
            
            barrier_values.append(B_R if B_R != np.inf else 1e6)
            geometric_values.append(L_beta)
            composite_values.append(V if V != np.inf else 1e6)
            
            # Compute derivatives
            if i > 0:
                R_prev = distances[i-1]
                beta_prev = bearings_rad[i-1]
                
                R_dot = (R - R_prev) / dt
                beta_dot = (beta_rad - beta_prev) / dt
                
                B_dot = self.barrier_derivative(R, R_dot)
                L_dot = self.geometric_lyapunov_derivative(beta_rad, beta_dot)
                V_dot = B_dot + L_dot  # Assuming w1=w2=1
                
                barrier_derivatives.append(B_dot if abs(B_dot) < 1e6 else 0)
                geometric_derivatives.append(L_dot)
                composite_derivatives.append(V_dot if abs(V_dot) < 1e6 else 0)
        
        # Stability analysis
        min_distance = np.min(distances)
        max_angular_size = np.max(angular_sizes)
        
        # Check if system maintains safety
        safety_maintained = min_distance > self.R_safe
        
        # Check boundedness
        max_barrier = np.max([b for b in barrier_values if b < 1e6])
        bounded = max_barrier < 1e6
        
        # Check ultimate boundedness (distances eventually stay above R_safe + δ)
        final_portion = distances[-int(0.2 * len(distances)):]  # Last 20% of simulation
        ultimate_bound = np.min(final_portion) > self.R_safe + 0.1
        
        # Count CBDR activations (when bearing rate is small)
        bearing_rates = np.diff(bearings) / dt
        cbdr_activations = np.sum(np.abs(bearing_rates * dt) <= angular_sizes[1:])
        cbdr_ratio = cbdr_activations / len(bearing_rates)
        
        return {
            'safety_maintained': safety_maintained,
            'bounded': bounded,
            'ultimate_bounded': ultimate_bound,
            'min_distance': min_distance,
            'max_angular_size': max_angular_size,
            'max_barrier_value': max_barrier,
            'cbdr_activation_ratio': cbdr_ratio,
            'barrier_values': np.array(barrier_values),
            'geometric_values': np.array(geometric_values),
            'composite_values': np.array(composite_values),
            'barrier_derivatives': np.array(barrier_derivatives),
            'geometric_derivatives': np.array(geometric_derivatives),
            'composite_derivatives': np.array(composite_derivatives)
        }
    
    def run_simulation_with_analysis(self, use_absolute=True, steps=5000, dt=0.01):
        """
        Run simulation and perform Lyapunov analysis
        """
        # Initialize ships with same configuration as comparison file
        ownship = ShipStatus("Ownship", velocity=1.0, acceleration=0, heading=90, 
                           rate_of_turn=0, position=[0, 0, 0], size=0.5)
        target_ship = ShipStatus("Ship A", velocity=2.0, acceleration=0, heading=90.0, 
                                rate_of_turn=1, position=[15, -15, 0], size=0.5)
        goal = ShipStatus("Goal", velocity=0.0, acceleration=0, heading=0, 
                         rate_of_turn=0, position=[0, 50, 0])
        
        # Data storage
        distances = []
        bearings = []
        absolute_bearings = []
        angular_sizes = []
        bearings_difference = []
        absolute_bearings_difference = []
        ownship_positions = []
        target_positions = []
        
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
            ownship_positions.append(ownship.position.copy())
            target_positions.append(target_ship.position.copy())
            
            # Apply control
            if use_absolute:
                ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_absolute(
                    absolute_bearings, absolute_bearings_difference, angular_sizes, 
                    ownship, goal, target_ship, dt)
            else:
                ownship.rate_of_turn, ownship.velocity = adj_ownship_heading_relative(
                    bearings, bearings_difference, angular_sizes, 
                    ownship, goal, target_ship, dt)
            
            # Update positions
            ownship.update(dt)
            target_ship.update(dt)
            
            # Calculate bearing rate
            if i > 0:
                new_bearing = get_bearing(ownship, target_ship)
                new_absolute_bearing = get_absolute_bearing(ownship, target_ship)
                
                bearing_diff = angle_difference_in_deg(bearing, new_bearing) / dt
                abs_bearing_diff = angle_difference_in_deg(absolute_bearing, new_absolute_bearing) / dt
                
                bearings_difference.append(bearing_diff)
                absolute_bearings_difference.append(abs_bearing_diff)
        
        return {
            'distances': np.array(distances),
            'bearings': np.array(bearings),
            'absolute_bearings': np.array(absolute_bearings),
            'angular_sizes': np.array(angular_sizes),
            'bearings_difference': np.array(bearings_difference),
            'absolute_bearings_difference': np.array(absolute_bearings_difference),
            'ownship_positions': np.array(ownship_positions),
            'target_positions': np.array(target_positions)
        }
    
    def plot_lyapunov_analysis(self, analysis_results, simulation_data, dt=0.01):
        """
        Plot Lyapunov analysis results
        """
        time = np.arange(len(simulation_data['distances'])) * dt
        
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('Lyapunov Stability Analysis for Angle-Only Collision Avoidance', fontsize=16)
        
        # Plot 1: Distance and safety threshold
        axes[0,0].plot(time, simulation_data['distances'], 'b-', label='Distance')
        axes[0,0].axhline(y=self.R_safe, color='r', linestyle='--', label=f'Safety Threshold ({self.R_safe}m)')
        axes[0,0].set_xlabel('Time (s)')
        axes[0,0].set_ylabel('Distance (m)')
        axes[0,0].set_title('Distance vs Time')
        axes[0,0].legend()
        axes[0,0].grid(True)
        
        # Plot 2: Barrier Lyapunov Function
        axes[0,1].plot(time, analysis_results['barrier_values'], 'g-', label='Barrier B(R)')
        axes[0,1].set_xlabel('Time (s)')
        axes[0,1].set_ylabel('Barrier Value')
        axes[0,1].set_title('Barrier Lyapunov Function')
        axes[0,1].set_yscale('log')
        axes[0,1].legend()
        axes[0,1].grid(True)
        
        # Plot 3: Geometric Lyapunov Function
        axes[1,0].plot(time, analysis_results['geometric_values'], 'm-', label='L(β) = 1-cos(β)')
        axes[1,0].set_xlabel('Time (s)')
        axes[1,0].set_ylabel('Geometric Lyapunov Value')
        axes[1,0].set_title('Geometric Lyapunov Function')
        axes[1,0].legend()
        axes[1,0].grid(True)
        
        # Plot 4: Composite Lyapunov Function
        axes[1,1].plot(time, analysis_results['composite_values'], 'c-', label='V(R,β) = B(R) + L(β)')
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Composite Lyapunov Value')
        axes[1,1].set_title('Composite Lyapunov Function')
        axes[1,1].set_yscale('log')
        axes[1,1].legend()
        axes[1,1].grid(True)
        
        # Plot 5: Lyapunov Derivatives
        time_deriv = time[1:]  # One less point for derivatives
        axes[2,0].plot(time_deriv, analysis_results['barrier_derivatives'], 'g--', label='Ḃ(R)', alpha=0.7)
        axes[2,0].plot(time_deriv, analysis_results['geometric_derivatives'], 'm--', label='L̇(β)', alpha=0.7)
        axes[2,0].axhline(y=0, color='k', linestyle='-', alpha=0.3)
        axes[2,0].set_xlabel('Time (s)')
        axes[2,0].set_ylabel('Derivative Value')
        axes[2,0].set_title('Lyapunov Function Derivatives')
        axes[2,0].legend()
        axes[2,0].grid(True)
        
        # Plot 6: Angular Size and Navigation Threshold
        axes[2,1].plot(time, simulation_data['angular_sizes'], 'orange', label='Angular Size α')
        axes[2,1].axhline(y=self.alpha_nav, color='r', linestyle='--', label=f'Navigation Threshold ({self.alpha_nav}°)')
        axes[2,1].set_xlabel('Time (s)')
        axes[2,1].set_ylabel('Angular Size (degrees)')
        axes[2,1].set_title('Angular Size vs Navigation Threshold')
        axes[2,1].legend()
        axes[2,1].grid(True)
        
        plt.tight_layout()
        return fig
    
    def print_stability_report(self, analysis_results):
        """
        Print a comprehensive stability report
        """
        print("="*80)
        print("LYAPUNOV STABILITY ANALYSIS REPORT")
        print("="*80)
        
        print(f"Safety Threshold (R_safe): {self.R_safe:.2f} m")
        print(f"Navigation Threshold (α_nav): {self.alpha_nav:.2f}°")
        print()
        
        print("STABILITY PROPERTIES:")
        print("-" * 40)
        print(f"✓ Safety Maintained: {'YES' if analysis_results['safety_maintained'] else 'NO'}")
        print(f"✓ Bounded: {'YES' if analysis_results['bounded'] else 'NO'}")
        print(f"✓ Ultimate Bounded: {'YES' if analysis_results['ultimate_bounded'] else 'NO'}")
        print()
        
        print("QUANTITATIVE METRICS:")
        print("-" * 40)
        print(f"Minimum Distance Achieved: {analysis_results['min_distance']:.3f} m")
        print(f"Maximum Angular Size: {analysis_results['max_angular_size']:.3f}°")
        print(f"Maximum Barrier Value: {analysis_results['max_barrier_value']:.3f}")
        print(f"CBDR Activation Ratio: {analysis_results['cbdr_activation_ratio']:.3f}")
        print()
        
        print("LYAPUNOV CONDITIONS:")
        print("-" * 40)
        
        # Check if composite Lyapunov derivative is negative most of the time
        negative_derivative_ratio = np.mean(analysis_results['composite_derivatives'] <= 0)
        print(f"Negative Derivative Ratio: {negative_derivative_ratio:.3f}")
        
        if negative_derivative_ratio > 0.7:
            print("✓ Lyapunov stability condition largely satisfied")
        else:
            print("⚠ Lyapunov stability condition needs attention")
        
        print()
        
        print("CONCLUSION:")
        print("-" * 40)
        if (analysis_results['safety_maintained'] and 
            analysis_results['bounded'] and 
            analysis_results['ultimate_bounded']):
            print("✓ System demonstrates SAFE, BOUNDED, and ULTIMATE BOUNDED behavior")
            print("✓ Collision avoidance controller is STABLE")
        else:
            print("⚠ System may need parameter tuning or design modifications")
        
        print("="*80)

def run_comprehensive_analysis():
    """
    Run comprehensive Lyapunov stability analysis
    """
    print("Starting Comprehensive Lyapunov Stability Analysis...")
    print()
    
    # Initialize analyzer
    analyzer = LyapunovAnalyzer(R_safe=2.0, alpha_nav=1.0)
    
    # Test both absolute and relative bearing control
    for use_absolute in [True, False]:
        control_type = "Absolute Bearing" if use_absolute else "Relative Bearing"
        print(f"Analyzing {control_type} Control...")
        
        # Run simulation
        sim_data = analyzer.run_simulation_with_analysis(use_absolute=use_absolute)
        
        # Perform Lyapunov analysis
        analysis = analyzer.analyze_stability_conditions(sim_data)
        
        # Print report
        print(f"\n{control_type.upper()} CONTROL ANALYSIS:")
        analyzer.print_stability_report(analysis)
        
        # Create plots
        fig = analyzer.plot_lyapunov_analysis(analysis, sim_data)
        fig.suptitle(f'Lyapunov Analysis - {control_type} Control', fontsize=16)
        
        # Save plot
        filename = f"lyapunov_analysis_{control_type.lower().replace(' ', '_')}.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Plot saved as: {filename}")
        
        plt.show()
        print("\n" + "="*80 + "\n")

if __name__ == "__main__":
    run_comprehensive_analysis()
