#!/usr/bin/env python3
"""
Generate visualization plots for test cases defined in test_cases.yaml.

This script reads test case definitions from YAML and generates trajectory
plots for visualization and documentation purposes. It can visualize any
test case that successfully calculates a trajectory.

Usage:
    python tests/generate_plots.py                           # Generate representative cases
    python tests/generate_plots.py --all                     # Generate all successful cases
    python tests/generate_plots.py --category basic_tests    # Specific category
    python tests/generate_plots.py --case test_case_name     # Specific test case
"""

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import argparse
import warnings
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
from two_point_interpolation import TwoPointInterpolation
from test_case_loader import load_test_cases


class TestVisualizer:
    """Utility class for visualizing test trajectories."""

    def __init__(self, output_dir='tests/plots'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def plot_trajectory(self, tpi, test_name, description=''):
        """
        Plot a trajectory from TwoPointInterpolation instance.

        Args:
            tpi: TwoPointInterpolation instance (after calc_trajectory())
            test_name: Name of the test (used for filename)
            description: Optional description to show in title
        """
        # Calculate total time and time points
        total_time = sum(tpi.dt)
        if total_time <= 0:
            print(f"    ⚠️  Skipping zero-time trajectory: {test_name}")
            return

        # Generate time series
        time_points = np.linspace(0, total_time, 200)
        positions = []
        velocities = []
        accelerations = []

        for t in time_points:
            p, v, a = tpi.get_point(t)
            positions.append(p)
            velocities.append(v)
            accelerations.append(a)

        # Create plots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

        # Position plot
        ax1.plot(time_points, positions, 'b-', linewidth=2, label='Position')
        ax1.axhline(y=tpi.p0, color='g', linestyle='--', alpha=0.7, label=f'Start: {tpi.p0:.3f}')
        ax1.axhline(y=tpi.pe, color='r', linestyle='--', alpha=0.7, label=f'End: {tpi.pe:.3f}')
        ax1.set_ylabel('Position [m]')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.set_title(f'{test_name}\n{description}')

        # Velocity plot
        ax2.plot(time_points, velocities, 'g-', linewidth=2, label='Velocity')
        ax2.axhline(y=tpi.vmax, color='orange', linestyle=':', alpha=0.7, label=f'vmax: {tpi.vmax:.3f}')
        ax2.axhline(y=-tpi.vmax, color='orange', linestyle=':', alpha=0.7)
        ax2.axhline(y=tpi.v0, color='g', linestyle='--', alpha=0.7, label=f'Start: {tpi.v0:.3f}')
        ax2.axhline(y=tpi.ve, color='r', linestyle='--', alpha=0.7, label=f'End: {tpi.ve:.3f}')
        ax2.set_ylabel('Velocity [m/s]')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Acceleration plot
        ax3.plot(time_points, accelerations, 'r-', linewidth=2, label='Acceleration')
        ax3.axhline(y=tpi.amax_accel, color='orange', linestyle=':', alpha=0.7, label=f'amax: {tpi.amax_accel:.3f}')
        ax3.axhline(y=-tpi.amax_decel, color='orange', linestyle=':', alpha=0.7, label=f'dmax: {tpi.amax_decel:.3f}')
        ax3.set_ylabel('Acceleration [m/s²]')
        ax3.set_xlabel('Time [s]')
        ax3.grid(True, alpha=0.3)
        ax3.legend()

        # Add phase boundaries
        cumulative_time = 0
        for i, dt in enumerate(tpi.dt[:-1]):  # Don't mark the final time
            cumulative_time += dt
            for ax in [ax1, ax2, ax3]:
                ax.axvline(x=cumulative_time, color='black', linestyle=':', alpha=0.5)

        # Add text info
        case_text = f"Case: {tpi.case}, Total time: {total_time:.3f}s"
        if hasattr(tpi, 'dt') and len(tpi.dt) > 0:
            phase_text = f"Phases: {[f'{dt:.3f}' for dt in tpi.dt]}"
            case_text += f"\n{phase_text}"

        fig.text(0.02, 0.02, case_text, fontsize=8, verticalalignment='bottom')

        plt.tight_layout()
        plt.subplots_adjust(bottom=0.1)

        # Save plot
        safe_filename = "".join(c for c in test_name if c.isalnum() or c in ('_', '-')).rstrip()
        output_path = self.output_dir / f"{safe_filename}.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"    ✅ Saved: {output_path}")


# Global visualizer instance
_visualizer = None


def get_visualizer():
    """Get or create the global visualizer instance."""
    global _visualizer
    if _visualizer is None:
        _visualizer = TestVisualizer()
    return _visualizer


def run_test_case(test_case, visualizer):
    """
    Run a single test case and generate visualization.

    Args:
        test_case: Test case dictionary from YAML
        visualizer: TestVisualizer instance

    Returns:
        bool: True if successful, False if failed/skipped
    """
    name = test_case['name']
    description = test_case.get('description', '')
    params = test_case.get('params', {})

    print(f"  📊 {name}")

    if not params:
        print(f"    ⚠️  No parameters - skipping")
        return False

    try:
        # Create and initialize trajectory
        tpi = TwoPointInterpolation()
        tpi.init(**params)

        # Calculate trajectory (ignore warnings)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            tpi.calc_trajectory()

        # Generate plot
        visualizer.plot_trajectory(tpi, name, description)
        return True

    except ValueError as e:
        # Expected error for error_tests category
        if 'expected' in test_case and 'error' in test_case['expected']:
            print(f"    ⚠️  Expected error: {str(e)[:60]}...")
            return False
        else:
            print(f"    ❌ Unexpected error: {e}")
            return False


def is_visualizable(test_case):
    """Check if a test case is suitable for visualization."""
    # Skip error cases, constraint validation, and utility tests
    test_type = test_case.get('test_type', 'standard')
    if test_type in ['error', 'constraint_validation', 'utility', 'no_init']:
        return False

    # Skip cases that expect errors
    expected = test_case.get('expected', {})
    if 'error' in expected:
        return False

    # Must have parameters for trajectory calculation
    if not test_case.get('params'):
        return False

    return True


def select_representative_cases(all_cases):
    """Select representative test cases for visualization."""
    representatives = []

    # Define priority cases from each category
    priority_cases = {
        'basic_tests': ['test_dp0_dv0_case'],
        'case0_tests': ['test_case0_forward_zero_velocities'],
        'case1_tests': ['test_case1_forward_zero_velocities'],
        'performance_tests': ['test_faster_deceleration'],
        'overspeed_tests': ['test_overspeed_warning'],
        'boundary_tests': ['test_initial_position', 'test_final_position']
    }

    for category, cases in all_cases.items():
        if category in priority_cases:
            # Use priority cases
            for case_name in priority_cases[category]:
                case = next((c for c in cases if c['name'] == case_name), None)
                if case and is_visualizable(case):
                    representatives.append((category, [case]))
        elif category not in ['error_tests', 'constraint_tests', 'integration_tests']:
            # For other categories, take first visualizable case
            visualizable_cases = [c for c in cases if is_visualizable(c)]
            if visualizable_cases:
                representatives.append((category, [visualizable_cases[0]]))

    return representatives


def find_test_case_by_name(case_name):
    """Find a test case by name across all categories."""
    all_cases = load_test_cases()
    for category, cases in all_cases.items():
        for case in cases:
            if case['name'] == case_name:
                return case
    return None


def main():
    """Main function to generate trajectory plots."""
    parser = argparse.ArgumentParser(description='Generate trajectory plots from YAML test cases')
    parser.add_argument(
        '--category',
        default='representative',
        help='Test category to visualize (default: representative selection)'
    )
    parser.add_argument(
        '--case',
        help='Specific test case name to visualize'
    )
    parser.add_argument(
        '--all',
        action='store_true',
        help='Generate plots for all successful test cases'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List all available categories and exit'
    )

    args = parser.parse_args()

    # List categories and exit
    if args.list:
        all_cases = load_test_cases()
        print("Available test categories:")
        for category, cases in all_cases.items():
            visualizable_count = sum(1 for case in cases if is_visualizable(case))
            print(f"  - {category}: {visualizable_count}/{len(cases)} visualizable test(s)")
        return

    # Setup visualizer
    vis = get_visualizer()

    print("📊 Generating trajectory visualization plots...")
    print()

    # Process specific test case
    if args.case:
        test_case = find_test_case_by_name(args.case)
        if test_case:
            success = run_test_case(test_case, vis)
            print(f"✅ Generated plot for: {args.case}" if success else f"❌ Failed to generate: {args.case}")
        else:
            print(f"❌ Test case not found: {args.case}")
        return

    # Determine which categories to process
    all_cases = load_test_cases()

    if args.all:
        # Process all visualizable cases
        categories = [(cat, cases) for cat, cases in all_cases.items()]
    elif args.category == 'representative':
        # Process representative cases from each category
        categories = select_representative_cases(all_cases)
    else:
        # Process specific category
        if args.category in all_cases:
            categories = [(args.category, all_cases[args.category])]
        else:
            print(f"❌ Category not found: {args.category}")
            return

    # Process each category
    total_success = 0
    total_skipped = 0

    for category_name, test_cases in categories:
        print(f"📁 Category: {category_name}")

        for test_case in test_cases:
            if not is_visualizable(test_case):
                total_skipped += 1
                continue

            success = run_test_case(test_case, vis)
            if success:
                total_success += 1
            else:
                total_skipped += 1

        print()

    print("="*60)
    print(f"✅ Generated {total_success} plot(s) successfully!")
    if total_skipped > 0:
        print(f"⚠️  Skipped {total_skipped} case(s) (errors or unsuitable for visualization)")
    print(f"   Location: {vis.output_dir}/")
    print("="*60)


if __name__ == "__main__":
    main()
