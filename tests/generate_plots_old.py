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
from test_case_loader import load_test_cases, get_test_category


class TestVisualizer:
    """Utility class for visualizing test trajectories."""

    def __init__(self, output_dir='tests/plots'):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.enabled = False

    def enable(self):
        """Enable visualization."""
        self.enabled = True

    def plot_trajectory(self, tpi, test_name, description=''):
        """
        Plot a trajectory from TwoPointInterpolation instance.

        Args:
            tpi: TwoPointInterpolation instance (after calc_trajectory())
            test_name: Name of the test (used for filename)
            description: Optional description to show in title
        """
        if not self.enabled:
            return

        try:
            total_time = sum(tpi.dt)
            if total_time == 0:
                return  # Skip zero-time trajectories

            # Generate time points
            t = np.linspace(0, total_time, 500)
            p_list, v_list, a_list = [], [], []

            for ti in t:
                p, v, a = tpi.get_point(ti)
                p_list.append(p)
                v_list.append(v)
                a_list.append(a)

            # Create figure
            fig, axes = plt.subplots(3, 1, figsize=(10, 8))
            fig.suptitle(f'{test_name}\n{description}', fontsize=12)

            # Position
            axes[0].plot(t, p_list, 'b-', linewidth=2, label='Position')
            axes[0].axhline(y=tpi.pe, color='r', linestyle='--', alpha=0.5, label=f'Target={tpi.pe:.2f}')
            axes[0].axhline(y=tpi.p0, color='g', linestyle='--', alpha=0.5, label=f'Start={tpi.p0:.2f}')
            axes[0].set_ylabel('Position [m]')
            axes[0].legend(loc='best')
            axes[0].grid(True, alpha=0.3)

            # Velocity
            axes[1].plot(t, v_list, 'g-', linewidth=2, label='Velocity')
            axes[1].axhline(y=tpi.vmax, color='orange', linestyle='--', alpha=0.5, label=f'vmax={tpi.vmax:.2f}')
            axes[1].axhline(y=-tpi.vmax, color='orange', linestyle='--', alpha=0.5)
            axes[1].axhline(y=tpi.v0, color='purple', linestyle=':', alpha=0.5, label=f'v0={tpi.v0:.2f}')
            axes[1].axhline(y=tpi.ve, color='brown', linestyle=':', alpha=0.5, label=f've={tpi.ve:.2f}')
            axes[1].set_ylabel('Velocity [m/s]')
            axes[1].legend(loc='best')
            axes[1].grid(True, alpha=0.3)

            # Acceleration
            axes[2].plot(t, a_list, 'r-', linewidth=2, label='Acceleration')
            axes[2].axhline(y=0, color='k', linestyle='-', alpha=0.3)
            axes[2].axhline(y=tpi.amax_accel, color='orange', linestyle='--', alpha=0.5, label=f'amax={tpi.amax_accel:.2f}')
            axes[2].axhline(y=-tpi.amax_decel, color='orange', linestyle='--', alpha=0.5, label=f'dec_max={tpi.amax_decel:.2f}')
            axes[2].set_xlabel('Time [s]')
            axes[2].set_ylabel('Acceleration [m/s²]')
            axes[2].legend(loc='best')
            axes[2].grid(True, alpha=0.3)

            # Add phase information
            info_text = f'Case: {tpi.case}\n'
            info_text += f'Phases: {len(tpi.dt)}\n'
            info_text += f'Total time: {total_time:.3f}s'
            axes[2].text(0.02, 0.98, info_text, transform=axes[2].transAxes,
                        verticalalignment='top', fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            plt.tight_layout()

            # Save figure
            filename = f"{test_name.replace(' ', '_').replace('/', '_')}.png"
            filepath = self.output_dir / filename
            plt.savefig(filepath, dpi=100, bbox_inches='tight')
            plt.close(fig)

            print(f"  📊 Plot saved: {filepath}")

        except Exception as e:
            print(f"  ⚠️  Failed to plot {test_name}: {e}")


def get_visualizer():
    """Get the global test visualizer instance."""
    return TestVisualizer()


def run_test_case(test_case: dict, visualizer) -> bool:
    """
    Run a single test case and generate its visualization.

    Args:
        test_case: Test case dictionary from YAML
        visualizer: TestVisualizer instance

    Returns:
        True if successful, False if error (expected for error_tests)
    """
    name = test_case['name']
    description = test_case['description']
    params = test_case['params']

    print(f"  {name}...")

    tpi = TwoPointInterpolation()
    tpi.init(**params)

    try:
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
            raise


def main():
    """Generate plots for test cases from YAML."""
    parser = argparse.ArgumentParser(
        description='Generate trajectory visualization plots from YAML test cases'
    )
    parser.add_argument(
        '--category',
        type=str,
        default='visualization_tests',
        help='Test category to visualize (default: visualization_tests, use "all" for all categories)'
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
            print(f"  - {category}: {len(cases)} test(s)")
        return

    # Setup visualizer
    vis = get_visualizer()
    vis.enable()

    print("📊 Generating visualization plots from YAML test cases...")
    print()

    # Determine which categories to process
    if args.category == 'all':
        all_cases = load_test_cases()
        categories = list(all_cases.keys())
    else:
        categories = [args.category]

    # Process each category
    total_success = 0
    total_skipped = 0

    for category in categories:
        print(f"📁 Category: {category}")
        try:
            test_cases = get_test_category(category)

            for test_case in test_cases:
                success = run_test_case(test_case, vis)
                if success:
                    total_success += 1
                else:
                    total_skipped += 1

            print()

        except ValueError as e:
            print(f"  ❌ Error: {e}")
            print()
            continue

    print("="*60)
    print(f"✅ Generated {total_success} plot(s) successfully!")
    if total_skipped > 0:
        print(f"⚠️  Skipped {total_skipped} error case(s) (expected)")
    print(f"   Location: {vis.output_dir}/")
    print("="*60)


if __name__ == "__main__":
    main()
