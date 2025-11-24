#!/usr/bin/env python3
"""
Example script for constant acceleration interpolation
"""

import sys
import os
import argparse
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import matplotlib.pyplot as plt
from two_point_interpolation import TwoPointInterpolation


def generate_case0_example(show=True, save=False):
    """Generate Case 0 example (vmax not reached)"""
    print("\n=== Case 0: vmax not reached ===")

    tpi = TwoPointInterpolation()

    # Initial and target states
    t0 = 1.0
    p0, pe = 5.0, 15.0
    v0, ve = 0.0, 0.0

    # Constraints
    acc_max = 2.0
    dec_max = 3.0
    vmax = 10.0

    tpi.init(p0, pe, acc_max, vmax, t0=t0, v0=v0, ve=ve, dec_max=dec_max)
    total_time = tpi.calc_trajectory()

    print(f"Initial time: {t0:.3f} s")
    print(f"Duration: {total_time:.3f} s")
    print(f"End time: {t0 + total_time:.3f} s")

    # Sample trajectory
    dt = 0.01
    t_array = np.arange(t0, t0 + total_time, dt)
    p_array = []
    v_array = []
    a_array = []

    for t in t_array:
        p, v, a = tpi.get_point(t)
        p_array.append(p)
        v_array.append(v)
        a_array.append(a)

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    axes[0].plot(t_array, p_array, 'b-', linewidth=2)
    axes[0].set_ylabel('Position [m]')
    axes[0].grid(True)
    axes[0].axhline(y=pe, color='r', linestyle='--', alpha=0.5, label='Target')
    axes[0].legend()

    axes[1].plot(t_array, v_array, 'g-', linewidth=2)
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].grid(True)
    axes[1].axhline(y=vmax, color='r', linestyle='--', alpha=0.5, label='vmax (not reached)')
    axes[1].legend()

    axes[2].plot(t_array, a_array, 'r-', linewidth=2)
    axes[2].set_ylabel('Acceleration [m/s²]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True)
    axes[2].axhline(y=acc_max, color='orange', linestyle='--', alpha=0.5, label=f'acc_max={acc_max}')
    axes[2].axhline(y=-dec_max, color='purple', linestyle='--', alpha=0.5, label=f'-dec_max={-dec_max}')
    axes[2].legend()

    plt.suptitle(f'Case 0: vmax not reached (t0={t0}, acc={acc_max}, dec={dec_max})')
    plt.tight_layout()

    if save:
        images_dir = os.path.join(os.path.dirname(__file__), 'images')
        os.makedirs(images_dir, exist_ok=True)
        output_path = os.path.join(images_dir, 'acc_constant_0.png')
        plt.savefig(output_path, dpi=150)
        print(f"Saved: {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


def generate_case1_example(show=True, save=False):
    """Generate Case 1 example (vmax reached)"""
    print("\n=== Case 1: vmax reached ===")

    tpi = TwoPointInterpolation()

    # Initial and target states
    t0 = 0.0
    p0, pe = 0.0, 50.0
    v0, ve = 2.0, 1.0

    # Constraints
    acc_max = 2.0
    dec_max = 4.0
    vmax = 8.0

    tpi.init(p0, pe, acc_max, vmax, t0=t0, v0=v0, ve=ve, dec_max=dec_max)
    total_time = tpi.calc_trajectory()

    print(f"Initial velocity: {v0:.3f} m/s")
    print(f"Final velocity: {ve:.3f} m/s")
    print(f"Duration: {total_time:.3f} s")

    # Sample trajectory
    dt = 0.01
    t_array = np.arange(t0, t0 + total_time, dt)
    p_array = []
    v_array = []
    a_array = []

    for t in t_array:
        p, v, a = tpi.get_point(t)
        p_array.append(p)
        v_array.append(v)
        a_array.append(a)

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))

    axes[0].plot(t_array, p_array, 'b-', linewidth=2)
    axes[0].set_ylabel('Position [m]')
    axes[0].grid(True)
    axes[0].axhline(y=pe, color='r', linestyle='--', alpha=0.5, label='Target')
    axes[0].legend()

    axes[1].plot(t_array, v_array, 'g-', linewidth=2)
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].grid(True)
    axes[1].axhline(y=vmax, color='r', linestyle='--', alpha=0.5, label=f'vmax={vmax}')
    axes[1].legend()

    axes[2].plot(t_array, a_array, 'r-', linewidth=2)
    axes[2].set_ylabel('Acceleration [m/s²]')
    axes[2].set_xlabel('Time [s]')
    axes[2].grid(True)
    axes[2].axhline(y=acc_max, color='orange', linestyle='--', alpha=0.5, label=f'acc_max={acc_max}')
    axes[2].axhline(y=-dec_max, color='purple', linestyle='--', alpha=0.5, label=f'-dec_max={-dec_max}')
    axes[2].legend()

    plt.suptitle(f'Case 1: vmax reached (v0={v0}, ve={ve}, acc={acc_max}, dec={dec_max})')
    plt.tight_layout()

    if save:
        images_dir = os.path.join(os.path.dirname(__file__), 'images')
        os.makedirs(images_dir, exist_ok=True)
        output_path = os.path.join(images_dir, 'acc_constant_1.png')
        plt.savefig(output_path, dpi=150)
        print(f"Saved: {output_path}")

    if show:
        plt.show()
    else:
        plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Two-point interpolation examples')
    parser.add_argument('--no-show', action='store_true', help='Do not display plots')
    parser.add_argument('--save', action='store_true', help='Save plots to images/ directory')
    args = parser.parse_args()

    show = not args.no_show
    save = args.save

    generate_case0_example(show=show, save=save)
    generate_case1_example(show=show, save=save)

    if save:
        print("\nPlots saved to images/ directory")
