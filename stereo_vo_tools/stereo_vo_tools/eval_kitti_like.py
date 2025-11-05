#!/usr/bin/env python3
"""
Evaluate visual odometry using TUM RGB-D metrics (ATE, RPE)
"""

import argparse
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt


def load_trajectory_tum(filename):
    """Load trajectory in TUM format: timestamp x y z qx qy qz qw"""
    data = np.loadtxt(filename)
    if data.shape[1] < 8:
        raise ValueError("TUM format requires at least 8 columns")
    return data


def align_trajectories(gt, est):
    """Align estimated trajectory to ground truth using timestamps"""
    # Simple nearest-neighbor matching
    aligned_est = []
    aligned_gt = []
    
    for gt_row in gt:
        gt_time = gt_row[0]
        # Find closest estimated timestamp
        time_diffs = np.abs(est[:, 0] - gt_time)
        closest_idx = np.argmin(time_diffs)
        
        if time_diffs[closest_idx] < 0.1:  # 100ms threshold
            aligned_gt.append(gt_row)
            aligned_est.append(est[closest_idx])
    
    return np.array(aligned_gt), np.array(aligned_est)


def compute_ate(gt, est):
    """Compute Absolute Trajectory Error (ATE)"""
    # Extract positions
    gt_pos = gt[:, 1:4]
    est_pos = est[:, 1:4]
    
    # Compute errors
    errors = np.linalg.norm(gt_pos - est_pos, axis=1)
    
    ate_rmse = np.sqrt(np.mean(errors ** 2))
    ate_mean = np.mean(errors)
    ate_median = np.median(errors)
    ate_std = np.std(errors)
    ate_min = np.min(errors)
    ate_max = np.max(errors)
    
    return {
        'rmse': ate_rmse,
        'mean': ate_mean,
        'median': ate_median,
        'std': ate_std,
        'min': ate_min,
        'max': ate_max,
        'errors': errors
    }


def compute_rpe(gt, est, delta=1.0):
    """Compute Relative Pose Error (RPE)"""
    # Extract positions
    gt_pos = gt[:, 1:4]
    est_pos = est[:, 1:4]
    
    # Compute relative poses
    trans_errors = []
    
    for i in range(len(gt_pos) - 1):
        dt = gt[i + 1, 0] - gt[i, 0]
        if abs(dt - delta) > 0.1:
            continue
        
        # Ground truth relative motion
        gt_rel = gt_pos[i + 1] - gt_pos[i]
        gt_dist = np.linalg.norm(gt_rel)
        
        # Estimated relative motion
        est_rel = est_pos[i + 1] - est_pos[i]
        est_dist = np.linalg.norm(est_rel)
        
        # Translation error
        trans_error = abs(est_dist - gt_dist)
        trans_errors.append(trans_error)
    
    trans_errors = np.array(trans_errors)
    
    rpe_rmse = np.sqrt(np.mean(trans_errors ** 2))
    rpe_mean = np.mean(trans_errors)
    rpe_median = np.median(trans_errors)
    rpe_std = np.std(trans_errors)
    
    return {
        'rmse': rpe_rmse,
        'mean': rpe_mean,
        'median': rpe_median,
        'std': rpe_std,
        'errors': trans_errors
    }


def plot_trajectory(gt, est, output_file):
    """Plot 2D trajectory"""
    plt.figure(figsize=(10, 10))
    
    plt.plot(gt[:, 1], gt[:, 2], 'b-', label='Ground Truth', linewidth=2)
    plt.plot(est[:, 1], est[:, 2], 'r--', label='Estimated', linewidth=2)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'Saved trajectory plot to {output_file}')


def plot_errors(ate_results, rpe_results, output_dir):
    """Plot error distributions"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # ATE errors
    axes[0].plot(ate_results['errors'], 'b-', linewidth=1)
    axes[0].axhline(y=ate_results['mean'], color='r', linestyle='--', 
                    label=f'Mean: {ate_results["mean"]:.3f} m')
    axes[0].set_xlabel('Frame')
    axes[0].set_ylabel('ATE (m)')
    axes[0].set_title('Absolute Trajectory Error')
    axes[0].legend()
    axes[0].grid(True)
    
    # RPE errors
    axes[1].plot(rpe_results['errors'], 'g-', linewidth=1)
    axes[1].axhline(y=rpe_results['mean'], color='r', linestyle='--',
                    label=f'Mean: {rpe_results["mean"]:.3f} m')
    axes[1].set_xlabel('Frame')
    axes[1].set_ylabel('RPE (m)')
    axes[1].set_title('Relative Pose Error')
    axes[1].legend()
    axes[1].grid(True)
    
    plt.tight_layout()
    output_file = output_dir / 'errors.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f'Saved error plots to {output_file}')


def save_results(ate_results, rpe_results, output_file):
    """Save results to CSV"""
    with open(output_file, 'w') as f:
        f.write('Metric,RMSE,Mean,Median,Std,Min,Max\n')
        f.write(f'ATE,{ate_results["rmse"]:.4f},{ate_results["mean"]:.4f},'
                f'{ate_results["median"]:.4f},{ate_results["std"]:.4f},'
                f'{ate_results["min"]:.4f},{ate_results["max"]:.4f}\n')
        f.write(f'RPE,{rpe_results["rmse"]:.4f},{rpe_results["mean"]:.4f},'
                f'{rpe_results["median"]:.4f},{rpe_results["std"]:.4f},,\n')
    
    print(f'Saved results to {output_file}')


def main():
    parser = argparse.ArgumentParser(description='Evaluate visual odometry')
    parser.add_argument('--ground-truth', required=True, help='Ground truth trajectory (TUM format)')
    parser.add_argument('--estimated', required=True, help='Estimated trajectory (TUM format)')
    parser.add_argument('--output', default='artifacts', help='Output directory')
    parser.add_argument('--delta', type=float, default=1.0, help='Delta for RPE computation')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Load trajectories
    print(f'Loading ground truth: {args.ground_truth}')
    gt = load_trajectory_tum(args.ground_truth)
    
    print(f'Loading estimated trajectory: {args.estimated}')
    est = load_trajectory_tum(args.estimated)
    
    # Align trajectories
    print('Aligning trajectories...')
    gt_aligned, est_aligned = align_trajectories(gt, est)
    print(f'Aligned {len(gt_aligned)} poses')
    
    if len(gt_aligned) < 2:
        print('Error: Not enough aligned poses')
        return
    
    # Compute metrics
    print('Computing ATE...')
    ate_results = compute_ate(gt_aligned, est_aligned)
    
    print('Computing RPE...')
    rpe_results = compute_rpe(gt_aligned, est_aligned, args.delta)
    
    # Print results
    print('\n' + '='*60)
    print('EVALUATION RESULTS')
    print('='*60)
    print(f'Number of poses: {len(gt_aligned)}')
    print(f'\nAbsolute Trajectory Error (ATE):')
    print(f'  RMSE:   {ate_results["rmse"]:.4f} m')
    print(f'  Mean:   {ate_results["mean"]:.4f} m')
    print(f'  Median: {ate_results["median"]:.4f} m')
    print(f'  Std:    {ate_results["std"]:.4f} m')
    print(f'  Min:    {ate_results["min"]:.4f} m')
    print(f'  Max:    {ate_results["max"]:.4f} m')
    
    print(f'\nRelative Pose Error (RPE):')
    print(f'  RMSE:   {rpe_results["rmse"]:.4f} m')
    print(f'  Mean:   {rpe_results["mean"]:.4f} m')
    print(f'  Median: {rpe_results["median"]:.4f} m')
    print(f'  Std:    {rpe_results["std"]:.4f} m')
    print('='*60)
    
    # Plot and save
    plot_trajectory(gt_aligned, est_aligned, output_dir / 'trajectory.png')
    plot_errors(ate_results, rpe_results, output_dir)
    save_results(ate_results, rpe_results, output_dir / 'results.csv')
    
    print(f'\nAll outputs saved to: {output_dir}')


if __name__ == '__main__':
    main()
