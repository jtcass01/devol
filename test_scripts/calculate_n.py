#!/usr/bin/env python3

import argparse
import glob
import math
import os
import sys

import numpy as np
import pandas as pd
import scipy.stats as stats


def calculate_run_rmse(csv_path):
    """Calculates the Position RMSE for EKF, PF, and DR from a single CSV run."""
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return None

    metrics = {}
    algos = ['EKF', 'PF', 'DR']

    for algo in algos:
        # Verify columns exist
        col_x, col_y = f'{algo}_X', f'{algo}_Y'
        if col_x not in df.columns or col_y not in df.columns:
            print(f"Warning: Missing columns for {algo} in {csv_path}")
            continue

        # Calculate position error at each time step
        dx = df['GT_X'] - df[col_x]
        dy = df['GT_Y'] - df[col_y]
        
        # Calculate Root Mean Square Error (RMSE) for the run
        squared_error = dx**2 + dy**2
        rmse = np.sqrt(squared_error.mean())
        metrics[algo] = rmse

    return metrics


def main():
    parser = argparse.ArgumentParser(description="Calculate required number of test cases (N) from a pilot study.")
    parser.add_argument('--pilot_dir', type=str, required=True,
                        help="Directory containing the pilot study CSV files.")
    parser.add_argument('--margin_of_error', type=float, default=0.05,
                        help="Acceptable margin of error (E) in meters. Default is 0.05 m.")
    parser.add_argument('--confidence_level', type=float, default=0.95,
                        help="Confidence level (e.g., 0.95 for 95% CI). Default is 0.95.")
    
    args = parser.parse_args()

    # Find all CSV files in the specified directory
    search_pattern = os.path.join(args.pilot_dir, '*.csv')
    csv_files = glob.glob(search_pattern)

    if not csv_files:
        print(f"No CSV files found in directory: {args.pilot_dir}")
        sys.exit(1)

    print(f"Found {len(csv_files)} pilot runs. Processing data...\n")

    # Collect RMSE data for all runs
    # Format: {'EKF': [rmse1, rmse2, ...], 'PF': [...], 'DR': [...]}
    algo_rmse_lists = {'EKF': [], 'PF': [], 'DR': []}

    for file in csv_files:
        run_metrics = calculate_run_rmse(file)
        if run_metrics:
            for algo, rmse in run_metrics.items():
                algo_rmse_lists[algo].append(rmse)

    # Calculate Z-score for the given confidence level
    # For 95% CI (0.95), alpha is 0.05, so we look up 0.975
    alpha = 1.0 - args.confidence_level
    z_score = stats.norm.ppf(1.0 - (alpha / 2.0))

    required_n = {}
    
    print(f"--- Pilot Study Statistics ---")
    for algo, rmse_list in algo_rmse_lists.items():
        if len(rmse_list) < 2:
            print(f"{algo}: Not enough data to calculate variance.")
            continue

        # Calculate mean and standard deviation of the RMSEs
        mean_rmse = np.mean(rmse_list)
        std_dev = np.std(rmse_list, ddof=1)  # ddof=1 for sample standard deviation

        # Formula: N = (Z * sigma / E)^2
        n_raw = (z_score * std_dev / args.margin_of_error)**2
        n_rounded = math.ceil(n_raw)
        required_n[algo] = n_rounded

        print(f"{algo}:")
        print(f"  Mean RMSE: {mean_rmse:.4f} m")
        print(f"  Std Dev:   {std_dev:.4f} m")
        print(f"  Calc N:    {n_raw:.2f} -> {n_rounded} runs")

    print("\n--- Conclusion ---")
    if required_n:
        max_algo = max(required_n, key=required_n.get)
        max_n = required_n[max_algo]
        print(f"To satisfy a {(args.confidence_level * 100):.0f}% Confidence Interval with a margin of error of {args.margin_of_error} m:")
        print(f"The driving algorithm is {max_algo}, requiring {max_n} test cases per configuration.")
        print(f"Recommendation: Run {max_n} test cases for your formal campaign.")
    else:
        print("Could not calculate N due to lack of valid data.")


if __name__ == '__main__':
    main()