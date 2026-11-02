import glob
import pandas as pd
import matplotlib.pyplot as plt
from utils import calc_95_ci


def load_pf_data(base_dir='test_results'):
    records = []
    # Use baseline to get clean accuracy metrics
    for csv_file in glob.glob(f'{base_dir}/particles_*/baseline/run_*/*.csv'):
        parts = csv_file.replace('\\', '/').split('/')
        particles = int(parts[-4].split('_')[1])

        try:
            df = pd.read_csv(csv_file)
            if df.empty:
                continue
            final_row = df.iloc[-1]
            avg_runtime = df['PF_Runtime'].mean()
            records.append(
                {
                    'Particles': particles,
                    'PF_Pos_RMSE': final_row['PF_Pos_RMSE'],
                    'PF_Runtime': avg_runtime,
                }
            )
        except Exception as e:
            pass

    return pd.DataFrame(records)


def main():
    df = load_pf_data()
    if df.empty:
        print('No data found.')
        return

    # Group by particle count and calculate the mean and 95% CI margin
    summary = df.groupby('Particles').agg(['mean', calc_95_ci]).reset_index()

    print('=== PF Particle Count Trade-offs (95% CI) ===')
    for _, row in summary.iterrows():
        p = int(row['Particles'][''])
        rmse = row['PF_Pos_RMSE']['mean']
        rmse_ci = row['PF_Pos_RMSE']['calc_95_ci']
        time = row['PF_Runtime']['mean']
        time_ci = row['PF_Runtime']['calc_95_ci']
        print(
            f'Particles: {p:4} | Pos RMSE: {rmse:.4f} ± {rmse_ci:.4f} m | Runtime: {time:.6f} ± {time_ci:.6f} s'
        )

    # Extract data for plotting
    particles = summary['Particles']['']
    rmse_mean = summary['PF_Pos_RMSE']['mean']
    rmse_ci_vals = summary['PF_Pos_RMSE']['calc_95_ci']
    time_mean = summary['PF_Runtime']['mean']
    time_ci_vals = summary['PF_Runtime']['calc_95_ci']

    # Create a single figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Plot Accuracy
    ax1.errorbar(particles, rmse_mean, yerr=rmse_ci_vals, fmt='-o', color='blue', capsize=5)
    ax1.set_xlabel('Number of Particles')
    ax1.set_ylabel('Position RMSE (m) [Lower is Better]')
    ax1.set_title('PF Accuracy vs. Particle Count (95% CI)')
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.set_xscale('log')
    ax1.set_xticks(particles)
    ax1.set_xticklabels(particles)

    # Plot Runtime
    ax2.errorbar(particles, time_mean, yerr=time_ci_vals, fmt='-o', color='red', capsize=5)
    ax2.set_xlabel('Number of Particles')
    ax2.set_ylabel('Average Runtime (s)')
    ax2.set_title('PF Runtime vs. Particle Count (95% CI)')
    ax2.grid(True, linestyle=':', alpha=0.7)
    ax2.set_xscale('log')
    ax2.set_xticks(particles)
    ax2.set_xticklabels(particles)

    plt.tight_layout()
    plt.savefig('particle_count_effects_ci.png')
    print('Graph saved to particle_count_effects_ci.png')


if __name__ == '__main__':
    main()
