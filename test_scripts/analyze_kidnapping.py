import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats as st


def calc_95_ci(data):
    """Calculates the 95% CI margin of error for an array of data."""
    n = len(data)
    if n < 2:
        return 0.0
    sem = st.sem(data)
    margin = sem * st.t.ppf((1 + 0.95) / 2.0, n - 1)
    return margin


def load_kidnapping_data(base_dir='test_results'):
    records = []
    for csv_file in glob.glob(f'{base_dir}/particles_*/*/run_*/*.csv'):
        parts = csv_file.replace('\\', '/').split('/')
        particles = int(parts[-4].split('_')[1])
        condition = parts[-3]

        try:
            df = pd.read_csv(csv_file)
            if df.empty:
                continue
            final_row = df.iloc[-1]
            records.append(
                {
                    'Particles': particles,
                    'Condition': condition,
                    'EKF_Pos_RMSE': final_row['EKF_Pos_RMSE'],
                    'PF_Pos_RMSE': final_row['PF_Pos_RMSE'],
                    'DR_Pos_RMSE': final_row['DR_Pos_RMSE'],
                }
            )
        except Exception as e:
            pass

    return pd.DataFrame(records)


def main():
    df = load_kidnapping_data()
    if df.empty:
        print('No data found.')
        return

    df_base = df[df['Condition'] == 'baseline']
    df_kid = df[df['Condition'] == 'kidnapped']

    labels = ['EKF', 'DR']

    # EKF and DR are aggregated across all runs for stability
    base_means = [df_base['EKF_Pos_RMSE'].mean(), df_base['DR_Pos_RMSE'].mean()]
    base_cis = [calc_95_ci(df_base['EKF_Pos_RMSE']), calc_95_ci(df_base['DR_Pos_RMSE'])]

    kid_means = [df_kid['EKF_Pos_RMSE'].mean(), df_kid['DR_Pos_RMSE'].mean()]
    kid_cis = [calc_95_ci(df_kid['EKF_Pos_RMSE']), calc_95_ci(df_kid['DR_Pos_RMSE'])]

    # Process each PF variant dynamically
    particle_counts = sorted(df['Particles'].unique())
    for p in particle_counts:
        labels.append(f'PF_{p}')
        df_base_p = df_base[df_base['Particles'] == p]
        df_kid_p = df_kid[df_kid['Particles'] == p]

        base_means.append(df_base_p['PF_Pos_RMSE'].mean())
        base_cis.append(calc_95_ci(df_base_p['PF_Pos_RMSE']))

        kid_means.append(df_kid_p['PF_Pos_RMSE'].mean())
        kid_cis.append(calc_95_ci(df_kid_p['PF_Pos_RMSE']))

    print('=== Robustness to Kidnapping (Position RMSE) ===')
    for i, label in enumerate(labels):
        print(
            f'{label:8} | Baseline: {base_means[i]:.4f} ± {base_cis[i]:.4f} m | '
            f'Kidnapped: {kid_means[i]:.4f} ± {kid_cis[i]:.4f} m'
        )

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.bar(
        x - width / 2,
        base_means,
        width,
        yerr=base_cis,
        label='Baseline',
        color='lightgreen',
        capsize=5,
        error_kw={'alpha': 0.7},
    )
    ax.bar(
        x + width / 2,
        kid_means,
        width,
        yerr=kid_cis,
        label='Kidnapped',
        color='salmon',
        capsize=5,
        error_kw={'alpha': 0.7},
    )

    ax.set_ylabel('Position RMSE (m)')
    ax.set_title('Robustness to Kidnapping with 95% CI (Baseline vs. Kidnapped)')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.legend()
    ax.grid(axis='y', linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.savefig('kidnapping_robustness_ci.png')
    print('Graph saved to kidnapping_robustness_ci.png')


if __name__ == '__main__':
    main()
