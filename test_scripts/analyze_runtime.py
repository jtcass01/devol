import glob
import pandas as pd
import matplotlib.pyplot as plt

from utils import calc_95_ci


def load_runtime_data(base_dir='test_results'):
    records = []
    for csv_file in glob.glob(f'{base_dir}/particles_*/baseline/run_*/*.csv'):
        parts = csv_file.replace('\\', '/').split('/')
        particles = int(parts[-4].split('_')[1])

        try:
            df = pd.read_csv(csv_file)
            if df.empty:
                continue

            mean_runtimes = df[['EKF_Runtime', 'PF_Runtime', 'DR_Runtime']].mean()

            records.append(
                {
                    'Particles': particles,
                    'EKF_Runtime': mean_runtimes['EKF_Runtime'] * 1000,
                    'PF_Runtime': mean_runtimes['PF_Runtime'] * 1000,
                    'DR_Runtime': mean_runtimes['DR_Runtime'] * 1000,
                }
            )
        except Exception:
            pass

    return pd.DataFrame(records)


def main():
    df = load_runtime_data()
    if df.empty:
        print('No data found.')
        return

    labels = ['EKF', 'DR']
    means = [df['EKF_Runtime'].mean(), df['DR_Runtime'].mean()]
    cis = [calc_95_ci(df['EKF_Runtime']), calc_95_ci(df['DR_Runtime'])]

    particle_counts = sorted(df['Particles'].unique())

    for p in particle_counts:
        labels.append(f'PF_{p}')
        df_p = df[df['Particles'] == p]
        means.append(df_p['PF_Runtime'].mean())
        cis.append(calc_95_ci(df_p['PF_Runtime']))

    print('=== Runtime Results (milliseconds per step) with 95% CI ===')
    for i, label in enumerate(labels):
        print(f'{label:8} : {means[i]:.3f} ± {cis[i]:.3f} ms')

    fig, ax = plt.subplots(figsize=(10, 6))

    colors = ['blue', 'gray'] + ['orange'] * len(particle_counts)

    ax.bar(labels, means, yerr=cis, color=colors, capsize=5, error_kw={'alpha': 0.7})

    ax.set_ylabel('Average Runtime (ms)')
    ax.set_title('Algorithm Runtime Comparison with 95% CI')
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.grid(axis='y', linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.savefig('runtime_comparison_ci.png')
    print('Graph saved to runtime_comparison_ci.png')


if __name__ == '__main__':
    main()
