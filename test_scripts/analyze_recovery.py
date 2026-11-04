import glob
import pandas as pd
import matplotlib.pyplot as plt
from utils import calc_95_ci

KIDNAP_THRESH = 1.0
RECOVER_THRESH = 0.3


def get_recovery_time(df, err_col, time_col='Time(s)'):
    high_err = df[df[err_col] > KIDNAP_THRESH]
    if high_err.empty:
        return None

    start_idx = high_err.index[0]
    start_time = df.loc[start_idx, time_col]
    post_kidnap = df.loc[start_idx:]

    recovered = post_kidnap[post_kidnap[err_col] < RECOVER_THRESH]

    if recovered.empty:
        return df[time_col].iloc[-1] - start_time

    return recovered.iloc[0][time_col] - start_time


def load_recovery_data(base_dir='test_results'):
    records = []
    for csv_file in glob.glob(f'{base_dir}/particles_*/kidnapped/run_*/*.csv'):
        parts = csv_file.replace('\\', '/').split('/')
        particles = int(parts[-4].split('_')[1])

        try:
            df = pd.read_csv(csv_file)
            if df.empty:
                continue

            records.append(
                {
                    'Particles': particles,
                    'EKF_Recovery': get_recovery_time(df, 'EKF_Pos_Err'),
                    'PF_Recovery': get_recovery_time(df, 'PF_Pos_Err'),
                    'DR_Recovery': get_recovery_time(df, 'DR_Pos_Err'),
                }
            )
        except Exception as e:
            pass

    return pd.DataFrame(records)


def main():
    df = load_recovery_data()
    if df.empty:
        print('No kidnapped data found to analyze.')
        return

    labels = ['EKF', 'DR']
    means = [df['EKF_Recovery'].mean(), df['DR_Recovery'].mean()]
    cis = [calc_95_ci(df['EKF_Recovery']), calc_95_ci(df['DR_Recovery'])]

    particle_counts = sorted(df['Particles'].unique())
    for p in particle_counts:
        labels.append(f'PF_{p}')
        df_p = df[df['Particles'] == p]
        means.append(df_p['PF_Recovery'].mean())
        cis.append(calc_95_ci(df_p['PF_Recovery']))

    print(f'=== Recovery Time Results with 95% CI ===')
    for i, label in enumerate(labels):
        print(f'{label:8} : {means[i]:.2f} ± {cis[i]:.2f} s')

    fig, ax = plt.subplots(figsize=(10, 6))

    colors = ['blue', 'gray'] + ['orange'] * len(particle_counts)

    bars = ax.bar(labels, means, yerr=cis, color=colors, capsize=5, error_kw={'alpha': 0.7})

    ax.set_ylabel('Average Time to Recover (s)')
    ax.set_title('Algorithm Recovery Time from Kidnapping with 95% CI')
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.grid(axis='y', linestyle=':', alpha=0.7)

    note = (
        'Note: Algorithms that fail to recover are capped at the remaining duration of the test.'
    )
    plt.figtext(
        0.5, 0.01, note, wrap=True, horizontalalignment='center', fontsize=9, style='italic'
    )

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.18)

    plt.savefig('kidnapping_recovery_time_ci.png')
    print('Graph saved to kidnapping_recovery_time_ci.png')


if __name__ == '__main__':
    main()
