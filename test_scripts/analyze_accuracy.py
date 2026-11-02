import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from utils import calc_95_ci

def load_accuracy_data(base_dir="test_results"):
    records = []
    for csv_file in glob.glob(f"{base_dir}/particles_*/baseline/run_*/*.csv"):
        parts = csv_file.replace('\\', '/').split('/')
        particles = int(parts[-4].split('_')[1])
        
        try:
            df = pd.read_csv(csv_file)
            if df.empty: continue
            final_row = df.iloc[-1]
            records.append({
                'Particles': particles,
                'EKF_Pos_RMSE': final_row['EKF_Pos_RMSE'],
                'EKF_Yaw_RMSE': final_row['EKF_Yaw_RMSE'],
                'PF_Pos_RMSE': final_row['PF_Pos_RMSE'],
                'PF_Yaw_RMSE': final_row['PF_Yaw_RMSE'],
                'DR_Pos_RMSE': final_row['DR_Pos_RMSE'],
                'DR_Yaw_RMSE': final_row['DR_Yaw_RMSE']
            })
        except Exception as e:
            print(f"Skipping {csv_file}: {e}")
            
    return pd.DataFrame(records)

def main():
    df = load_accuracy_data()
    if df.empty:
        print("No data found. Check your test_results directory.")
        return

    metrics = {}
    
    # EKF and DR (aggregate all runs)
    metrics['EKF'] = {
        'pos_mean': df['EKF_Pos_RMSE'].mean(),
        'pos_ci': calc_95_ci(df['EKF_Pos_RMSE']),
        'yaw_mean': df['EKF_Yaw_RMSE'].mean(),
        'yaw_ci': calc_95_ci(df['EKF_Yaw_RMSE'])
    }
    
    metrics['DR'] = {
        'pos_mean': df['DR_Pos_RMSE'].mean(),
        'pos_ci': calc_95_ci(df['DR_Pos_RMSE']),
        'yaw_mean': df['DR_Yaw_RMSE'].mean(),
        'yaw_ci': calc_95_ci(df['DR_Yaw_RMSE'])
    }

    # PF variants
    particle_counts = sorted(df['Particles'].unique())
    for p in particle_counts:
        df_p = df[df['Particles'] == p]
        metrics[f'PF_{p}'] = {
            'pos_mean': df_p['PF_Pos_RMSE'].mean(),
            'pos_ci': calc_95_ci(df_p['PF_Pos_RMSE']),
            'yaw_mean': df_p['PF_Yaw_RMSE'].mean(),
            'yaw_ci': calc_95_ci(df_p['PF_Yaw_RMSE'])
        }

    print("=== Accuracy Results (Baseline) with 95% CI ===")
    for algo, data in metrics.items():
        print(f"{algo:10} | Pos RMSE: {data['pos_mean']:.4f} ± {data['pos_ci']:.4f} m | "
              f"Yaw RMSE: {data['yaw_mean']:.4f} ± {data['yaw_ci']:.4f} rad")

    labels = list(metrics.keys())
    pos_means = [m['pos_mean'] for m in metrics.values()]
    pos_cis = [m['pos_ci'] for m in metrics.values()]
    yaw_means = [m['yaw_mean'] for m in metrics.values()]
    yaw_cis = [m['yaw_ci'] for m in metrics.values()]

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Add yerr parameter to plot the confidence intervals as error bars
    ax.bar(x - width/2, pos_means, width, yerr=pos_cis, label='Position RMSE (m)', 
           color='skyblue', capsize=5, error_kw={'alpha':0.7})
    ax.bar(x + width/2, yaw_means, width, yerr=yaw_cis, label='Yaw RMSE (rad)', 
           color='lightcoral', capsize=5, error_kw={'alpha':0.7})

    ax.set_ylabel('RMSE (Lower is Better)')
    ax.set_title('Algorithm Accuracy Comparison with 95% CI (Baseline)')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.legend()
    ax.grid(axis='y', linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.savefig('accuracy_comparison_ci.png')
    print("Graph saved to accuracy_comparison_ci.png")

if __name__ == "__main__":
    main()