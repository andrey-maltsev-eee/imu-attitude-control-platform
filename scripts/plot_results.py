```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('data/complementary_filter_log.csv')
df['acc_minus_fused'] = df['acc_roll_deg'] - df['fused_roll_deg']
df['gyro_minus_fused'] = df['gyro_roll_deg'] - df['fused_roll_deg']

# Print exact interval metrics
intervals = {
    'Interval 1: Stationary (0.0-5.5 s)': (df['time_s'] >= 0.0) & (df['time_s'] <= 5.5),
    'Interval 2: Tilted Hold (10.5-18.0 s)': (df['time_s'] >= 10.5) & (df['time_s'] <= 18.0),
    'Interval 3: Dynamic Shaking (22.0-37.234 s)': (df['time_s'] >= 22.0) & (df['time_s'] <= 37.234),
}

for name, mask in intervals.items():
    sub = df.loc[mask, ['acc_roll_deg', 'gyro_roll_deg', 'fused_roll_deg']]
    print(f"\n=== {name} (n={len(sub)}) ===")
    print(sub.agg(['mean', 'median', 'min', 'max', 'std']).round(3))

plt.rcParams['figure.dpi'] = 150
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11.5, 7.8), sharex=True, gridspec_kw={'height_ratios': [2.1, 1.3]})

ax1.plot(df['time_s'], df['acc_roll_deg'], color='#e74c3c', alpha=0.45, linewidth=1.0, label='Accelerometer Tilt (acc_roll_deg)')
ax1.plot(df['time_s'], df['gyro_roll_deg'], color='#2980b9', linestyle='--', linewidth=1.6, label='Integrated Gyroscope Roll (gyro_roll_deg)')
ax1.plot(df['time_s'], df['fused_roll_deg'], color='#27ae60', linewidth=2.1, label='Complementary Filter (fused_roll_deg, α = 0.98)')

ax1.axvspan(0, 5.5, color='#ecf0f1', alpha=0.65, label='Interval 1: Stationary (0.0–5.5 s)')
ax1.axvspan(10.5, 18.0, color='#fef9e7', alpha=0.75, label='Interval 2: Tilted Hold (10.5–18.0 s)')
ax1.axvline(20.256, color='#8e44ad', linestyle=':', linewidth=1.4, label='Table Impact Event (t = 20.256 s)')
ax1.axvspan(22.0, 37.234, color='#fdedec', alpha=0.45, label='Interval 3: Dynamic Shaking (22.0–37.234 s)')

ax1.set_title('MPU-6500 One-Axis Roll Estimation: Complementary Filter vs. Single-Sensor Estimates', fontsize=12.5, fontweight='bold', pad=10)
ax1.set_ylabel('Estimated Roll Angle (deg)', fontsize=11)
ax1.set_ylim(-62, 12)
ax1.grid(True, linestyle=':', alpha=0.7)
ax1.legend(loc='lower right', fontsize=8.4, ncol=2, framealpha=0.95)

ax2.plot(df['time_s'], df['acc_minus_fused'], color='#e74c3c', alpha=0.55, linewidth=0.95, label='Accelerometer Minus Fused (acc_roll_deg - fused_roll_deg)')
ax2.plot(df['time_s'], df['gyro_minus_fused'], color='#2980b9', linewidth=1.8, label='Gyroscope Minus Fused (gyro_roll_deg - fused_roll_deg)')
ax2.axhline(0, color='black', linewidth=0.8, linestyle='-')
ax2.axvline(20.256, color='#8e44ad', linestyle=':', linewidth=1.2)
ax2.annotate('Impact spike: -48.04°\n(t = 20.256 s)', xy=(20.256, -48.04), xytext=(22.5, -43.0),
             arrowprops=dict(arrowstyle='->', color='#c0392b', lw=1.0), fontsize=8.5, color='#c0392b', fontweight='bold')

ax2.set_title('Estimator Differences Relative to Fused Output (Full Unclipped Range)', fontsize=11, fontweight='bold', pad=8)
ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('Difference (deg)', fontsize=11)
ax2.set_ylim(-53, 18)
ax2.grid(True, linestyle=':', alpha=0.7)
ax2.legend(loc='lower left', fontsize=8.8, framealpha=0.94)

plt.tight_layout()
plt.savefig('results/plots/complementary_filter_comparison.png', bbox_inches='tight')
print("Saved results/plots/complementary_filter_comparison.png")