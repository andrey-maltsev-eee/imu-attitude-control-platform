import pandas as pd
import matplotlib.pyplot as plt

# Load recorded CSV telemetry
df = pd.read_csv('data/complementary_filter_log.csv')
df['gyro_drift'] = df['gyro_roll_deg'] - df['fused_roll_deg']
df['acc_noise'] = df['acc_roll_deg'] - df['fused_roll_deg']

plt.rcParams['figure.dpi'] = 150
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11.5, 7.5), sharex=True, gridspec_kw={'height_ratios': [2.2, 1.2]})

# Top plot: Sensor Fusion comparison
ax1.plot(df['time_s'], df['acc_roll_deg'], color='#e74c3c', alpha=0.45, linewidth=1.0, label='Accelerometer Only (acc_roll_deg) — Sensitive to Linear Acceleration')
ax1.plot(df['time_s'], df['gyro_roll_deg'], color='#2980b9', linestyle='--', linewidth=1.7, label='Gyroscope Integration Only (gyro_roll_deg) — Accumulates Drift (+3.06°)')
ax1.plot(df['time_s'], df['fused_roll_deg'], color='#27ae60', linewidth=2.2, label='Complementary Filter (fused_roll_deg, α = 0.98)')

ax1.axvspan(0, 5.5, color='#ecf0f1', alpha=0.65, label='Phase 1: Stationary 0° Baseline (0–5.5 s)')
ax1.axvspan(10.5, 18.0, color='#fef9e7', alpha=0.75, label='Phase 2: ~45° Tilted Hold (10.5–18.0 s)')
ax1.axvspan(22.0, 37.234, color='#fdedec', alpha=0.45, label='Phase 3: Dynamic Shaking / Disturbances (22–37.2 s)')

ax1.set_title('MPU-6500 One-Axis Attitude Estimation: Complementary Filter vs. Raw Sensors', fontsize=13, fontweight='bold', pad=10)
ax1.set_ylabel('Roll Angle (deg)', fontsize=11)
ax1.set_ylim(-63, 15)
ax1.grid(True, linestyle=':', alpha=0.7)
ax1.legend(loc='lower left', fontsize=8.5, ncol=2, framealpha=0.92)

# Bottom plot: Isolated Sensor Errors
ax2.plot(df['time_s'], df['acc_noise'], color='#e74c3c', alpha=0.5, linewidth=0.9, label='Accelerometer Dynamic Spike Error (acc - fused)')
ax2.plot(df['time_s'], df['gyro_drift'], color='#2980b9', linewidth=1.9, label='Cumulative Gyroscope Drift Error (gyro - fused)')
ax2.axhline(0, color='black', linewidth=0.8, linestyle='-')

ax2.set_title('Isolated Sensor Errors: High-Frequency Accelerometer Spikes vs. Low-Frequency Gyro Drift', fontsize=11, fontweight='bold', pad=8)
ax2.set_xlabel('Time (s)', fontsize=11)
ax2.set_ylabel('Angle Deviation (deg)', fontsize=11)
ax2.set_ylim(-15, 15)
ax2.grid(True, linestyle=':', alpha=0.7)
ax2.legend(loc='upper left', fontsize=9, framealpha=0.92)

plt.tight_layout()
plt.savefig('results/plots/complementary_filter_comparison.png', bbox_inches='tight')
print("Plot saved to results/plots/complementary_filter_comparison.png")