# One-Axis IMU Tilt Estimation & Sensor Fusion (MPU-6500 / Arduino UNO)

An introductory first-year EEE embedded sensing and signal-processing project implementing one-axis roll tilt estimation on an **Arduino UNO R3** using a 6-axis **MPU-6500 MEMS IMU** over I²C.

This repository documents the completed estimator milestone: register-level I²C acquisition via Arduino `Wire`, six-position static accelerometer calibration, startup gyroscope zero-rate bias compensation, discrete-time **Complementary Filter** fusion, and Python time-series analysis. Closed-loop servo actuation is scoped as future work.

---

## 1. Hardware & Electrical Notes

* **Microcontroller:** Arduino UNO R3 (`ATmega328P`, 16 MHz)
* **Sensor Breakout:** Multi-model carrier board marked `MPU-9250/6500/9255`; device identity register `WHO_AM_I` (`0x75`) verified as `0x70` (MPU-6500) at I²C address `0x68`.
* **Interface & Configuration:** Burst register reads starting at `ACCEL_XOUT_H` (`0x3B`) using `<Wire.h>` with signed 16-bit two's complement decoding and explicit I²C read-status validation, without external IMU libraries. Firmware explicitly configures and verifies `GYRO_CONFIG` (`0x1B = 0x00`, $\pm 250^\circ/\text{s}$) and `ACCEL_CONFIG` (`0x1C = 0x00`, $\pm 2g$).

| Breakout Pin | Arduino UNO Pin | Function & Electrical Note |
| :--- | :--- | :--- |
| `VCC` | `5V` | Power input to breakout onboard voltage regulator |
| `GND` | `GND` | Common ground reference |
| `SDA` | `A4 (SDA)` | I²C Serial Data |
| `SCL` | `A5 (SCL)` | I²C Serial Clock |

*Electrical compatibility note:* While the breakout's onboard regulator accepts `5V` on `VCC`, the MPU-6500 IC itself is rated for 3.3V supply and logic levels. Direct connection to the Arduino UNO `SDA`/`SCL` pins functioned during these bench tests, but the specific breakout's pull-up resistor rail and actual bus high-voltage levels have not yet been measured with a multimeter/oscilloscope and should be verified before long-term use or replication.

---

## 2. Six-Position Accelerometer Calibration

Using `firmware/accelerometer_average/accelerometer_average.ino` ($N = 200$ valid samples averaged per static orientation), six orthogonal orientation means were recorded in `data/accelerometer-six-position.txt`. Per-axis offset (`bias`) and sensitivity (`scale`) for the $\pm 2g$ range ($16384\text{ counts}/g$ nominal) were estimated from opposite faces:

$$a_i = \frac{\text{raw}_i - \text{bias}_i}{\text{scale}_i} \quad [g]$$

| Axis | Estimated Bias (counts) | Empirical Scale Factor (counts/$g$) | Nominal $\pm 2g$ Scale |
| :--- | ---: | ---: | ---: |
| **X** | $+263.74$ | $16427.78$ | $16384$ |
| **Y** | $+276.66$ | $16405.48$ | $16384$ |
| **Z** | $-697.16$ | $16627.08$ | $16384$ |

### Reported Static Spot Checks (`accelerometer_angle.ino`)
Static roll around the sensor X-axis is estimated from gravity projections on the Y–Z plane as $\theta_{\text{acc}} = \text{atan2}(a_y, a_z) \cdot \frac{180}{\pi}$ (valid primarily for rotation about X under zero linear acceleration). During bench checks in Serial Monitor (`115200 baud`, human-readable output format), the following manual poses were observed (reported spot checks without an independent metrological angle fixture):
* **Flat on table ($\approx 0^\circ$):** $a_x = -0.02g, a_y = 0.00g, a_z = 1.03g \rightarrow \Vert{}a\Vert{} = 1.03g$, $\theta_{\text{acc}} \in [-0.3^\circ, +0.4^\circ]$.
* **Manual $\approx -45^\circ$ tilt:** $a_x = 0.00g, a_y = -0.72g, a_z = 0.72g \rightarrow \Vert{}a\Vert{} = 1.02g$, $\theta_{\text{acc}} = -45.0^\circ$.
* **Resting on side header pins ($\approx -90^\circ$):** $a_x = -0.01g, a_y = -1.00g, a_z = 0.10g \rightarrow \Vert{}a\Vert{} = 1.00g$, $\theta_{\text{acc}} = -84.3^\circ$.
* **Vigorous hand shaking:** Individual axes reached $\approx 1.98g$ (near the $\pm 2g$ per-axis ceiling, $\Vert{}a\Vert{} = 3.40g$), producing transient $\theta_{\text{acc}}$ values up to $134.2^\circ$ due to linear acceleration.

---

## 3. One-Axis Complementary Sensor Fusion & Telemetry Analysis

Implemented in `firmware/complementary_filter/complementary_filter.ino`, the sketch averages 500 valid stationary gyro-X samples at startup to estimate zero-rate offset, measures loop intervals $\Delta t$ via `micros()`, and updates a scalar complementary filter ($\alpha = 0.98$, time constant $\tau \approx 1.17\text{ s}$ at the mean software polling rate of $41.87\text{ Hz}$, $\Delta t_{\text{mean}} = 23.88\text{ ms}$):

$$\theta_{\text{fused}}[k] = \alpha \left(\theta_{\text{fused}}[k-1] + \omega_x[k]\Delta t\right) + (1 - \alpha)\theta_{\text{acc}}[k]$$

![Complementary Filter Comparison](results/plots/complementary_filter_comparison.png)

### Measured Interval Statistics (`data/complementary_filter_log.csv`, 1,560 samples / 37.234 s)

| Interval / Event | Samples ($n$) | Accelerometer Tilt (`acc_roll_deg`) | Integrated Gyro X (`gyro_roll_deg`) | Complementary Filter (`fused_roll_deg`) | Observed Behaviour |
| :--- | ---: | :--- | :--- | :--- | :--- |
| **Interval 1: Stationary (`0.0–5.5 s`)** | 232 | Mean: $+0.74^\circ$<br>Std: $0.142^\circ$ | Mean: $+0.55^\circ$<br>Std: $0.056^\circ$ | Mean: $+0.69^\circ$<br>Std: **$0.030^\circ$** | **$4.69\times$ lower output standard deviation** (scatter around mean) than accelerometer tilt. |
| **Interval 2: Tilted Hold (`10.5–18.0 s`)** | 312 | Mean: $-43.68^\circ$<br>Std: $0.560^\circ$ | Mean: $-42.34^\circ$<br>Std: $0.216^\circ$ | Mean: $-43.66^\circ$<br>Std: **$0.239^\circ$** | **$2.35\times$ lower standard deviation** ($5.50\times$ lower variance) than raw accelerometer tilt during manual hold. |
| **Impact Event (`t = 20.256 s`)** | 1 | **$-47.85^\circ$** | $+3.67^\circ$ | **$+0.19^\circ$** *(prev: $-2.04^\circ$)* | Attenuates the transient table-impact spike (`acc - fused = -48.04°`), though $2\%$ ($1 - \alpha$) of the corrupted accelerometer-derived tilt angle still enters the recurrence per sample. |
| **Interval 3: Dynamic Shaking (`22.0–37.234 s`)** | 639 | Mean: $-1.44^\circ$<br>Std: $2.278^\circ$ | Mean: $+1.34^\circ$<br>Std: $1.386^\circ$ | Mean: $-1.44^\circ$<br>Std: **$1.345^\circ$** | Reduces dynamic scatter; by $37.234\text{ s}$, `gyro_roll_deg` ($+2.24^\circ$) and `fused_roll_deg` ($-0.74^\circ$) diverge by **$2.98^\circ$** as the filter tracks the long-term gravity reference. |

*Limitations:* Without an external optical/mechanical angle reference, these metrics quantify relative estimator scatter and divergence rather than absolute angular accuracy. The scalar update assumes rotation predominantly about the X-axis and does not handle $\pm 180^\circ$ `atan2` wrap-around or 3D Euler coupling.

---

## 4. Quick Start & Reproduction

1. **Firmware Upload (Arduino IDE):** Select `Arduino Uno` (`ATmega328P`), connect the MPU-6500 as above, and upload:
   * `firmware/accelerometer_average/accelerometer_average.ino` (`9600 baud`) for 200-sample static calibration averages.
   * `firmware/accelerometer_angle/accelerometer_angle.ino` (`115200 baud`) for human-readable calibrated $g$-units, vector norm, and static roll.
   * `firmware/complementary_filter/complementary_filter.ino` (`115200 baud`) for fused CSV output. Keep the board stationary for the first ~2 seconds after reset during 500-sample gyro bias calibration.
2. **Capturing a New CSV Log:** Open Serial Monitor at `115200 baud` (disable timestamps in the IDE), press `RESET` on the UNO, perform the test sequence, unplug USB to stop streaming, and copy-paste the output starting from the header line `time_s,acc_roll_deg,gyro_roll_deg,fused_roll_deg` into `data/complementary_filter_log.csv`.
3. **Plot & Statistics Reproduction (Python 3):** Run from the repository root (`D:\IMU`):
   ```bash
   pip install pandas matplotlib
   python scripts/plot_results.py
   ```

---

## 5. Next Steps (Milestone D)

* Design a mechanically constrained 1-DOF tiltable test rig with an SG90 servo actuator and separate power supply to close an external P/PD attitude stabilisation loop.

---

## 6. Provenance & Acknowledgements

Developed by Andrey Maltsev (Year 1 BEng EEE, University of Manchester) as a hands-on learning project. Hardware wiring, sensor probing (`WHO_AM_I = 0x70`), 6-position data collection, bench experiments, and Serial telemetry capture were performed by the student; AI assistants (ChatGPT / Gemini) were used for roadmap structuring, code/plotting templates, and documentation review.