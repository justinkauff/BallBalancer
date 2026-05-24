# 2-DOF Ball Balancing System

Real-time closed-loop control system that balances a steel ball on a 2-axis tilting platform. Implements independent PID controllers for X and Y axes, reading analog position data from a 4-wire resistive touchscreen and commanding two servo motors in real time.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Uno (or compatible) |
| Position Sensor | 4-wire resistive touchscreen |
| Actuators | 2× hobby servo motors |
| Servo X pin | Digital pin 9 |
| Servo Y pin | Digital pin 11 |

### Touchscreen Wiring

| Touchscreen Pin | Arduino Pin |
|---|---|
| XP | A1 |
| XM | A0 |
| YP | A3 |
| YM | A2 |

---

## Dependencies

Only the Arduino built-in `Servo.h` library is required — no additional installs needed.

---

## How It Works

Each control loop iteration:
1. Reads XY ball position from the resistive touchscreen (averaged over 5 samples)
2. Computes error relative to the plate center setpoint (97mm, 74mm)
3. Runs independent PID calculations for X and Y axes
4. Commands both servos via `writeMicroseconds()` to tilt the plate and correct the error

A 2mm deadband near the setpoint suppresses derivative kick, and a low-pass filter (`alpha = 0.9`) smooths the derivative term to reduce servo jitter.

---

## PID Parameters

```cpp
float kp = 2.0;
float ki = 0.167;
float kd = 1.375;
float alpha = 0.9;   // derivative low-pass filter coefficient
```

These were tuned for the specific hardware used in this build. If you're replicating the system, expect to re-tune — start with `ki = 0` and `kd = 0`, dial in `kp` until the ball oscillates stably, then add `kd` to damp the oscillation, and finally add a small `ki` to eliminate steady-state offset.

---

## Calibration

Before running, verify or adjust these constants at the top of the sketch to match your hardware:

| Constant | Description |
|---|---|
| `X_MIN`, `X_MAX` | Touchscreen ADC range for X axis (default: 40–955) |
| `Y_MIN`, `Y_MAX` | Touchscreen ADC range for Y axis (default: 61–927) |
| `W`, `H` | Physical plate dimensions in mm (default: 187 × 141) |
| `u0_x`, `u0_y` | Servo microsecond values that level the plate (default: 1650, 1350) |
| `Z_MIN`, `Z_MAX` | Touch detection pressure thresholds (default: 80–900) |
| `x_home`, `y_home` | Setpoint in mm — should be the physical center of your plate |

To find `u0_x` / `u0_y`, manually command each servo and observe when the plate sits level. To find the ADC range, open the Serial Monitor and slide the ball to each corner of the touchscreen while printing the raw values.

---

## Serial Output

The sketch outputs position data over Serial at **115200 baud** in CSV format:

```
timestamp_ms, x_mm, y_mm
```

This can be logged and plotted to visualize ball trajectory and evaluate controller performance.

---

## License

MIT
