# Steering

Active steering controller running on Rigby. Reads a target angle over serial, reads current wheel angle from a potentiometer, and drives the steering motor with proportional control.

Source: `FT-Hardware/Code/Rigby/SteeringCode/SteeringWithPotCode/SteeringWithPotCode.ino`

## How It Works

1. Receives a target angle (degrees) over serial as a newline-terminated float via [FTSerial](ftserial.md)
2. Reads current angle from a potentiometer on the steering column
3. Proportional controller drives the motor until the error is within the dead band
4. Prints telemetry (angle, target, error) at ~20 Hz

## Hardware Setup

| Pin | Constant   | Function                          |
|-----|------------|-----------------------------------|
| A3  | `POT_PIN`  | Potentiometer (steering feedback) |
| 5   | `RPWM_PIN` | Right PWM to motor driver         |
| 6   | `LPWM_PIN` | Left PWM to motor driver          |
| 7   | `REN_PIN`  | Right enable on motor driver      |
| 8   | `LEN_PIN`  | Left enable on motor driver       |

Motor driver is a BTS7960 dual H-bridge. If steering turns the wrong way, swap `RPWM_PIN` and `LPWM_PIN`.

## Configuration Constants

**Calibration — must be set before deploying:**

| Constant        | Default | Description                                        |
|-----------------|---------|----------------------------------------------------|
| `POT_ADC_LEFT`  | `150`   | ADC reading at left mechanical extreme             |
| `POT_ADC_RIGHT` | `870`   | ADC reading at right mechanical extreme            |

`POT_ADC_LEFT` must always be less than `POT_ADC_RIGHT`. Move steering to each extreme and record the ADC values.

**Control:**

| Constant            | Default | Description                                                           |
|---------------------|---------|-----------------------------------------------------------------------|
| `ANGLE_LIMIT_DEG`   | `80.0`  | Max steering angle in either direction (degrees)                      |
| `STOP_BAND_DEG`     | `1.0`   | Dead band — how close to target before motor stops                    |
| `GAIN`              | `3.0`   | Proportional gain                                                     |
| `PWM_MIN`           | `35`    | Minimum PWM (avoids stalling)                                         |
| `PWM_MAX`           | `200`   | Maximum PWM                                                           |
| `ANGLE_SMOOTHING`   | `0.20`  | Smoothing filter coefficient. Must not be 0 or motor won't move       |
| `SERIAL_TIMEOUT_MS` | `1500`  | Miliseconds without serial before timeout                             |
| `HOLD_LAST_VALUE`   | `true`  | On timeout: hold last angle (`true`) or return to 0 (`false`)         |

## Serial Protocol

**Input:** newline-terminated floats
```
25.5
```

**Output:** telemetry at ~20 Hz
```
angle= 12.34 target= 25.50 error= 13.16
```

## Functions

```cpp
float directionCheck(float x, float a, float b)
```
Clamps `x` to `[a, b]`. Used to enforce angle limits.

---

```cpp
float mapPotToDeg(int adc)
```
Maps a raw ADC reading to degrees. Linear from `[POT_ADC_LEFT, POT_ADC_RIGHT]` → `[-ANGLE_LIMIT_DEG, +ANGLE_LIMIT_DEG]`.

---

```cpp
void drive(int pwmSigned)
```
Drives the motor. Positive = right, negative = left, 0 = stop.

---

```cpp
void coastStop()
```
Stops the motor.

---

```cpp
void driverEnable(bool en)
```
Enables or disables the motor driver.

