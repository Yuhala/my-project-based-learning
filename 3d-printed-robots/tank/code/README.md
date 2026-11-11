# N20 + TB6612FNG + ESP32-S3 (tank, 2 motors)

Two MECCANIXITY N20 motors on one TB6612FNG: channel A = left, channel B = right.

Encoders are unused for now. Only connect each motor’s **Red** and **White** wires. Leave Black, Blue, Yellow, and Green disconnected.

## Power

```
Battery +  ------------->  TB6612 VM
Battery -  ------------->  TB6612 GND  ----+
ESP32 GND  --------------------------------+---- common ground
ESP32 3V3  ------------->  TB6612 VCC
```

On a free-roaming rover: `Battery + -> 5V buck -> ESP32 5V`, still with the same common ground.

Do not put battery + on ESP32 or on TB6612 **VCC**.

## ESP32 → TB6612 (control)

```
ESP32 GPIO8   ---------->  TB6612 STBY
ESP32 GPIO9   ---------->  TB6612 PWMA     (left speed)
ESP32 GPIO10  ---------->  TB6612 AIN1     (left direction)
ESP32 GPIO11  ---------->  TB6612 AIN2     (left direction)
ESP32 GPIO12  ---------->  TB6612 PWMB     (right speed)
ESP32 GPIO13  ---------->  TB6612 BIN1     (right direction)
ESP32 GPIO14  ---------->  TB6612 BIN2     (right direction)
```

## TB6612 → motors

```
TB6612 AO1  ------------>  Left N20 Red
TB6612 AO2  ------------>  Left N20 White

TB6612 BO1  ------------>  Right N20 Red
TB6612 BO2  ------------>  Right N20 White
```

If one track runs backward, swap that motor’s red/white, or set `LEFT_INVERT` / `RIGHT_INVERT` in the sketch.

## Sketch

`driveTank(left, right)` — same signs = straight; opposite signs = turn.  
`spinRoutine()` runs both tracks at PWM 64 / 128 / 192 / 255.
