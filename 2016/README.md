# autonomous_vehicle

Arduino source code for an autonomous vehicle using the following components:

1. Arduino Mega 2560
2. Adafruit Ultimate GPS (Serial)
3. Sparkfun HMC6352 magenetometer breakout (I2C)
4. Octasonic HC-SR04 Breakout (SPI)
5. Pololu Qik 2s12v10 motor controller board (Serial)

Need to replace standard SD library with this one that uses soft SPI on pins 10,11,12,13 instead of 50,51,52

https://github.com/adafruit/SD

## GPS

| Arduino | GPS   | Wire color |
|---------|-------|------------|
| 18      | RX    | Gray       |
| 19      | TX    | White      |
| 5V      | VCC   | Blue       |
| GND     | GND   | Purple     |

## Compass

| Arduino | Compass | Wire color |
|---------|---------|------------|
| 20      | SDA     | Gray       |
| 21      | SCL     | White      |
| 5V      | VCC     | Blue       |
| GND     | GND     | Purple     |

## Qik

| Arduino | Component   | Wire color |
|---------|-------------|------------|
| 62 (A8) | Qik RESET   | White      |
| 63 (A9) | Qik TX      | Gray       |
| 64 (A10)| Qik RX      | Purple     |
| GND     | Qik GND     | Black      |

## Octasonic

| Arduino | Octasonic | Wire color |
|---------|-----------|------------|
| 50      | MISO      | Yellow     |
| 51      | MOSI      | Green      |
| 52      | SCK       | Orange     |
| 53      | SS        | Blue       |

## SD Shield

| Arduino | Component   | Wire color |
|---------|-------------|------------|
| 10-13   | SD shield   | N/A        |

