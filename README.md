## pico-bme280

MicroPython module for performing measurements (temperature, atmospheric pressure, humidity) using BME280 sensor.

#### Usage
```
import machine
import time
from bme280 import BME280


def main():
    i2c = machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3))
    bme = BME280(i2c=i2c)

    temp, pres, hum = bme.get_readings()


if __name__ == '__main__':
    main()
```

#### Resources
- https://github.com/robert-hh/BME280 <- MicroPython reference
- https://github.com/boschsensortec/BME280_SensorAPI <- data parsing / sensor state management

