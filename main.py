import machine
import time
from bme280 import BME280


def main():
    i2c = machine.I2C(1, sda=machine.Pin(2), scl=machine.Pin(3))
    bme = BME280(i2c=i2c)

    for _ in range(5):
        temp, pres, hum = bme.get_readings()

        print("###")
        print(f"Temperature: {round(temp, 1)}°C")
        print(f"Pressure: {round(pres, 1)}hPa")
        print(f"Humidity: {round(hum, 1)}%")
        print("###")

        time.sleep(2)


if __name__ == '__main__':
    main()
