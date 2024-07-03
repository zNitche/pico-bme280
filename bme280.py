# Reference
# https://github.com/robert-hh/BME280
# https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c

import struct
from micropython import const
import time

CONTROL_REGISTER_ADDR = const(0xF4)
TEMP_PRESS_CALIBRATION_DATA_ADDR = const(0x88)
HUMIDITY_CALIBRATION_DATA_ADDR = const(0xE1)
DATA_ADDR = const(0xF7)
REGISTER_CONTROL_HUMIDITY = const(0xF2)

SLEEP_MODE = const(0x00)
FORCED_MODE = const(0x01)


# https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c#L1192
# BME280_CONCAT_BYTES etc. covered by struct.unpack
class CalibrationData:
    def __init__(self, temp_press_data: bytearray, humidity_data: bytearray):
        temp_pres_struct = struct.unpack("<HhhH8hBB", temp_press_data)
        humidity_struct = struct.unpack("<hB4b", humidity_data)

        self.dig_T1 = temp_pres_struct[0]
        self.dig_T2 = temp_pres_struct[1]
        self.dig_T3 = temp_pres_struct[2]

        self.dig_P1 = temp_pres_struct[3]
        self.dig_P2 = temp_pres_struct[4]
        self.dig_P3 = temp_pres_struct[5]
        self.dig_P4 = temp_pres_struct[6]
        self.dig_P5 = temp_pres_struct[7]
        self.dig_P6 = temp_pres_struct[8]
        self.dig_P7 = temp_pres_struct[9]
        self.dig_P8 = temp_pres_struct[10]
        self.dig_P9 = temp_pres_struct[11]

        self.dig_H1 = temp_pres_struct[13]

        self.dig_H2 = humidity_struct[0]
        self.dig_H3 = humidity_struct[1]
        self.dig_H4 = humidity_struct[2] << 4 | (humidity_data[4] & 0x0F)
        self.dig_H5 = humidity_struct[4] << 4 | (humidity_data[4] >> 4)
        self.dig_H6 = humidity_struct[5]


class BME280:
    def __init__(self, i2c, oversampling=5, address=0x76):
        self.address = address
        self.i2c = i2c

        # min 1, max 5
        self.__oversampling_mode = oversampling
        self.__mode_calc_baseline = self.__oversampling_mode << 5 | self.__oversampling_mode << 2

        self.__setup()
        self.calibration_data: CalibrationData = self.__get_calibration_data()

    def __setup(self):
        # Put to sleep
        mode_calc = self.__mode_calc_baseline | SLEEP_MODE
        self.i2c.writeto_mem(self.address, CONTROL_REGISTER_ADDR, bytearray([mode_calc]))

    def __get_calibration_data(self) -> CalibrationData:
        temp_press_data = self.i2c.readfrom_mem(self.address, TEMP_PRESS_CALIBRATION_DATA_ADDR, 26)
        humidity_data = self.i2c.readfrom_mem(self.address, HUMIDITY_CALIBRATION_DATA_ADDR, 7)

        return CalibrationData(temp_press_data, humidity_data)

    def __read_raw_data(self) -> tuple[int, int, int]:
        # https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c#L836
        # https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c#L620

        # set mode to forced
        mode_calc = self.__mode_calc_baseline | FORCED_MODE

        self.i2c.writeto_mem(self.address, REGISTER_CONTROL_HUMIDITY, bytearray([self.__oversampling_mode]))
        time.sleep_ms(50)

        self.i2c.writeto_mem(self.address, CONTROL_REGISTER_ADDR, bytearray([mode_calc]))
        time.sleep_ms(50)

        results: bytearray = self.i2c.readfrom_mem(self.address, DATA_ADDR, 8)

        # shifts performed by trial and error, arduino lib says x << 12 and x << 4
        # I changed it till got positive values
        pressure = ((results[0] << 16) | (results[1] << 8) | results[2]) >> 4
        temperature = ((results[3] << 16) | (results[4] << 8) | results[5]) >> 4
        humidity = (results[6] << 8) | results[7]

        return temperature, pressure, humidity

    # https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c#L987
    def __compensate_temperature(self, raw_data: int) -> tuple[float, float]:
        var1 = raw_data / 16384 - self.calibration_data.dig_T1 / 1024
        var1 = var1 * self.calibration_data.dig_T2
        var2 = (raw_data / 131072 - self.calibration_data.dig_T1 / 8192)
        var2 = (var2 * var2) * self.calibration_data.dig_T3
        t_fine = var1 + var2
        temperature = (var1 + var2) / 5120

        return temperature, t_fine

    # https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c#L1016
    def __compensate_pressure(self, raw_data: int, t_fine: float = 0) -> float:
        pressure = 0

        var1 = t_fine / 2 - 64000
        var2 = var1 * var1 * self.calibration_data.dig_P6 / 32768
        var2 = var2 + var1 * self.calibration_data.dig_P5 * 2
        var2 = (var2 / 4) + (self.calibration_data.dig_P4 * 65536)
        var3 = self.calibration_data.dig_P3 * var1 * var1 / 524288
        var1 = (var3 + self.calibration_data.dig_P2 * var1) / 524288
        var1 = (1 + var1 / 32768) * self.calibration_data.dig_P1

        if var1 > 0:
            pressure = 1048576 - raw_data
            pressure = (pressure - (var2 / 4096)) * 6250 / var1
            var1 = self.calibration_data.dig_P9 * pressure * pressure / 2147483648
            var2 = pressure * self.calibration_data.dig_P8 / 32768
            pressure = (pressure + (var1 + var2 + self.calibration_data.dig_P7) / 16)

        return pressure / 100

    # https://github.com/boschsensortec/BME280_SensorAPI/blob/958cef65daa46123bdc3a8da9fb2a1b0abffc653/bme280.c#L952
    def __compensate_humidity(self, raw_data: int, t_fine: float = 0) -> float:
        var1 = t_fine - 76800
        var2 = (self.calibration_data.dig_H4 * 64 + (self.calibration_data.dig_H5 / 16384) * var1)
        var3 = raw_data - var2
        var4 = self.calibration_data.dig_H2 / 65536
        var5 = (1 + (self.calibration_data.dig_H3 / 67108864) * var1)
        var6 = 1 + (self.calibration_data.dig_H6 / 67108864) * var1 * var5
        var6 = var3 * var4 * (var5 * var6)
        humidity = var6 * (1 - self.calibration_data.dig_H1 * var6 / 524288)

        return humidity

    def get_readings(self) -> tuple[float, float, float]:
        raw_readings = self.__read_raw_data()
        temperature, t_fine = self.__compensate_temperature(raw_readings[0])
        pressure = self.__compensate_pressure(raw_readings[1], t_fine)
        humidity = self.__compensate_humidity(raw_readings[2], t_fine)

        return temperature, pressure, humidity
