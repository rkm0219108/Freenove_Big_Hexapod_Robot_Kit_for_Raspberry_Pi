import numpy as np
import smbus


LOAD_BATTERY_CHANNEL = 0
RPI_BATTERY_CHANNEL = 4

BATTERY_LIST_SIZE = 25


class ADS7830:
    def __init__(self):
        # Get I2C bus
        self.bus = smbus.SMBus(1)
        # I2C address of the device
        self.ADS7830_DEFAULT_ADDRESS = 0x48
        # ADS7830 Command Set
        self.ADS7830_CMD = 0x84  # Single-Ended Inputs
        self.load_battery_setup = False
        self.rpi_battery_setup = False
        self.load_battery_voltage = [0] * BATTERY_LIST_SIZE
        self.rpi_battery_voltage = [0] * BATTERY_LIST_SIZE

    def read_adc(self, channel: int) -> int:
        """Select the Command data from the given provided value above"""
        COMMAND_SET = self.ADS7830_CMD | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
        self.bus.write_byte(self.ADS7830_DEFAULT_ADDRESS, COMMAND_SET)
        data = self.bus.read_byte(self.ADS7830_DEFAULT_ADDRESS)
        return data

    def voltage(self, channel: int) -> float:
        if channel == LOAD_BATTERY_CHANNEL or channel == RPI_BATTERY_CHANNEL:
            if not self.load_battery_setup or not self.rpi_battery_setup:
                for i in range(BATTERY_LIST_SIZE):
                    data: list[int] = []
                    for j in range(BATTERY_LIST_SIZE):
                        data.append(self.read_adc(channel))
                    if channel == LOAD_BATTERY_CHANNEL:
                        self.load_battery_voltage.pop(0)
                        self.load_battery_voltage.append(max(data))
                        battery_voltage = np.mean(self.load_battery_voltage) / 255.0 * 5.0
                        self.load_battery_setup = True
                    else:
                        self.rpi_battery_voltage.pop(0)
                        self.rpi_battery_voltage.append(max(data))
                        battery_voltage = np.mean(self.rpi_battery_voltage) / 255.0 * 5.0
                        self.rpi_battery_setup = True
            else:
                data: list[int] = []
                for j in range(BATTERY_LIST_SIZE):
                    data.append(self.read_adc(channel))
                if channel == LOAD_BATTERY_CHANNEL:
                    self.load_battery_voltage.pop(0)
                    self.load_battery_voltage.append(max(data))
                    battery_voltage = np.mean(self.load_battery_voltage) / 255.0 * 5.0
                else:
                    self.rpi_battery_voltage.pop(0)
                    self.rpi_battery_voltage.append(max(data))
                    battery_voltage = np.mean(self.rpi_battery_voltage) / 255.0 * 5.0
        else:
            data: list[int] = []
            for i in range(9):
                data.append(self.read_adc(channel))
            battery_voltage = np.percentile(data, 50) / 255.0 * 5.0
        return battery_voltage

    def battery_power(self):
        load_battery = round(self.voltage(LOAD_BATTERY_CHANNEL) * 3, 2)
        rpi_battery = round(self.voltage(RPI_BATTERY_CHANNEL) * 3, 2)
        return load_battery, rpi_battery


if __name__ == '__main__':
    pass
