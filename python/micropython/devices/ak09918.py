from micropython import const
import ustruct
import time

AK09918_I2C_ADDR = 0x0C
AK09918_WIA1 = const(0x00) # Company ID
AK09918_WIA2 = const(0x01) # Device ID

AK09918_ST1 = const(0x10) # Status 1
AK09918_ST1_DRDY = const(0) # Data ready bit

AK09918_HXL = const(0x11)
AK09918_HXH = const(0x12)
AK09918_HYL = const(0x13)
AK09918_HYH = const(0x14)
AK09918_HZL = const(0x15)
AK09918_HZH = const(0x16)

AK09918_ST2 = const(0x18) # Status 2
AK09918_ST2_HOFL = const(3) 

AK09918_CNTL2 = const(0x31) # Control settings 2
AK09918_CNTL2_MODE = const(0)
AK09918_POWER_DOWN = const(0b00000)
AK09918_NORMAL = const(0b00001)
AK09918_CONTINUOUS_10HZ = const(0b00010)
AK09918_CONTINUOUS_20HZ = const(0b00100)
AK09918_CONTINUOUS_50HZ = const(0b00110)
AK09918_CONTINUOUS_100HZ = const(0b01000)
AK09918_SELF_TEST = const(0b10000)

AK09918_CNTL3 = const(0x32) # Control settings 3
AK09918_CNTL3_SRST = const(0)

class AK09918:
    """
    AK09918 device module implementation.

    :param i2c: I2C instance
    """
    def __init__(self, i2c):
        self.i2c = i2c
        self.wake_to = None
        # From datasheet
        # Sensitivity 0.15uT / LSB
        self.Magnetometer_Sensitivity = 0.15

    def _write_raw8(self, addr, value):
        """
        Writes byte to address.
        """
        self.i2c.writeto_mem(AK09918_I2C_ADDR, addr, bytes([value]))

    def _read_raw8u(self, addr):
        """
        Reads unsigned byte from address.
        """
        return int.from_bytes(self.i2c.readfrom_mem(AK09918_I2C_ADDR, addr, 1))

    def _read_bytes(self, addr, length):
        """
        Reads <length> number of bytes starting from <addr>.

        :param addr: Address of first byte
        :param length: Total number of bytes to read
        """
        return self.i2c.readfrom_mem(AK09918_I2C_ADDR, addr, length)

    def _read_data(self):
        """
        Read raw magnetometer data, return as 3-tuple.
        """
        st1 = self._read_raw8u(AK09918_ST1)
        # Check data ready bit
        if (st1 & (1 << AK09918_ST1_DRDY)):
            data = self._read_bytes(AK09918_HXL, 6)
            # NOTE: St2 must be read to finish the data reading procedure in continuous mode
            st2 = self._read_raw8u(AK09918_ST2)
            # Check magnetometer overflow bit
            if not (st2 & (1 << AK09918_ST2_HOFL)):
                # Two's completement little-endian unpacking
                return ustruct.unpack('<hhh', data)
            else:
                self.reset()
                self.set_mode(self.wake_to)
        return None

    def setup(self, mode=AK09918_CONTINUOUS_100HZ):
        """
        Setup device for default operation mode.
        """
        self.reset()
        self.set_mode(mode)

    def reset(self):
        """
        Reset device.
        """
        self._write_raw8(AK09918_CNTL3, (1 << AK09918_CNTL3_SRST))
        time.sleep_ms(100)

    def read_mxyz(self):
        """
        Read magnetometer data, return as 3-tuple.
        """
        data = self._read_data()
        if data is not None:
            mx, my, mz = data
            return (
                mx * self.Magnetometer_Sensitivity,
                my * self.Magnetometer_Sensitivity,
                mz * self.Magnetometer_Sensitivity
            )
        return None

    def set_mode(self, mode):
        """
        Set the operation mode of the sensor.

        :param mode: Operation mode
        """
        if mode is not AK09918_POWER_DOWN:
            self.wake_to = mode
        # Always transit trough power down mode
        self.off()
        # Then set the desired mode
        self._write_raw8(AK09918_CNTL2, (mode << AK09918_CNTL2_MODE))
        time.sleep_ms(100)

    def off(self):
        """
        Power down mode.
        """
        self._write_raw8(AK09918_CNTL2, (AK09918_POWER_DOWN << AK09918_CNTL2_MODE))
        time.sleep_ms(100)

    def on(self):
        """
        Restore previous mode.
        """
        if (self.wake_to is not None):
            self.set_mode(self.wake_to)

if __name__ == '__main__':
    from machine import I2C
    i2c = I2C(0, scl=21, sda=20, freq=400_000)

    m = AK09918(i2c) 
    m.setup()

    try:
        while True:
            print(m.read_mxyz())
            time.sleep_ms(10)
    except KeyboardInterrupt:
        print("")
