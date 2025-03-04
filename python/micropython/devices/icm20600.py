from micropython import const
import machine
import time

ICM20600_I2C_ADDR = 0x69
ICM20600_SMPLRT_DIV = const(0x19) # (sample rate divider configuration)

ICM20600_CONFIG = const(0x1A)
ICM20600_CONFIG_DLPF_CFG = const(0) # (gyro and temperature sensor)

ICM20600_GYRO_CONFIG = const(0x1B)
ICM20600_GYRO_CONFIG_FS_SEL = const(4)
ICM20600_GYRO_CONFIG_FCHOICE_B = const(0)

ICM20600_ACCEL_CONFIG = const(0x1C)
ICM20600_ACCEL_CONFIG_FS_SEL = const(3) # (full scale select. 2g. 4g. 8g, 16g)

ICM20600_ACCEL_CONFIG2 = const(0x1D)
ICM20600_ACCEL_CONFIG2_DEC2_CFG = const(4) # (low-noise mode, averaging over 4, 8, 16 or 32 samples)
ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B = const(3)
ICM20600_ACCEL_CONFIG2_ACCEL_A_DLPF_CFG = const(0) # (digital low-pass filter configuration)

ICM20600_LP_MODE_CFG = const(0x1E)
ICM20600_LP_MODE_CFG_GYRO_CYCLE = const(7)
ICM20600_LP_MODE_CFG_G_AVGCFG = const(4) # (low-power mode, averaging over 2, 4, 8, 16, 32, 64, 128 samples)

ICM20600_ACCEL_WOM_X_THR = const(0x20) # Wake-on X motion threshold [7:0]
ICM20600_ACCEL_WOM_Y_THR = const(0x21) # Wake-on Y motion threshold [7:0]
ICM20600_ACCEL_WOM_Z_THR = const(0x22) # Wake-on Z motion threshold [7:0]

ICM20600_FIFO_ENABLE = const(0x23) # GYRO_FIFO_EN [4] (including TEMP_OUT_H, TEMP_OUT_L), ACCEL_FIFO_EN [3]
ICM20600_FIFO_ENABLE_GYRO_FIFO_EN = const(4)
ICM20600_FIFO_ENABLE_ACCEL_FIFO_EN = const(3)

ICM20600_INT_PIN_CFG = const(0x37)
ICM20600_INT_PIN_CFG_INT_LEVEL = const(7)
ICM20600_INT_PIN_CFG_INT_OPEN = const(6)
ICM20600_INT_PIN_CFG_LATCH_INT_EN = const(5)
ICM20600_INT_PIN_CFG_INT_RD_CLEAR = const(4)
ICM20600_INT_PIN_CFG_FSYNC_INT_LEVEL = const(3)
ICM20600_INT_PIN_CFG_FSYNC_INT_MODE_EN = const(2)
ICM20600_INT_PIN_CFG_INT2_EN = const(0)

ICM20600_INT_ENABLE = const(0x38)
ICM20600_INT_ENABLE_WOM_X_INT_EN = const(7)
ICM20600_INT_ENABLE_WOM_Y_INT_EN = const(6)
ICM20600_INT_ENABLE_WOM_Z_INT_EN = const(5)
ICM20600_INT_ENABLE_FIFO_OFLOW_EN = const(4)
ICM20600_INT_ENABLE_FSYNC_INT_EN = const(3)
ICM20600_INT_ENABLE_GDRIVE_INT_EN = const(2)
ICM20600_INT_ENABLE_DATA_RDY_INT_EN = const(0)

ICM20600_FIFO_WM_INT_STATUS = const(0x39)
ICM20600_FIFO_WM_INT_STATUS_FIFO_WM_INT = const(6)

ICM20600_INT_STATUS = const(0x3A)
ICM20600_INT_STATUS_WOM_X_INT = const(7)
ICM20600_INT_STATUS_WOM_Y_INT = const(6)
ICM20600_INT_STATUS_WOM_Z_INT = const(5)
ICM20600_INT_STATUS_FIFO_OFLOW_INT = const(4)
ICM20600_INT_STATUS_DATA_RDY_INT = const(0)

ICM20600_ACCEL_XOUT_H = const(0x3B)
ICM20600_ACCEL_XOUT_L = const(0x3C)
ICM20600_ACCEL_YOUT_H = const(0x3D)
ICM20600_ACCEL_YOUT_L = const(0x3E)
ICM20600_ACCEL_ZOUT_H = const(0x3F)
ICM20600_ACCEL_ZOUT_L = const(0x40)
ICM20600_GYRO_XOUT_H = const(0x43)
ICM20600_GYRO_XOUT_L = const(0x44)
ICM20600_GYRO_YOUT_H = const(0x45)
ICM20600_GYRO_YOUT_L = const(0x46)
ICM20600_GYRO_ZOUT_H = const(0x47)
ICM20600_GYRO_ZOUT_L = const(0x48)
ICM20600_TEMP_OUT_H = const(0x41)
ICM20600_TEMP_OUT_L = const(0x42)

ICM20600_FIFO_WM_TH1 = const(0x60) # FIFO_WM_TH [9:8]
ICM20600_FIFO_WM_TH2 = const(0x61) # FIFO_WM_TH [7:0]

ICM20600_ACCEL_INTEL_CTRL = const(0x69)
ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN = const(7) 
ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE = const(6) 
ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT = const(1) 
ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE = const(0) # Sets WOM interrupt mode (mode 0 = x | y | z, mode = 1 x & y & z)
ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_OR = const(0)
ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_AND = const(1)

ICM20600_USER_CTRL = const(0x6A) # FIFO_EN [6], FIFO_RST [2], SIGN_COND_RST [0]

ICM20600_PWR_MGMT_1 = const(0x6B)
ICM20600_PWR_MGMT_1_DEVICE_RESET = const(7)
ICM20600_PWR_MGMT_1_SLEEP = const(6)
ICM20600_PWR_MGMT_1_CYCLE = const(5)
ICM20600_PWR_MGMT_1_GYRO_STANDBY = const(4)
ICM20600_PWR_MGMT_1_TEMP_DIS = const(3)
#ICM20600_PWR_MGMT_1_CLKSEL = [2:0]

ICM20600_PWR_MGMT_2 = const(0x6C)
ICM20600_PWR_MGMT_2_STBY_XA = const(5)
ICM20600_PWR_MGMT_2_STBY_YA = const(4)
ICM20600_PWR_MGMT_2_STBY_ZA = const(3)
ICM20600_PWR_MGMT_2_STBY_XG = const(2)
ICM20600_PWR_MGMT_2_STBY_YG = const(1)
ICM20600_PWR_MGMT_2_STBY_ZG = const(0)

ICM20600_FIFO_COUNTH = const(0x72)
ICM20600_FIFO_COUNTL = const(0x73)
ICM20600_FIFO_R_W = const(0x74)

class ICM20600:
    """
    ICM20600 device module implementation.

    :param i2c: I2C instance
    :param int1: Interrupt 1 pin number (int or Pin)
    :param int2: Interrupt 2 pin number (int or Pin)
    """
    def __init__(self, i2c, int1=None, int2=None, **kwargs):
        self.i2c = i2c
        self.int1 = None
        self.int2 = None
        # From datasheet
        self.Accel_Sensitivity = 16384
        self.Gyro_Sensitivity = 131
        self.Temp_Sensitivity = 326.8
        self.RoomTemp_Offset = 25.0
        # Interrupt handlers
        self._irq_data_handler = None
        self._irq_data_time = 0
        self._irq_wom_handler = None
        self._irq_wom_debounce = 0
        self._irq_wom_time = 0
        self._irq_setup(int1, int2)

    def _write_raw8(self, addr, value):
        """
        Writes byte to address.
        """
        self.i2c.writeto_mem(ICM20600_I2C_ADDR, addr, bytes([value]))

    def _read_raw8u(self, addr):
        """
        Reads unsigned byte from address.
        """
        return int.from_bytes(self.i2c.readfrom_mem(ICM20600_I2C_ADDR, addr, 1))

    def _read_raw16u(self, addr):
        """
        Reads unsigned word from address.
        """
        return (self._read_raw8u(addr) << 8) | self._read_raw8u(addr + 1)

    def _read_raw16(self, addr):
        word = self._read_raw16u(addr)
        if word & 0x8000:
            return word - 0x10000
        return word

    def _irq_handler1(self, *args, **kwargs):
        """
        Global interrupt handler on INT1 (data interrupts).
        """
        # Read interrupt status
        x_wom, y_wom, z_wom, fifo_of, data_rdy = self._irq_status()

        # Handle data interrupt
        if self._irq_data_handler is not None and (data_rdy):
            a = self.read_axyz()
            g = self.read_gxyz()
            T = self.read_t()

            t = time.ticks_us()
            # Compute delta
            td = t - self._irq_data_time
            self._irq_data_time = t
            self._irq_data_handler(td, a, g, T)

    def _irq_handler2(self, *args, **kwargs):
        """
        Global interrupt handler on INT1 (motion, FSYNC, FIFO_OF).
        """
        t = time.ticks_ms()
        
        # Read interrupt status
        x_wom, y_wom, z_wom, fifo_of, data_rdy = self._irq_status()

        # Handle WOM interrupt
        if self._irq_wom_handler is not None and (x_wom or y_wom or z_wom):
            if (t - self._irq_wom_time) > self._irq_wom_debounce:
                self._irq_wom_time = t
                self._irq_wom_handler(x_wom, y_wom, z_wom)

    def _irq_status(self):
        """
        Read interrupt status.
        """
        # Read interrupt status (auto-clears register)
        int_status = self._read_raw8u(ICM20600_INT_STATUS)
        # Read value tuple
        (x_wom, y_wom, z_wom, fifo_of, data_rdy) = (
            (int_status & (1 << ICM20600_INT_STATUS_WOM_X_INT)),
            (int_status & (1 << ICM20600_INT_STATUS_WOM_Y_INT)),
            (int_status & (1 << ICM20600_INT_STATUS_WOM_Z_INT)),
            (int_status & (1 << ICM20600_INT_STATUS_FIFO_OFLOW_INT)),
            (int_status & (1 << ICM20600_INT_STATUS_DATA_RDY_INT)),
        )
        # Return tuple
        return (x_wom, y_wom, z_wom, fifo_of, data_rdy)

    def _irq_setup(self, *args, **kwargs):
        """
        Global interrupt handler setup for all provided interrupt pins.
        """
        int1, int2 = args

        # Default register configuration
        int_pin_cfg = (0 &
            # INT/DRDY pin is active HIGH
            ~(1 << ICM20600_INT_PIN_CFG_INT_LEVEL) &
            # INT/DRDY pin is configured as push-pull
            ~(1 << ICM20600_INT_PIN_CFG_INT_OPEN) |
            # Level held until interrupt status is cleared
            (1 << ICM20600_INT_PIN_CFG_LATCH_INT_EN) &
            # Interrupt status cleared by reading INT_STATUS register
            ~(1 << ICM20600_INT_PIN_CFG_INT_RD_CLEAR) 
        )

        if int1 is not None:
            if not isinstance(int1, machine.Pin):
                int1 = machine.Pin(int1, machine.Pin.IN, machine.Pin.PULL_DOWN)
            self.int1 = int1
            self.int1.irq(trigger=machine.Pin.IRQ_RISING, handler=self._irq_handler1)

        if int2 is not None:
            if not isinstance(int2, machine.Pin):
                int2 = machine.Pin(int2, machine.Pin.IN, machine.Pin.PULL_DOWN)
            self.int2 = int2
            self.int2.irq(trigger=machine.Pin.IRQ_RISING, handler=self._irq_handler2)
            # Enable INT2
            int_pin_cfg |= (1 << ICM20600_INT_PIN_CFG_INT2_EN)
        
        # Write register state
        self._write_raw8(ICM20600_INT_PIN_CFG, int_pin_cfg)

    def _accel_snr_32x(self):
        """
        Sets the accelerometer noise level to default mode, averaging over 32 samples.
        """
        # Low-noise mode configuration, average over 8 samples
        self._write_raw8(ICM20600_ACCEL_CONFIG2, (
            (7 << ICM20600_ACCEL_CONFIG2_ACCEL_A_DLPF_CFG) &
            # Disables DLPF override
            ~(1 << ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B) |
            # Sets averaging mode to 32x
            (3 << ICM20600_ACCEL_CONFIG2_DEC2_CFG)
        ))

    def _gyro_snr_16x(self):
        """
        Sets the gyroscope noise level to default mode, averaging over 32 samples.
        """
        self._write_raw8(ICM20600_LP_MODE_CFG, (
            (1 << ICM20600_LP_MODE_CFG_GYRO_CYCLE) |
            (4 << ICM20600_LP_MODE_CFG_G_AVGCFG)
        ))

    def _odr_10Hz(self):
        """
        Sets output data rate to 10Hz.
        """
        # Set output data rate to 10Hz, (ODR = internal sample rate (1kHz) / (1 + SMPLRT_DIV)
        self._write_raw8(ICM20600_SMPLRT_DIV, 99)

    def _odr_100Hz(self):
        """
        Sets output data rate to 100Hz.
        """
        # Set output data rate to 100Hz, (ODR = internal sample rate (1kHz) / (1 + SMPLRT_DIV)
        self._write_raw8(ICM20600_SMPLRT_DIV, 9)

    def _out_defaults(self):
        """
        Sets output defaults.
        """
        self._accel_snr_32x()
        self._gyro_snr_16x()
        self._odr_10Hz()
        # Read DEC2_CFG to determine value range
        dec2_cfg = (self._read_raw8u(ICM20600_ACCEL_CONFIG2) >> ICM20600_ACCEL_CONFIG2_DEC2_CFG) & 0b11
        # Recompute accelerator sensitivity value
        self.Accel_Sensitivity = 16384
        self.Accel_Sensitivity /= 2 ** dec2_cfg

    def _wom_enable(self, mode=ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_OR):
        """
        Enable wake on motion register configuration

        :param mode: x | y | z (OR) mode = 0, x & y & z (AND) mode = 1
        """
        _pwr_mgmt_1 = self._read_raw8u(ICM20600_PWR_MGMT_1)
        self._write_raw8(ICM20600_PWR_MGMT_1, _pwr_mgmt_1 &
            ~(1 << ICM20600_PWR_MGMT_1_CYCLE) &
            ~(1 << ICM20600_PWR_MGMT_1_SLEEP) &
            ~(1 << ICM20600_PWR_MGMT_1_GYRO_STANDBY)
        )
        _pwr_mgmt_2 = self._read_raw8u(ICM20600_PWR_MGMT_2)
        # Make sure accelerometer is enabled and gyro is disabled
        self._write_raw8(ICM20600_PWR_MGMT_2, _pwr_mgmt_2 &
            ~(1 << ICM20600_PWR_MGMT_2_STBY_XA) &
            ~(1 << ICM20600_PWR_MGMT_2_STBY_YA) &
            ~(1 << ICM20600_PWR_MGMT_2_STBY_ZA) |
            (1 << ICM20600_PWR_MGMT_2_STBY_XG) |
            (1 << ICM20600_PWR_MGMT_2_STBY_YG) |
            (1 << ICM20600_PWR_MGMT_2_STBY_ZG)
        )
        _accel_config2 = self._read_raw8u(ICM20600_ACCEL_CONFIG2)
        # Override digital low pass filter by setting FCHOICE_B (4kHz sample rate)
        # set A_DLPF_CFG to 0b001 according to datasheet (no effect here)
        self._write_raw8(ICM20600_ACCEL_CONFIG2, _accel_config2 |
            (1 << ICM20600_ACCEL_CONFIG2_ACCEL_FCHOICE_B) |
            (1 << ICM20600_ACCEL_CONFIG2_ACCEL_A_DLPF_CFG)
        )
        _accel_intel_ctrl = self._read_raw8u(ICM20600_ACCEL_INTEL_CTRL)
        # Enable OR mode
        if mode == ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE_OR:
            _accel_intel_ctrl &= ~(1 << ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE)
        # Enable AND mode
        else: _accel_intel_ctrl |= (1 << ICM20600_ACCEL_INTEL_CTRL_WOM_TH_MODE)
        # Enable accelerometer intelligent mode
        self._write_raw8(ICM20600_ACCEL_INTEL_CTRL, _accel_intel_ctrl | 
            (1 << ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN) |
            (1 << ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE)
        )
        # Re-enable low-power mode
        _pwr_mgmt_1 = self._read_raw8u(ICM20600_PWR_MGMT_1)
        self._write_raw8(ICM20600_PWR_MGMT_1, _pwr_mgmt_1 | (1 << ICM20600_PWR_MGMT_1_CYCLE))

        return True

    def _wom_disable(self):
        """
        Disable wake on motion register configuration.
        """
        _int_enable = self._read_raw8u(ICM20600_INT_ENABLE)
        # Clear all WOM interrupt enable bits
        self._write_raw8(ICM20600_INT_ENABLE, _int_enable &
             ~(1 << ICM20600_INT_ENABLE_WOM_X_INT_EN) &
             ~(1 << ICM20600_INT_ENABLE_WOM_Y_INT_EN) &
             ~(1 << ICM20600_INT_ENABLE_WOM_Z_INT_EN)
        )
        # Re-enable accelerometer and gyro (clear all bits)
        self._write_raw8(ICM20600_PWR_MGMT_2, 0)

        # Re-enable normal operation mode
        _pwr_mgmt_1 = self._read_raw8u(ICM20600_PWR_MGMT_1)
        self._write_raw8(ICM20600_PWR_MGMT_1, _pwr_mgmt_1 &
            ~(1 << ICM20600_PWR_MGMT_1_CYCLE) &
            ~(1 << ICM20600_PWR_MGMT_1_SLEEP) &
            ~(1 << ICM20600_PWR_MGMT_1_GYRO_STANDBY)
        )
        # Disable accelerator intelligent mode
        _accel_intel_ctrl = self._read_raw8u(ICM20600_ACCEL_INTEL_CTRL)
        self._write_raw8(ICM20600_ACCEL_INTEL_CTRL, _accel_intel_ctrl & 
            ~(1 << ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_EN) &
            ~(1 << ICM20600_ACCEL_INTEL_CTRL_ACCEL_INTEL_MODE)
        )
        self._out_defaults()
        return True

    def _data_enable(self):
        """
        Enable data ready register configuration.
        """
        int_enable = self._read_raw8u(ICM20600_INT_ENABLE)
        self._write_raw8(ICM20600_INT_ENABLE, int_enable | (1 << ICM20600_INT_ENABLE_DATA_RDY_INT_EN))
        return True

    def _data_disable(self):
        """
        Disable data ready register configuration.
        """
        int_enable = self._read_raw8u(ICM20600_INT_ENABLE)
        self._write_raw8(ICM20600_INT_ENABLE, int_enable & ~(1 << ICM20600_INT_ENABLE_DATA_RDY_INT_EN))
        return True

    def setup(self):
        """
        Setup device for default operation mode.
        """
        self.reset()
        self._out_defaults()

    def reset(self):
        """
        Reset device.
        """
        # Disable interrupts intitially
        self._wom_disable()
        self._data_disable()
        # Avoid output limiting to below 0x7fff by setting the OUTPUT_LIMI bit
        self._write_raw8(ICM20600_ACCEL_INTEL_CTRL, 0 | (1 << ICM20600_ACCEL_INTEL_CTRL_OUTPUT_LIMIT)) 
        self._write_raw8(ICM20600_PWR_MGMT_1, 0)
        time.sleep_ms(100)
        
    def read_axyz(self):
        """
        Returns current accelerometer x, y, z values in counts
        """
        return (self._read_raw16(ICM20600_ACCEL_XOUT_H), 
                self._read_raw16(ICM20600_ACCEL_YOUT_H), 
                self._read_raw16(ICM20600_ACCEL_ZOUT_H))

    def read_gxyz(self):
        """
        Returns current gyroscope x, y, z values in counts
        """
        return (self._read_raw16(ICM20600_GYRO_XOUT_H),
                self._read_raw16(ICM20600_GYRO_YOUT_H),
                self._read_raw16(ICM20600_GYRO_ZOUT_H))

    def read_t(self):
        """
        Returns current temperature value in degrees C
        """
        return self._read_raw16(ICM20600_TEMP_OUT_H) / self.Temp_Sensitivity + self.RoomTemp_Offset

    def set_wom_threshold(self, x=None, y=None, z=None, debounce=0):
        """
        Sets the accelerometer wake on motion threshold

        :param x: X threshold value in counts
        :param y: Y threshold value in counts
        :param z: Z threshold value in counts
        :param debounce: Debounce level in ms
        """
        self._int_enable = self._read_raw8u(ICM20600_INT_ENABLE)
        if x is not None:
            self._int_enable |= (1 << ICM20600_INT_ENABLE_WOM_X_INT_EN)
            self._write_raw8(ICM20600_ACCEL_WOM_X_THR, x)
        if y is not None:
            self._int_enable |= (1 << ICM20600_INT_ENABLE_WOM_Y_INT_EN)
            self._write_raw8(ICM20600_ACCEL_WOM_Y_THR, y)
        if z is not None:
            self._int_enable |= (1 << ICM20600_INT_ENABLE_WOM_Z_INT_EN)
            self._write_raw8(ICM20600_ACCEL_WOM_Z_THR, z)
        self._write_raw8(ICM20600_INT_ENABLE, self._int_enable)
        self._irq_wom_debounce = debounce

    def set_wom(self, callback=None, **kwargs):
        """
        Sets wake on motion handler.

        :param callback: User function to handle the WOM interrupt
        :param x: X threshold value in counts
        :param y: Y threshold value in counts
        :param z: Z threshold value in counts
        """
        if not callable(callback):
            return self._wom_disable()
        if kwargs:
            self.set_wom_threshold(**kwargs)
        # Save reference to WOM interrupt handler
        self._irq_wom_handler = callback
        # Enable WOM interrupt
        self._wom_enable()

    def set_handler(self, callback=None, **kwargs):
        """
        Sets data handler.
        """
        if not callable(callback):
            return self._data_disable()
        self._irq_data_handler = callback
        # Enable data interrupt
        self._data_enable()

if __name__ == '__main__':
    from machine import I2C
    i2c = I2C(0, scl=21, sda=20, freq=400_000)

    ag = ICM20600(i2c, int1=8, int2=9)
    ag.setup()
    ag.set_handler(lambda td, a, g: print("Data", td, a, g))

    while True:
        time.sleep_ms(100)
