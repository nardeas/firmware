from micropython import const
import machine
import ustruct
import time

PCA96X_MODE1 = const(0x00)
PCA96X_MODE1_EXTCLK = const(6) # 0=internal clock, 1=external
PCA96X_MODE1_AI = const(5) # Register autoincrement
PCA96X_MODE1_SLEEP = const(4) # 0= normal mode, 1=low power mode (oscillator off)
PCA96X_MODE1_SUB1 = const(3) # 1=responds to i2c bus subaddress 1
PCA96X_MODE1_SUB2 = const(2) # 1=responds to i2c bus subaddress 2
PCA96X_MODE1_SUB3 = const(1) # 1=responds to i2c bus subaddress 3
PCA96X_MODE1_ALLCALL = const(0) # 1=responds to i2c bus all call

PCA96X_MODE2 = const(0x01)
PCA96X_MODE2_INVRT = const(4) # 0=output logic not inverted, 1=output logic inverted
PCA96X_MODE2_OUTDRV = const(2) # 0=open-drain, 1=totem pole
PCA96X_MODE2_OUTNE1 = const(1)
PCA96X_MODE2_OUTNE0 = const(0)

PCA96X_SUBADR1 = const(0x02)
PCA96X_SUBADR2 = const(0x03)
PCA96X_SUBADR3 = const(0x04)
PCA96X_ALLCALLADR = const(0x05)
PCA96X_LED0_ON_L = const(0x06)
PCA96X_LED0_ON_H = const(0x07)
PCA96X_LED0_OFF_L = const(0x08)
PCA96X_LED0_OFF_H = const(0x09)
PCA96X_ALL_LED_ON_L = const(0xFA)
PCA96X_ALL_LED_ON_H = const(0xFB)
PCA96X_ALL_LED_OFF_L = const(0xFC)
PCA96X_ALL_LED_OFF_H = const(0xFD)

PCA96X_PRE_SCALE = const(0xFE)
PCA96X_PRE_SCALE_MIN = const(3)
PCA96X_PRE_SCALE_MAX = const(255)

PCA96X_MAX_VALUE = const(4096 - 1)
PCA96X_INTCLK = const(25_000_000)

class PCA9685:
    """
    PCA9685 PWM controller.

    :param i2c: I2C instance
    :param addr: I2C address
    :param oe: Optional output enable pin
    """
    def __init__(self, i2c, addr=0x40, oe=None):
        self.i2c = i2c
        self.address = addr
        if oe is not None:
            if not isinstance(oe, machine.Pin):
                # default to off mode
                oe = machine.Pin(oe, machine.Pin.OUT, machine.Pin.PULL_DOWN)
        self.oe = oe
        self.inverted = True
        self.frequency = 1500

    def _write_raw8(self, addr, value):
        """
        Writes byte to address.
        """
        self.i2c.writeto_mem(self.address, addr, bytearray([value]))

    def _read_raw8(self, addr):
        """
        Reads byte from address.
        """
        return int.from_bytes(self.i2c.readfrom_mem(self.address, addr, 1))

    def _normalize(self, value):
        """
        Normalize output value to supported range 0 - 4095.

        :param value: Output value
        """
        return int(min(PCA96X_MAX_VALUE, max(value, 0)))

    def setup(self, addr=None, freq=None, inverted=None):
        """
        Setup device.

        :param freq: Output PWM frequency 0-1536
        :param inverted: If true, output logic level will be inverted (drain mode)
        """
        if addr is not None:
            self.address = addr
        if freq is not None:
            self.frequency = freq
        if inverted is not None:
            self.inverted = bool(inverted)
        self.reset()
        self.freq(value=self.frequency)
        self.all(0)

    def reset(self):
        """
        Reset device.
        """
        self.off()
        
        mode1 = 0x0
        mode2 = 0x0

        # Enable autoincrement
        mode1 |= (1 << PCA96X_MODE1_AI)

        # Set inverted mode
        if self.inverted: 
            mode2 |= (1 << PCA96X_MODE2_INVRT)
        # Enable output high-impedance on OE=1
        mode2 |= (1 << PCA96X_MODE2_OUTNE0)

        # Write configuration
        self._write_raw8(PCA96X_MODE1, mode1)
        self._write_raw8(PCA96X_MODE2, mode2)

        self.on()

    def on(self):
        """
        Set output on, OE=low.
        """
        if self.oe is not None:
            self.oe.value(0)
        time.sleep_ms(1)

    def off(self):
        """
        Set output off, OE=high.
        """
        if self.oe is not None:
            self.oe.value(1)
        time.sleep_ms(1)

    def freq(self, value=None):
        """
        Sets output refresh rate (PWM frequency) if value is provided, reads and returns
        current value if None.

        - Supported refresh rate is 24Hz to 1526Hz; this is a limit set by hardware, as
          prescale value can only range from 3 to 255.

        :param value: New value or None to read register
        """
        if value is not None:
            prescale = int(PCA96X_INTCLK / (value) / 4096.0 - 1)
            prescale = min(PCA96X_PRE_SCALE_MAX, max(PCA96X_PRE_SCALE_MIN, prescale))
            
            # This procedure is required for EXTCLK
            mode1 = self._read_raw8(PCA96X_MODE1)
            # Set sleep
            self._write_raw8(PCA96X_MODE1, mode1 | (1 << PCA96X_MODE1_SLEEP))
            self._write_raw8(PCA96X_PRE_SCALE, prescale)
            # Restore mode
            self._write_raw8(PCA96X_MODE1, mode1)
            time.sleep_us(5)

            # Enable autoincrement
            mode1 |= (1 << PCA96X_MODE1_AI)
            self._write_raw8(PCA96X_MODE1, mode1)
        
        # Return current value
        return int(PCA96X_INTCLK / (4096.0 * (self._read_raw8(PCA96X_PRE_SCALE) + 1)))

    def value(self, i=None, v=None, offset=0):
        """
        Set output value at index i or read as tuple if None.

        :param i: Channel index, None to set all indices
        :param v: Output value as on duty cycle between 0 - PCA96X_MAX_VALUE
        :param offset: Offset value for phase shifting
        """
        # Check channel index
        if i >= 16 or i < 0:
            raise ValueError('Invalid channel index, 0 <= i < 16')

        # Address start by index
        addr = PCA96X_LED0_ON_L + 4*i

        # Set on/off value with programmable phase shift (offset)
        if v is not None:
            v = self._normalize(v)
            self.i2c.writeto_mem(self.address, addr, ustruct.pack('<HH', 0, v))
        # Return current value at index
        return tuple(ustruct.unpack('<HH', self.i2c.readfrom_mem(self.address, addr, 4)))

    def values(self, values, offset=0):
        """
        Set multiple values starting from channel 0. Auto-increment must be enabled

        :param values: List of values to set
        """
        stream = []
        for v in values:
            v = self._normalize(v)
            # Set on value
            stream.append(offset)
            # Set off value
            stream.append(v)
        data = ustruct.pack('<' + 'H' * len(stream), *stream)
        self.i2c.writeto_mem(self.address, PCA96X_LED0_ON_L, data)
        return len(values)

    def all(self, value, offset=0):
        """
        Sets all outputs to same value.

        :param value: Output value
        :param offset: Output offset
        """
        return self.values([value, ]*16, offset=offset)

if __name__ == '__main__':
    from machine import I2C, Pin

    i2c = I2C(1, scl=Pin(19), sda=Pin(18), freq=400_000)

    oeconfig = [10, 12, 11]
    controllers = []
    for j, addr in enumerate(i2c.scan()[:3]):
        print(f"Found PWM controller at address: 0x{addr:02X} (oe={oeconfig[j]})")
        pca = PCA9685(i2c, addr=addr)#oe=oeconfig[j])
        pca.setup(inverted=True)
        controllers.append(pca)

    print("Begin test 1...")
    for i, c in enumerate(controllers):
        print(f"Controller at 0x{c.address:02X} ({i})") 
        for v in [4096, 2048, 1024, 0]:
            c.all(v)
            print(i, v)
            time.sleep_ms(500)

    print("Begin test 2...")
    while True:
        for i, c in enumerate(controllers):
            print(f"Controller at 0x{c.address:02X} ({i})") 
            for j in range(16):
                for v in range(0, 4096, 50):
                    c.value(j, v)
                    time.sleep_ms(1)
            c.all(0)
