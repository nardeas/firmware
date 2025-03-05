from micropython import const
import machine
import struct
import time

# Control register address mapping
AD9833_CTRL_ADDR = const((0 << 15) | (0 << 14))
AD9833_FREQ0_ADDR = const((0 << 15) | (1 << 14))
AD9833_FREQ1_ADDR = const((1 << 15) | (0 << 14))
AD9833_PHASE_ADDR = const((1 << 15) | (1 << 14))

# Control register bit mapping
AD9833_D28 = const(13) # Enable 28-bit values, first write = LSB, second write = MSB
AD9833_HLB = const(12) # When D28=0 and HLB=1, writes to MSB, HLB=0 writes to LSB
                       # ;this allows value updates with single write
AD9833_FSELECT = const(11) # FSELECT=0 FREQ0 selected, FREQ1 otherwise
AD9833_PSELECT = const(10) # DB13=0 PHASE0 selected, PHASE1 otherwise
AD9833_D9 = const(9) # Reserved; set to zero
AD9833_RESET = const(8)
AD9833_SLEEP1 = const(7) # When SLEEP1=1 the internal clock is disabled
AD9833_SLEEP12 = const(6) # When SLEEP12=1 the chip is in power-down mode
AD9833_OPBITEN = const(5) # When OPBITEN=1 output at VOUT is disconnected from DAC
AD9833_D4 = const(4) # Reserved; set to zero
AD9833_DIV2 = const(3) # Used with OPBITEN
AD9833_D2 = const(2) # Reserved; set to zero
AD9833_MODE = const(1) # Sinewave when MODE=0, triangle otherwise
                       # unless used with OPBITEN
AD9833_D0 = const(0) # Reserved; set to zero

class AD9833:
    '''
    AD9833 DDS module driver

    @param sdo: SPI0 TX (MOSI) pin number
    @param clk: SPI0 CLK pin number
    @param cs: SPI0 CS pin number (chip-select, same as FSYNC)
    @param fmclk: Clock frequency in MHz
    '''
    def __init__(self, sdo, clk, cs, fmclk=25):
        self.fmclk = int(10**6 * fmclk)

        self.sdo = machine.Pin(sdo)
        self.clk = machine.Pin(clk)

        self.cs = machine.Pin(cs, machine.Pin.OUT)
        self.cs.value(1)

        self.spi = machine.SPI(
            0,
            baudrate=4_000_000,
            polarity=1,
            phase=0,
            sck=self.clk,
            mosi=self.sdo)

        # Control register state
        self._ctrl = None

    @property
    def _default_ctrl(self):
        return (
            # Enable 28-bit mode
            (1 << AD9833_D28)
            # Clear HLB bit
            &~(1 << AD9833_HLB)
            # Clear FSELECT; FREQ0 at output
            &~(1 << AD9833_FSELECT)
            # Clear PSELECT; PHASE0 at output
            &~(1 << AD9833_PSELECT)
            # Clear rest
            &~(1 << AD9833_D9)
            &~(1 << AD9833_RESET)
            &~(1 << AD9833_SLEEP1)
            &~(1 << AD9833_SLEEP12)
            &~(1 << AD9833_OPBITEN)
            &~(1 << AD9833_D4)
            &~(1 << AD9833_DIV2)
            &~(1 << AD9833_D2)
            &~(1 << AD9833_MODE)
            &~(1 << AD9833_D0)
        )

    def _write16(self, word):
        '''
        Write 16-bit word on the SPI interface
        '''
        self.cs.value(0)
        self.spi.write(struct.pack('>H', word))
        self.cs.value(1)
    
    def _write_freq(self, value, idx=0):
        '''
        Writes frequency value
        '''
        addr = AD9833_FREQ0_ADDR
        # Select FREQ1 address
        if idx > 0:
            addr = AD9833_FREQ1_ADDR

        # Expect 28-bit value; we can implement half-updating later
        # Mask to 14-bits, write LSB, then MSB
        self._write16(addr | (value & 0x3FFF)) 
        self._write16(addr | ((value >> 14) & 0x3FFF)) 

    def _write_phase(self, value, idx=0):
        '''
        Writes phase value
        '''
        word = AD9833_PHASE_ADDR
        # Set D13 for PHASE1
        if idx > 0:
            word |= (1 << 13)
        # Mask to 12-bits
        word |= (value & 0x0FFF)
        self._write16(word)

    def _reset_set(self):
        '''
        Set reset bit
        '''
        self._ctrl = self._ctrl | (1 << AD9833_RESET)
        self._write16(AD9833_CTRL_ADDR | self._ctrl)
    
    def _reset_clear(self):
        '''
        Clear reset bit
        '''
        self._ctrl = self._ctrl &~ (1 << AD9833_RESET)
        self._write16(AD9833_CTRL_ADDR | self._ctrl)

    def setup(self):
        '''
        Setup device defaults
        '''
        self._ctrl = self._default_ctrl
        self._write16(AD9833_CTRL_ADDR | self._ctrl)

    def freq_to_val(self, f):
        '''
        Returns frequency register destination value from Hz
        '''
        # Compute register value
        return int((f * (1 << 28)) / self.fmclk)

    def phase_to_val(self, p):
        '''
        Set phase in 0-4095
        '''
        return max(0, min(4095, int(p)))

    def set_mode(self, m=0):
        '''
        Set output mode
        '''
        if m != 0:
            # Set mode bit (triangle wave output)
            return self._write16(AD9833_CTRL_ADDR | self._ctrl | (1 << AD9833_MODE))
        # Clear mode bit (sine wave output)
        return self._write16(AD9833_CTRL_ADDR | self._ctrl &~(1 << AD9833_MODE))

    def set_output(self, f=None, p=None, fsel=None, psel=None, enable=False):
        '''
        Select output idx
        '''
        self._reset_set()
        if (f is not None) and (fsel is not None):
            self._write_freq(self.freq_to_val(f), idx=fsel)
        if (p is not None) and (psel is not None):
            self._write_phase(self.phase_to_val(p), idx=psel)
        if enable and ((fsel is not None) or (psel is not None)):
            reg = self._ctrl &~(1 << AD9833_FSELECT) &~ (1 << AD9833_PSELECT)
            if fsel:
                reg |= (1 << AD9833_FSELECT)
            if psel:
                reg |= (1 << AD9833_PSELECT)
            # Clear reset bit simultaneously
            self._ctrl = reg &~ (1 << AD9833_RESET)
            return self._write16(AD9833_CTRL_ADDR | self._ctrl)
        return self._reset_clear()

if __name__ == '__main__':
    awg1 = AD9833(sdo=19, clk=18, cs=17, fmclk=25) 
    awg1.setup()
    awg1.set_mode(0)
    awg1.set_output(None, 0, 0, 1, enable=True)
    awg1.set_output(1000, 0, 1, 0, enable=True)
    awg1.set_output(5000, 0, 0, 0, enable=True)

    awg2 = AD9833(sdo=19, clk=18, cs=20, fmclk=25) 
    awg2.setup()
    awg2.set_mode(0)
    awg2.set_output(None, 0, 0, 1, enable=True)
    awg2.set_output(1000, 0, 1, 0, enable=True)
    awg2.set_output(10000, 0, 0, 0, enable=True)

    t0 = time.ticks_us()
    t1 = t0
    t2 = 0
    while True:
        for i in range(0, 4096):
            #awg2.set_output(None, i, None, 0)
            awg2._write_phase(i)
            time.sleep_ms(1)
        t2 = time.ticks_us()
        print(f'[+{(t2 - t0) / 1e6}s | {(t2 - t1)/1000}ms] Phase cycle complete')
        t1 = time.ticks_us()

    #adc = machine.ADC(machine.Pin(28))
    #adc_sample_count = 100
    #adc_sample_period_ms = 1
    #phase = 0

    #while True:
    #    read_sum = 0
    #    for i in range(adc_sample_count):
    #        # Read 12-bit value
    #        read_sum += adc.read_u16() >> 4
    #        time.sleep_ms(adc_sample_period_ms)
    #    read_avg = int(read_sum / adc_sample_count)

    #    if read_avg != phase:
    #        phase = read_avg
    #        awg2.set_output(None, phase) 
    #        print('Phase: ', phase)
