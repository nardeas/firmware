from math import (
    sqrt,
    atan2,
    asin,
    degrees,
    radians
)
import time

class Fusion(object):
    """
    Provides sensor fusion algorithm that allows heading, pitch and roll to be extracted from
    accelerometer + gyroscope + compass data. This implementation uses the Madgwick algorithm.
    The calculations takes ~1.6 ms on a PyBoard @168MHz.

    # NOTES: Beta value
    
    >>> 
    The Madgwick algorithm has a "magic number" Beta which determines the tradeoff between accuracy and 
    response speed. In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 
    2.7 degrees/s) was found to give optimal accuracy. This is essentially the I coefficient in a PID
    control sense; the bigger the feedback coefficient, the faster the solution converges, usually at
    the expense of accuracy. In any case, this is the free parameter in the Madgwick filtering and
    fusion scheme.
    """
    declination = -23.5 # Optional offset for true north. A +ve value adds to heading

    def __init__(self):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.beta = sqrt(3.0 / 4.0) * radians(40) # 40
        self.mbias = (0, 0, 0)
        self.heading = 0
        self.pitch = 0
        self.roll = 0

    def calibrate(self, getter, wait=1000):
        """
        Calibrates sensor fusion with average magnetometer data over the specified timeframe

        :param getter: Function that return mx, my, mz
        :param wait: Wait time in milliseconds
        """
        # Initialise max and min lists with current values
        mmax = getter()
        mmin = mmax[:]

        t_start = time.ticks_ms()
        while ((time.ticks_ms() - t_start) < wait):
            try:
                mxyz = tuple(getter())
                for i in range(3):
                    mmax[i] = max(mmax[i], mxyz[i])
                    mmin[x] = min(mmin[i], mxyz[x])
                time.sleep_ms(10)
            except (TypeError, ValueError):
                continue
        self.mbias = tuple(map(lambda a, b: (a + b)/2, mmin, mmax))

    def update(self, axyz, gxyz, mxyz, delta): 
        """
        Recomputes state with latest data

        :param axyz: 3-tuple of accelerometer data (x, y, z)
        :param gxyz: 3-tuple of gyroscope data (x, y, z)
        :param mxyz: 3-tuple of magnetometer data (x, y, z)
        :param delta: Time-delta in seconds
        """
        # Units irrelevant (normalised)
        mx, my, mz = (mxyz[i] - self.mbias[i] for i in range(3)) 
        # Units irrelevant (normalised)
        ax, ay, az = axyz
        # Unit deg/s
        gx, gy, gz = (radians(v) for v in gxyz)
        # Short name local variable for readability
        q1, q2, q3, q4 = (self.q[i] for i in range(4))

        # Auxiliary variables to avoid repeated arithmetic
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return

        # Use reciprocal for division
        norm = 1 / norm                     
        ax *= norm
        ay *= norm
        az *= norm

        # Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return
        
        # Use reciprocal for division
        norm = 1 / norm
        mx *= norm
        my *= norm
        mz *= norm

        # Reference direction of Earth's magnetic field
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz

        # Gradient descent algorithm corrective step
        s1 = (-_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4)
             + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q2 + _2q3q4 - ay) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4)
             + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q2 + _2q3q4 - ay) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az)
             + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
             + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
             + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q2 + _2q3q4 - ay) 
              + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4)
              + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        # Normalize step magnitude
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # Compute rate of change of quaternion
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # Integrate to yield quaternion
        q1 += qDot1 * delta
        q2 += qDot2 * delta
        q3 += qDot3 * delta
        q4 += qDot4 * delta

        # Normalize quaternion
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm

        # Recompute heading
        self.heading = self.declination + degrees(atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))

        # Recompute pitch
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))

        # Recompute roll
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))


