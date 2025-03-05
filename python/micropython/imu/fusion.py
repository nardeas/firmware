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
    accelerometer + gyroscope + compass data. this implementation uses the madgwick algorithm.
    the calculations takes ~1.6 ms on a pyboard @168mhz.

    # notes: beta value
    
    >>> 
    the madgwick algorithm has a "magic number" beta which determines the tradeoff between accuracy and 
    response speed. in the original madgwick study, beta of 0.041 (corresponding to gyromeaserror of 
    2.7 degrees/s) was found to give optimal accuracy. this is essentially the i coefficient in a pid
    control sense; the bigger the feedback coefficient, the faster the solution converges, usually at
    the expense of accuracy. in any case, this is the free parameter in the madgwick filtering and
    fusion scheme.
    """
    declination = -23.5 # optional offset for true north. a +ve value adds to heading

    def __init__(self):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.beta = sqrt(3.0 / 4.0) * radians(40) # 40
        self.mbias = (0, 0, 0)
        self.heading = 0
        self.pitch = 0
        self.roll = 0

    def calibrate(self, getter, wait=1000):
        """
        calibrates sensor fusion for average magnetometer data

        :param getter: function that return mx, my, mz
        :param wait: wait time in milliseconds
        """
        # initialise max and min lists with current values
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
            except (typeerror, valueerror):
                continue
        self.mbias = tuple(map(lambda a, b: (a + b)/2, mmin, mmax))

    def update(self, axyz, gxyz, mxyz, delta): 
        """
        recoumputes state with latest data

        :param axyz: 3-tuple of accelerometer data (x, y, z)
        :param gxyz: 3-tuple of gyroscope data (x, y, z)
        :param mxyz: 3-tuple of magnetometer data (x, y, z)
        :param delta: time-delta in seconds
        """
        # units irrelevant (normalised)
        mx, my, mz = (mxyz[i] - self.mbias[i] for i in range(3)) 
        # units irrelevant (normalised)
        ax, ay, az = axyz
        # unit deg/s
        gx, gy, gz = (radians(v) for v in gxyz)
        # short name local variable for readability
        q1, q2, q3, q4 = (self.q[i] for i in range(4))

        # auxiliary variables to avoid repeated arithmetic
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

        # normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az)
        if (norm == 0):
            return

        # use reciprocal for division
        norm = 1 / norm                     
        ax *= norm
        ay *= norm
        az *= norm

        # normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz)
        if (norm == 0):
            return
        
        # use reciprocal for division
        norm = 1 / norm
        mx *= norm
        my *= norm
        mz *= norm

        # reference direction of earth's magnetic field
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

        # gradient descent algorithm corrective step
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

        # normalize step magnitude
        norm = 1 / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)
        s1 *= norm
        s2 *= norm
        s3 *= norm
        s4 *= norm

        # compute rate of change of quaternion
        qdot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qdot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qdot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qdot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4

        # integrate to yield quaternion
        q1 += qdot1 * delta
        q2 += qdot2 * delta
        q3 += qdot3 * delta
        q4 += qdot4 * delta

        # normalize quaternion
        norm = 1 / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)
        self.q = q1 * norm, q2 * norm, q3 * norm, q4 * norm

        # recompute heading
        self.heading = self.declination + degrees(atan2(2.0 * (self.q[1] * self.q[2] + self.q[0] * self.q[3]),
            self.q[0] * self.q[0] + self.q[1] * self.q[1] - self.q[2] * self.q[2] - self.q[3] * self.q[3]))

        # recompute pitch
        self.pitch = degrees(-asin(2.0 * (self.q[1] * self.q[3] - self.q[0] * self.q[2])))

        # recompute roll
        self.roll = degrees(atan2(2.0 * (self.q[0] * self.q[1] + self.q[2] * self.q[3]),
            self.q[0] * self.q[0] - self.q[1] * self.q[1] - self.q[2] * self.q[2] + self.q[3] * self.q[3]))

