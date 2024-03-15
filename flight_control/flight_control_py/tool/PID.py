
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, offset=0, time=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.offset = offset
        self.setpoint = setpoint

        self.time_prev = time
        self.e_prev = 0
        self.integral = 0

    def PID(self, measurement, time):
        # PID calculations
        e = self.setpoint - measurement
        P = self.Kp*e
        self.integral = self.integral + self.Ki*e*(time - self.time_prev)
        D = self.Kd*(e - self.e_prev)/(time - self.time_prev)# calculate manipulated variable - MV
        MV = self.offset + P + self.integral + D
        # update stored data for next iteration
        self.e_prev = e
        self.time_prev = time
        return MV

    def setpoint(self, setpoint):
        self.setpoint = setpoint

    def reset(self):
        self.integral = 0
        self.e_prev = 0
        self.time_prev = 0


