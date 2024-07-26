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
        '''
        calculate PID
        @parm measurement: 移動誤差
        @parm time: 更新時的時間
        '''
        # PID calculations
        e = measurement - self.setpoint
        P = self.Kp * e

        time_diff = time - self.time_prev

        if time_diff == 0:
            time_diff = 1e-6  # 避免除以零的情况，设置一个非常小的值
        time_diff = time_diff * 1e-9
        self.integral = self.integral + self.Ki * e * time_diff
        D = (
            self.Kd * (e - self.e_prev) / time_diff
        )  # calculate manipulated variable - MV
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
