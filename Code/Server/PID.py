#encoding : UTF-8
from utils import restriction


class IncrementalPID:
    ''' PID controller'''

    def __init__(self, p: float=0.0, i: float=0.0, d: float=0.0):
        self.point = 0.0
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.last_error = 0.0
        self.p_error = 0.0
        self.p_error = 0.0
        self.d_error = 0.0
        self.i_saturation = 10.0
        self.output = 0.0

    # PID Calculation
    def compute_pid(self, feedback: float):
        error = self.point - feedback
        self.p_error = self.Kp * error
        self.i_error += error 
        self.d_error = self.Kd * (error - self.last_error)
        self.i_error = restriction(self.i_error, (-self.i_saturation, self.i_saturation))

        self.output = self.p_error + (self.Ki * self.i_error) + self.d_error
        self.last_error = error
        return self.output

    def setKp(self, proportional_gain: float):
        self.Kp = proportional_gain

    def setKi(self, integral_gain: float):
        self.Ki = integral_gain

    def setKd(self, derivative_gain: float):
        self.Kd = derivative_gain

    def setI_saturation(self, saturation: float):
        self.i_saturation = saturation
