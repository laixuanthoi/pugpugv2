import numpy as np
# using to map theta PUGPUG data to PWM servo


class HardwareInterface:
    def __init__(self, config):
        self.servo_params = ServoParams()

    def unZipData(self, legs, datas):
        for i in range(len(legs)):
            for j in range(len(legs[i])):
                yield legs[i][j], datas[i][j]
