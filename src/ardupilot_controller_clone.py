class MotorController(object):
    def __init__(self):
        self.yaw_factor =     [0.0,  1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.lateral_factor = [0.0,  0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0]
        self.forward_factor = [0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

        self.motor_min_pwm = 1400
        self.motor_max_pwm = 1500

    def set_current_position(self, position):
        self.current_position = position

    def set_goal_position(self, position):
        self.goal_position = position

    def get_yfl_out(self, yaw_thrust, forward_thrust, lateral_thrust):
        yfl_out = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        yfl_max = float("-inf")
        for i in range(len(yfl_out)):
            yfl_out[i] = yaw_thrust * self.yaw_factor[i] + forward_thrust * self.forward_factor[i] + lateral_thrust * self.lateral_factor[i]

            if abs(yfl_out[i]) > yfl_max:
                yfl_max = abs(yfl_out[i])

        for i in range(len(yfl_out)):
            yfl_out[i] = yfl_out[i] / yfl_max

        return yfl_out

    def yfl_to_motor_speeds(self, yfl_out):
        motor_speeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(motor_speeds):
            motor_speeds[i] = 1500.0 + yfl_out * 100.0 

            assert(self.motor_min_pwm <= motor_speeds[i])
            assert(self.motor_max_pwm >= motor_speeds[i])

        return motor_speeds
