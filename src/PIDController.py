import collections
import utils
import rospy
import tf

class PIDController(object):
    # how should we handle the I-gains?
    # do a reset at zero crossing? No I think not -- maybe a fixed time window?
    def set_gains_from_config(self, gains_json):
        self.x_p = gains_json["x_p"]
        self.x_i = gains_json["x_i"]
        self.x_d = gains_json["x_d"]
        self.y_p = gains_json["y_p"]
        self.y_i = gains_json["y_i"]
        self.y_d = gains_json["y_d"]
        self.yaw_p = gains_json["yaw_p"]
        self.yaw_i = gains_json["yaw_i"]
        self.yaw_d = gains_json["yaw_d"]
        self.z_p = gains_json["z_p"]
        self.z_i = gains_json["z_i"]
        self.z_d = gains_json["z_d"]
        self.roll_p = gains_json["roll_p"]
        self.roll_i = gains_json["roll_i"]
        self.roll_d = gains_json["roll_d"]
        self.pitch_p = gains_json["pitch_p"]
        self.pitch_i = gains_json["pitch_i"]
        self.pitch_d = gains_json["pitch_d"]
        

    @classmethod
    def from_config(cls, config, gains_selector):
        controller = PIDController()

        controller.set_gains_from_config(config["controller_config"][gains_selector])

        controller.x_factor = config["conftroller_config"]["x_factor"]
        controller.y_factor = config["conftroller_config"]["x_factor"]
        controller.z_factor = config["conftroller_config"]["x_factor"]
        controller.roll_factor = config["conftroller_config"]["x_factor"]
        controller.pitch_factor = config["conftroller_config"]["x_factor"]
        controller.yaw_factor = config["conftroller_config"]["x_factor"]

        controller.forward_minimum_pwms = config["controller_config"]["forward_minimum_pwms"]
        controller.backward_minimum_pwms = config["controller_config"]["backward_minimum_pwms"]
        controller.max_motor_speed = config["controller_config"]["max_motor_speeds"]

        return controller

    def __init__(self):
        self.number_motors = 8
        self.current_position = None

        self.error_integral = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.last_position_update_time = None
        self.latest_imu_reading = None

    def convert_thrust_vector_to_motor_intensities(self, thrust_vector):
        # thrust vector is: x,y,yaw,z,roll,pitch
        motor_intensities = [0.0] * self.number_motors

        max_intensity = 1.0

        # x,y,yaw intensities
        for i in range(len(motor_intensities)):
            motor_intensities[i] = (
                thrust_vector[0] * self.x_factor[i] +
                thrust_vector[1] * self.y_factor[i] +
                thrust_vector[2] * self.yaw_factor[i] +
                thrust_vector[3] * self.z_factor[i] +
                thrust_vector[4] * self.roll_factor[i] +
                thrust_vector[5] * self.pitch_factor[i]
            )

            if abs(motor_intensities[i]) > max_intensity:
                max_intensity = abs(motor_intensities[i])

        normalized_intensities = [val / max_intensity for val in motor_intensities]

        for intensity in normalized_intensities:
            assert(abs(intensity) <= 1.0)

        return normalized_intensities

    def convert_motor_intensities_to_pwms(self, intensities):
        offset_motor_speeds = [
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500,
            1500
        ]

        for i in range(len(offset_motor_speeds)):
            offset = 0

            if intensities[i] < 0:
                offset = self.backward_minimum_pwms[i] 
            elif intensities[i] > 0:
                offset = self.forward_minimum_pwms[i]

            offset_motor_speeds[i] = offset_motor_speeds[i] + (intensities[i] * self.max_motor_speed) + offset

        return offset_motor_speeds


    def update_error_integrals(self, next_position):
        next_error = utils.get_error(next_position, self.goal_position)
        seconds_since_last_update = (rospy.Time.now() - self.last_position_update_time).to_sec()

        for dimension in range(3):
            self.error_integral[dimension] = (next_error[dimension] * seconds_since_last_update) + self.error_integral[dimension]


    def clear_error_integrals(self):
        self.error_integral = [0.0] * len(self.error_integral)
        self.error_history = []


    def set_current_position(self, position):
        if self.last_position_update_time is not None:
            self.update_error_integrals(position)

        self.last_position_update_time = rospy.Time.now()
        self.current_position = position


    def set_current_velocity(self, velocity):
        self.current_velocity = velocity


    def set_goal_position(self, goal):
        self.goal_position = goal


    def get_error(self):
        return utils.get_error(self.current_position, self.goal_position)


    def get_xyyaw_thrust_vector(self):
        error = self.get_error()

        return [
            error[0] * self.x_p + self.current_velocity[0] * self.x_d + self.error_integral[0] * self.x_i,
            error[1] * self.y_p + self.current_velocity[1] * self.y_d + self.error_integral[1] * self.y_i,
            error[5] * self.yaw_p + self.current_velocity[5] * self.yaw_d
        ]


    def set_latest_imu_reading(self, latest_imu):
        self.latest_imu_reading = latest_imu


    def get_angle_error_from_imu_reading(self):
        imu_orientation = [
            self.latest_imu_reading.orientation.x,
            self.latest_imu_reading.orientation.y,
            self.latest_imu_reading.orientation.z,
            self.latest_imu_reading.orientation.w
        ]

        roll, pitch, _ = tf.transformations.euler_from_quaternion(imu_orientation)

        return utils.angle_error_rads(roll, 0.0), utils.angle_error_rads(pitch, 0.0)


    def get_zrp_thrust_vector(self):
        error = self.get_error()

        z_thrust = (error[2] * self.z_p) + (self.current_velocity[2] * self.z_d) + (self.error_integral[2] * self.z_i)

        if self.latest_imu_reading is not None:
            roll_error, pitch_error = self.get_angle_error_from_imu_reading()

            roll_velocity = self.latest_imu_reading.angular_velocity.x
            pitch_velocity = self.latest_imu_reading.angular_velocity.y

            return [
                z_thrust,
                self.roll_p * roll_error + self.roll_d * roll_velocity,
                self.pitch_p * pitch_error + self.pitch_d * pitch_velocity
            ]

        return [
            z_thrust,
            0.0,
            0.0,
        ]


    def get_next_rc_override(self):
        xyyaw_thrusts = self.get_xyyaw_thrust_vector()
        zrp_thrusts = self.get_zrp_thrust_vector()

        thrust_vector = xyyaw_thrusts + zrp_thrusts

        motor_intensities = self.convert_thrust_vector_to_motor_intensities(thrust_vector)
        motor_speeds = self.convert_motor_intensities_to_pwms(motor_intensities)

        for speed in motor_speeds:
            assert((abs(speed) - 1500) <= 150)

        return utils.construct_rc_message(motor_speeds)
