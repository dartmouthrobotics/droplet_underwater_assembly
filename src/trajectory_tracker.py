import collections
import utils
import rospy
import tf

NUMBER_MOTORS = 8

"""
Motor locations:

Top refers to motors facing up on top of bluerov
Bottom (the ones facing inward on bottom)
Front means towards the camera
Left is to the left if you are looking at the back of the bluerov towards the camera (ie you are behind it)

1: top back right
2: bottom front right
3: top front right
4: top front left
5: bottom front left
6: top back left
7: bottom back left
8: bottom back right
"""


class PIDTracker(object):
    # how should we handle the I-gains?
    # do a reset at zero crossing? No I think not -- maybe a fixed time window?
    def __init__(self, x_p, y_p, yaw_p, x_d, y_d, yaw_d, x_i, y_i, yaw_i, pitch_p, pitch_i, pitch_d, roll_p, roll_i, roll_d, z_p, z_i, z_d):
        self.x_p = x_p
        self.x_i = x_i
        self.x_d = x_d

        self.y_p = y_p
        self.y_i = y_i
        self.y_d = y_d

        self.yaw_p = yaw_p
        self.yaw_i = yaw_i
        self.yaw_d = yaw_d

        self.z_p = z_p
        self.z_i = z_i
        self.z_d = z_d

        self.roll_p = roll_p
        self.roll_i = roll_i
        self.roll_d = roll_d

        self.pitch_p = pitch_p
        self.pitch_i = pitch_i
        self.pitch_d = pitch_d

        self.max_motor_speed = 100

        self.yaw_factor =   [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        self.x_factor =     [0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.y_factor =     [0.0,  0.0, 0.0, 0.0, -1.0, 0.0, -1.0, 0.0]

        self.roll_factor =  [1.0, 0.0, -1.0, -1.0, 0.0, -1.0, 0.0, 0.0]
        self.pitch_factor = [-1.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0]
        self.z_factor =     [1.0, 0.0, -1.0, 1.0, 0.0, 1.0, 0.0, 0.0]

        self.error_history = []
        self.number_error_history_frames = 700

        self.forward_minimum_pwms = [
            18,
            25,
            22,
            20,
            25,
            25,
            20,
            20,
        ]

        self.backward_minimum_pwms = [
            -30,
            -30,
            -28,
            -35,
            -25,
            -25,
            -30,
            -30,
        ]

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
        motor_intensities = [0.0] * 8

        max_intensity = 1.0

        # x,y,yaw intensities
        for i in range(len(motor_intensities)):
            motor_intensities[i] = (thrust_vector[0] * self.x_factor[i] +
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

        for dimension in range(6):
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
            error[5] * self.yaw_p + self.current_velocity[5] * self.yaw_d + self.error_integral[5] * self.yaw_i
        ]


    def get_next_motion_primitive(self):
        return MotionPrimitive("NULL", [1500] * 8, None, 0.0, 0.0)


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


class OpenLoopTracker(PIDTracker):
    # we want to move to another tag, so what does that look like?
    def init(self, **kwargs):
        super(OpenLoopTracker, self).init(
            **kwargs
        )

        self.pulse_on_z_thrust = 0.5
        self.pulse_on_yaw_thrust = 0.5

        self.on_time_ratio = 0.6
        self.cycle_time = 6.0
        self.cycle_start_time = None

    def cycle_is_complete(self):
        if cycle is None:
            return True

        return (rospy.Time.now() - self.cycle_start_time).to_sec() > self.cycle_time

    def should_be_on(self):
        if self.cycle_start_time is None:
            return False

        current_cycle_time = (rospy.Time.now() - self.cycle_start_time).to_sec()

        return current_cycle_time < self.on_time_ratio * self.cycle_time

    def get_next_rc_override(self):
        zrp_thrusts = self.get_zrp_thrust_vector()
        xyyaw_thrusts = [0.0, 0.0, 0.0]

        if not self.cycle_is_complete():
            self.cycle_start_time = rospy.Time.now()

        if self.should_be_on():
            zrp_thrusts[0] = self.pulse_on_z_thrust
        else:
            xyyaw_thrusts[2] = self.pulse_on_yaw_thrust

        thrust_vector = xyyaw_thrusts + zrp_thrusts
        motor_intensities = self.convert_thrust_vector_to_motor_intensities(thrust_vector)

        motor_speeds = self.convert_motor_intensities_to_pwms(motor_intensities)

        for speed in motor_speeds:
            assert((abs(speed) - 1500) <= 150)

        return utils.construct_rc_message(motor_speeds)
        # we want to maintain the roll, yaw correction since we get that every frame easily
        # lets try pulsing up and to the left
