import collections
import utils
import rospy
import tf

NUMBER_MOTORS = 8

# feeding z_d into this stuff?
# would that really help?
# why was it so unstable previously?

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
        self.z_override = 0.0

        #self.yaw_factor =   [0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.yaw_factor =   [-1.0, 1.0, 0.0, 0.0, 0.0, -1.0, -1.0, 0.0]
        self.x_factor =     [-1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0]
        self.y_factor =     [-1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0]

        self.roll_factor =  [0.0, 0.0, 1.0, 1.0, -1.0, 0.0, 0.0, -1.0]
        self.pitch_factor = [0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0]
        self.z_factor =     [0.0, 0.0, -1.0, 1.0, 1.0, 0.0, 0.0, -1.0]

        self.lateral_motors = [0, 1, 5, 6]
        self.updown_motors =  [2, 3, 4, 7]
        self.max_motor_speed_lateral = 150
        self.max_motor_speed_updown = 300

        self.error_history = []
        self.number_error_history_frames = 700

        self.forward_minimum_pwms = [
            20,
            20,
            20,
            20,
            20,
            25,
            20,
            20,
        ]

        self.backward_minimum_pwms = [
            -30,
            -35,
            -30,
            -35,
            -30,
            -30,
            -35,
            -35,
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

        self.goal_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for lateral_motor in self.lateral_motors:
            if lateral_motor in self.updown_motors:
                raise Exception("A motor cannot be assigned to both the lateral and up/down set!")

        for i in range(8):
            if i not in self.lateral_motors and i not in self.updown_motors:
                raise Exception("Motor {} is not assigned as either lateral or up/down!".format(i))

        if len(self.lateral_motors) + len(self.updown_motors) != 8:
            raise Exception("The total number of motors assigned to either lateral or up/down must be 8!")


    def set_z_override(self, override):
        self.z_override = override


    def normalize_intensities(self, intensities):
        lateral_intensities = [intensities[i] for i in self.lateral_motors]
        updown_intensities = [intensities[i] for i in self.updown_motors]

        max_lateral_intensity = max(1.0, max(map(abs, lateral_intensities)))
        max_updown_intensity = max(1.0, max(map(abs, updown_intensities)))

        normalized_lateral = [val / max_lateral_intensity for val in lateral_intensities]
        normalized_updown = [val / max_updown_intensity for val in updown_intensities]

        combined = [0.0] * 8

        for i, lateral_motor in enumerate(self.lateral_motors):
            combined[lateral_motor] = normalized_lateral[i] 

        for i, updown_motor in enumerate(self.updown_motors):
            combined[updown_motor] = normalized_updown[i] 
        return combined


    def convert_thrust_vector_to_motor_intensities(self, thrust_vector):
        # thrust vector is: x,y,yaw,z,roll,pitch
        motor_intensities = [0.0] * 8

        # x,y,yaw intensities
        for i in range(len(motor_intensities)):
            motor_intensities[i] = (thrust_vector[0] * self.x_factor[i] +
                thrust_vector[1] * self.y_factor[i] +
                thrust_vector[2] * self.yaw_factor[i] +
                thrust_vector[3] * self.z_factor[i] +
                thrust_vector[4] * self.roll_factor[i] +
                thrust_vector[5] * self.pitch_factor[i]
            )

        normalized_intensities = self.normalize_intensities(motor_intensities)

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

            max_motor_speed = self.max_motor_speed_lateral

            if i in self.updown_motors:
                max_motor_speed = self.max_motor_speed_updown

            offset_motor_speeds[i] = offset_motor_speeds[i] + (intensities[i] * max_motor_speed) + offset

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

        _, roll, pitch = tf.transformations.euler_from_quaternion(imu_orientation, axes='szyx')

        return utils.angle_error_rads(roll, 0.0), utils.angle_error_rads(pitch, 0.0)


    def get_zrp_thrust_vector(self):
        error = self.get_error()

        if self.z_override != 0.0:
            error[2] = self.z_override

        z_thrust = (error[2] * self.z_p) + (self.current_velocity[2] * self.z_d) + (self.error_integral[2] * self.z_i)

        if self.latest_imu_reading is not None:
            roll_error, pitch_error = self.get_angle_error_from_imu_reading()

            roll_velocity = self.latest_imu_reading.angular_velocity.y # weird correspondance
            pitch_velocity = self.latest_imu_reading.angular_velocity.x

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
            assert((abs(speed) - 1500) <= 400)

        return utils.construct_rc_message(motor_speeds)


class OpenLoopTracker(PIDTracker):
    # we want to move to another tag, so what does that look like?
    def __init__(self, **kwargs):
        super(OpenLoopTracker, self).__init__(
            **kwargs
        )
        self.pulse_on_z_thrust = 0.6
        self.pulse_on_yaw_thrust = -0.5

        self.on_time_ratio = 0.7
        self.cycle_time = 2.0
        self.cycle_start_time = None

    def cycle_is_complete(self):
        if self.cycle_start_time is None:
            return True

        return (rospy.Time.now() - self.cycle_start_time).to_sec() > self.cycle_time

    def should_be_on(self):
        if self.cycle_start_time is None:
            return False

        current_cycle_time = (rospy.Time.now() - self.cycle_start_time).to_sec()

        return current_cycle_time < self.on_time_ratio * self.cycle_time

    def get_next_rc_override(self):
        zrp_thrusts = [0.0, 0.0, 0.0]
        xyyaw_thrusts = [0.0, 0.0, 0.0]

        if self.cycle_is_complete():
            self.cycle_start_time = rospy.Time.now()

        if self.should_be_on():
            zrp_thrusts[0] = self.pulse_on_z_thrust
            xyyaw_thrusts[2] = self.pulse_on_yaw_thrust

        thrust_vector = xyyaw_thrusts + zrp_thrusts

        motor_intensities = self.convert_thrust_vector_to_motor_intensities(thrust_vector)

        motor_speeds = self.convert_motor_intensities_to_pwms(motor_intensities)

        for speed in motor_speeds:
            assert((abs(speed) - 1500) <= 400)

        return utils.construct_rc_message(motor_speeds)
        # we want to maintain the roll, yaw correction since we get that every frame easily
        # lets try pulsing up and to the left


class BinaryPController(PIDTracker):
    def __init__(self, **kwargs):
        super(BinaryPController, self).__init__(**kwargs)
        self.x_d = 0.0
        self.y_d = 0.0
        self.z_d = 0.0
        self.yaw_d = 0.0

        self.x_i = 0.0
        self.y_i = 0.0
        self.z_i = 0.0
        self.yaw_i = 0.0

        self.xyz_on_thrust = 0.4
        self.yaw_on_thrust = 0.2

    def get_next_rc_override(self):
        zrp_thrusts = self.get_zrp_thrust_vector()
        zrp_thrusts[2] = 0.0
        zrp_thrusts[1] = 0.0

        xyyaw_thrusts = self.aw_thrust_vector()

        thrust_vector = xyyaw_thrusts + zrp_thrusts

        for (index, thrust) in enumerate(thrust_vector):
            # ignore roll and pitch in this
            if index in (3,4):
                continue

            on_thrust = self.xyz_on_thrust
            if index == 5:
                on_thrust = self.yaw_on_thrust

            if thrust > 0.0:
                thrust_vector[index] = on_thrust
            if thrust < 0.0:
                thrust_vector[index] = -on_thrust

        motor_intensities = self.convert_thrust_vector_to_motor_intensities(thrust_vector)

        motor_speeds = self.convert_motor_intensities_to_pwms(motor_intensities)

        for speed in motor_speeds:
            assert((abs(speed) - 1500) <= 400)

        return utils.construct_rc_message(motor_speeds)
