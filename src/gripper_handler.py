import utils
import rospy
import mavros_msgs.srv
import config

class GripperHandler(object):
    def __init__(self):
        self.toggle_time_seconds = 3.0 
        self.channel = 8
        self.toggle_pwm = 50
        self.close_time_addon = 0.5
        self.rotation_rate = 1 # 1 pwm per frame

        self.toggle_start_time = None

        # zero means not moving, less than zero means closing, more than zero means opening.
        self.move_direction = 0

        self.wrist_rotation_increment_per_frame_pwm = 2

        # gripper starts aligned with robot body
        self.current_rotation_position = config.GRIPPER_ROTATION_INITIAL
        self.desired_rotation_position = config.GRIPPER_ROTATION_INITIAL

        self.gripper_rotation_service_proxy = rospy.ServiceProxy("/mavros/cmd/command", mavros_msgs.srv.CommandLong)

        self.wrist_min_pwm = 900
        self.wrist_max_pwm = 1600
        self.wrist_update_pause_seconds = 0.1
        self.last_wrist_update_time = None

    def rotate_to_position(self, rotation_position):
        self.desired_rotation_position = rotation_position

    @property
    def is_toggling(self):
        return self.toggle_start_time is not None

    def move_gripper_blocking(self, rc_override_publisher, direction):
        if direction == 1:
            self.start_opening()
        elif direction == -1:
            self.start_closing()
        else:
            raise Exception("Inalid gripper direction!")

        poll_rate = rospy.Rate(1)
        while self.is_toggling:
            self.update()
            move_gripper_message = utils.construct_stop_rc_message()
            self.mix_into_rc_override_message(move_gripper_message)

            for channel in move_gripper_message.channels:
                assert(abs(1500 - channel) < 100)

            rc_override_publisher.publish(move_gripper_message)
            poll_rate.sleep()

    def update(self):
        if self.desired_rotation_position is not None:
            time_since_last_wrist_update = float("inf")

            if self.last_wrist_update_time is not None:
                time_since_last_wrist_update = (rospy.Time.now() - self.last_wrist_update_time).to_sec()

            if time_since_last_wrist_update > self.wrist_update_pause_seconds:
                if self.desired_rotation_position < self.current_rotation_position:
                    self.current_rotation_position = self.current_rotation_position - self.wrist_rotation_increment_per_frame_pwm
                    self.publish_gripper_rotation_position(self.current_rotation_position)
                    self.last_wrist_update_time = rospy.Time.now()

                elif self.desired_rotation_position > self.current_rotation_position:
                    self.current_rotation_position = self.current_rotation_position + self.wrist_rotation_increment_per_frame_pwm
                    self.publish_gripper_rotation_position(self.current_rotation_position)
                    self.last_wrist_update_time = rospy.Time.now()

        if self.toggle_start_time is None and self.move_direction == 0:
            return

        current_time = rospy.Time.now()

        toggle_time_addon = 0.0
        if self.move_direction == -1:
            toggle_time_addon = self.close_time_addon

        if (current_time - self.toggle_start_time).to_sec() > self.toggle_time_seconds + toggle_time_addon:
            rospy.loginfo("Completed moving gripper.")
            self.move_direction = 0
            self.toggle_start_time = None

    def publish_gripper_rotation_position(self, position):
        if self.current_rotation_position < config.MIN_WRIST_PWM or self.current_rotation_position > config.MAX_WRIST_PWM:
            raise Exception("Error attempting to rotate to an invalid wrist pwm: {}. Valid range is: ({},{})".format(self.current_rotation_position, config.MIN_WRIST_PWM, config.MAX_WRIST_PWM))

        self.gripper_rotation_service_proxy(
            broadcast=False,
            command=183,
            confirmation=0,
            param1=0,
            param2=self.current_rotation_position
        )

    def start_opening(self):
        if self.is_toggling:
            rospy.logwarn("Cannot start opening the gripper. It is already moving!")
            return

        self.toggle_start_time = rospy.Time.now()
        self.move_direction = 1

    def start_closing(self):
        if self.is_toggling:
            rospy.logwarn("Cannot start closing the gripper. It is already moving!")
            return

        self.toggle_start_time = rospy.Time.now()
        self.move_direction = -1

    def mix_into_rc_override_message(self, rc_override_message):
        rc_override_message.channels[self.channel] = 1500 + (self.move_direction * self.toggle_pwm) # the new chip I soldered reverses the meaning of the sign

    def close_gripper_blocking(self, rc_override_publisher):
        self.move_gripper_blocking(rc_override_publisher, -1)

    def open_gripper_blocking(self, rc_override_publisher):
        self.move_gripper_blocking(rc_override_publisher, 1)
