import utils
import rospy

class GripperHandler(object):
    def __init__(self, config):
        self.toggle_time_seconds = config["gripper_config"]["toggle_time_seconds"]
        self.channel = config["gripper_config"]["channel"]
        self.toggle_pwm = config["gripper_config"]["toggle_pwm"]
        self.direction_modifier = config["gripper_config"]["direction_modifier"]

        self.toggle_start_time = None

        # zero means not moving, less than zero means closing, more than zero means opening.
        self.move_direction = 0

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
        if self.toggle_start_time is None and self.move_direction == 0:
            return

        current_time = rospy.Time.now()

        if (current_time - self.toggle_start_time).to_sec() > self.toggle_time_seconds:
            rospy.loginfo("Completed moving gripper.")
            self.move_direction = 0
            self.toggle_start_time = None

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
        rc_override_message.channels[self.channel] = 1500 + (self.direction_modifier * self.move_direction * self.toggle_pwm)

    def close_gripper_blocking(self, rc_override_publisher):
        self.move_gripper_blocking(rc_override_publisher, -1)

    def open_gripper_blocking(self, rc_override_publisher):
        self.move_gripper_blocking(rc_override_publisher, 1)
