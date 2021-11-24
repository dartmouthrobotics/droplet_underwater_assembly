import utils
import rospy
import mavros_msgs.srv
import config

class GripperHandler(object):
    def __init__(self):
        self.move_direction = 0

        self.current_rotation_position = 1500
        self.desired_rotation_position = config.WRIST_ROTATION_INITIAL

        self.desired_finger_position = config.FINGER_POSITION_INITIAL
        self.current_finger_position = config.FINGER_POSITION_INITIAL

        self.servo_command_proxy = rospy.ServiceProxy("/mavros/cmd/command", mavros_msgs.srv.CommandLong)

        self.last_wrist_update_time = None
        self.last_fingers_update_time = None
        self.tick_rate = 0.05

    def rotate_to_position(self, rotation_position):
        self.desired_rotation_position = rotation_position

    def start_opening_fingers(self):
        self.desired_finger_position = config.FINGER_OPEN_POSITION

    def start_closing_fingers(self):
        self.desired_finger_position = config.FINGER_CLOSED_POSITION

    def move_servo_one_tick(self, desired_position, current_position, last_update_time, servo_index, move_rate):
            if last_update_time is not None:
                time_since_last_update = (rospy.Time.now() - last_update_time).to_sec()
            else:
                time_since_last_update = float("inf")

            next_position = current_position
            updated = False

            update_time = last_update_time
            if time_since_last_update > self.tick_rate:
                if current_position < desired_position:
                    next_position = min(next_position + move_rate, desired_position)
                    updated = True

                elif current_position > desired_position:
                    next_position = max(next_position - move_rate, desired_position)
                    updated = True

                update_time = rospy.Time.now()

            if updated:
                self.publish_servo_position(servo_index=servo_index, position=next_position)

            return next_position, update_time


    def update(self):
        self.current_rotation_position, self.last_wrist_update_time = self.move_servo_one_tick(
            desired_position=self.desired_rotation_position,
            current_position=self.current_rotation_position,
            last_update_time=self.last_wrist_update_time,
            servo_index=config.WRIST_SERVO_INDEX,
            move_rate=config.WRIST_ROTATION_RATE_PWM_PER_TICK
        )

        # open instantly but close slowly
        finger_move_rate = config.FINGER_CLOSE_RATE_PWM_PER_TICK
        if self.current_finger_position < self.desired_finger_position:
            finger_move_rate = config.FINGER_OPEN_RATE_PWM_PER_TICK

        self.current_finger_position, self.last_fingers_update_time = self.move_servo_one_tick(
            desired_position=self.desired_finger_position,
            current_position=self.current_finger_position,
            last_update_time=self.last_fingers_update_time,
            servo_index=config.FINGER_SERVO_INDEX,
            move_rate=finger_move_rate
        )

    def publish_servo_position(self, position, servo_index):
        if position < config.MIN_SERVO_PWM or position > config.MAX_SERVO_PWM:
            raise Exception("Invalid servo position given! Range is {}-{} given position is {}".format(
                config.MIN_SERVO_PWM,
                config.MAX_SERVO_PWM,
                position
            ))

        self.servo_command_proxy(
            broadcast=False,
            command=config.MAV_CMD_DO_SET_SERVO,
            confirmation=0,
            param1=servo_index,
            param2=position
        )

    def close_gripper_blocking(self, rc_override_publisher):
        self.move_gripper_blocking(rc_override_publisher, -1)

    def open_gripper_blocking(self, rc_override_publisher):
        self.move_gripper_blocking(rc_override_publisher, 1)
