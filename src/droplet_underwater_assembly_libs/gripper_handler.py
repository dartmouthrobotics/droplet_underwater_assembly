import utils
import rospy
import mavros_msgs.srv
import config

class GripperHandler(object):
    def __init__(self):
        self.move_direction = 0

        self.finger_servo_speed = config.GRIPPER_STOP_PWM 
        self.servo_command_proxy = rospy.ServiceProxy("/mavros/cmd/command", mavros_msgs.srv.CommandLong)
        self.motion_started_time = None

    def start_opening_fingers(self):
        if self.is_opening or self.is_closing:
            self.publish_servo_position(
                config.GRIPPER_STOP_PWM,
                config.FINGER_SERVO_INDEX
            )
            raise Exception("Cannot open a non-stopped gripper!")

        self.finger_servo_speed = config.GRIPPER_OPEN_PWM
        self.motion_started_time = rospy.Time.now()
        self.publish_servo_position(
            self.finger_servo_speed,
            config.FINGER_SERVO_INDEX
        )

    def start_closing_fingers(self):
        if self.is_opening or self.is_closing:
            self.publish_servo_position(
                config.GRIPPER_STOP_PWM,
                config.FINGER_SERVO_INDEX
            )
            raise Exception("Cannot close a non-stopped gripper!")

        self.finger_servo_speed = config.GRIPPER_CLOSE_PWM
        self.motion_started_time = rospy.Time.now()
        self.publish_servo_position(
            self.finger_servo_speed,
            config.FINGER_SERVO_INDEX
        )

    def stop(self):
        self.finger_servo_speed = config.GRIPPER_STOP_PWM
        self.motion_started_time = None
        self.publish_servo_position(
            self.finger_servo_speed,
            config.FINGER_SERVO_INDEX
        )

    @property
    def is_opening(self):
        return self.finger_servo_speed == config.GRIPPER_OPEN_PWM

    @property
    def is_closing(self):
        return self.finger_servo_speed == config.GRIPPER_CLOSE_PWM

    def update(self):
        if self.is_opening:
            if (rospy.Time.now() - self.motion_started_time).to_sec() > config.GRIPPER_OPEN_TIME:
                self.motion_start_time = None
                self.finger_servo_speed = config.GRIPPER_STOP_PWM
                self.publish_servo_position(
                    self.finger_servo_speed,
                    config.FINGER_SERVO_INDEX
                )

        if self.is_closing:
            if (rospy.Time.now() - self.motion_started_time).to_sec() > config.GRIPPER_CLOSE_TIME:
                self.motion_start_time = None
                self.finger_servo_speed = config.GRIPPER_STOP_PWM
                self.publish_servo_position(
                    self.finger_servo_speed,
                    config.FINGER_SERVO_INDEX
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
