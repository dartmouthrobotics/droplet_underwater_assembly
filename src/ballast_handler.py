import utils
import rospy
import mavros_msgs.srv
import config

class BallastHandler(object):
    def __init__(self):
        self.servo_command_proxy = rospy.ServiceProxy("/mavros/cmd/command", mavros_msgs.srv.CommandLong)
        self.state_fill_with_air = 'FILL_BALLAST_WITH_AIR'
        self.state_empty_ballast_air = 'EMPTY_BALLAST_AIR'
        self.state_neutral = 'NEUTRAL'

        self.current_state = self.state_neutral
        self.entered_state_time = None

    def start_filling_ballast_with_air(self):
        self.current_state = self.state_fill_with_air
        self.entered_state_time = rospy.Time.now()
        self.publish_servo_position(config.BALLAST_AIR_IN_PWM, config.BALLAST_SERVO_INDEX)

    def start_emptying_ballast_air(self):
        self.current_state = self.state_empty_ballast_air
        self.entered_state_time = rospy.Time.now()
        self.publish_servo_position(config.BALLAST_AIR_OUT_PWM, config.BALLAST_SERVO_INDEX)

    def go_to_neutral(self):
        self.current_state = self.state_neutral
        self.entered_state_time = rospy.Time.now()
        self.publish_servo_position(config.BALLAST_AIR_NEUTRAL_PWM, config.BALLAST_SERVO_INDEX)

    def update(self):
        seconds_in_state = (rospy.Time.now() - self.entered_state_time).to_sec()

        if self.current_state == self.state_fill_with_air:
            if seconds_in_state > config.BALLAST_AIR_FILL_TIME_SECONDS:
                self.go_to_neutral()

        elif self.current_state == self.state_empty_ballast_air:
            if seconds_in_state > config.BALLAST_AIR_EMPTY_TIME_SECONDS:
                self.go_to_neutral()

        elif self.current_state == self.state_neutral:
            self.go_to_neutral()


    def fill_ballast_with_air(self):
        self.publish_servo_position(config.FILL_BALLAST_PWM, )

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
