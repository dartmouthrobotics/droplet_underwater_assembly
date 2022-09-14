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

        self.pulse_interval = None
        self.pulsing_state = None
        self.current_pulse_start = None

        self.empty_time = None


    def start_filling_ballast_with_air(self):
        rospy.loginfo("BALLAST Air in valve configuration")
        self.current_state = self.state_fill_with_air
        self.entered_state_time = rospy.Time.now()
        self.publish_servo_position(
            config.BALLAST_SCUBA_VALVE_FULL_OPEN_PWM,
            config.BALLAST_SCUBA_SERVO_INDEX
        )
        self.publish_servo_position(
            config.BALLAST_TANK_OUTLET_CLOSED_PWM,
            config.BALLAST_TANK_OUTLET_SERVO_INDEX
        )


    def start_emptying_ballast_air(self, empty_time=None):
        self.current_state = self.state_empty_ballast_air
        rospy.loginfo("BALLAST Air out valve configuration")

        self.empty_time = config.BALLAST_AIR_EMPTY_TIME_SECONDS
        if empty_time is not None:
            self.empty_time = empty_time

        self.entered_state_time = rospy.Time.now()
        self.publish_servo_position(
            config.BALLAST_SCUBA_VALVE_CLOSED_PWM,
            config.BALLAST_SCUBA_SERVO_INDEX
        )
        self.publish_servo_position(
            config.BALLAST_TANK_OUTLET_FULL_OPEN_PWM,
            config.BALLAST_TANK_OUTLET_SERVO_INDEX
        )


    def go_to_neutral(self):
        self.current_state = self.state_neutral
        self.entered_state_time = rospy.Time.now()
        self.publish_servo_position(
            config.BALLAST_SCUBA_VALVE_CLOSED_PWM,
            config.BALLAST_SCUBA_SERVO_INDEX
        )
        self.publish_servo_position(
            config.BALLAST_TANK_OUTLET_CLOSED_PWM,
            config.BALLAST_TANK_OUTLET_SERVO_INDEX
        )


    def start_pulsing(self, state, neutral_time, on_time):
        # go between the given state and neutral repeatedly
        assert(
            state in [self.state_fill_with_air, self.state_empty_ballast_air]
        )

        self.pulse_interval = on_time
        self.neutral_pulse_time = neutral_time
        self.air_in_pulse_time = on_time
        self.air_out_pulse_time = on_time
        self.pulsing_state = state


    def stop_pulsing(self):
        self.pulse_interval = None
        self.pulsing_state = None
        self.go_to_neutral()


    def update(self):
        if self.pulse_interval is not None:
            if self.current_pulse_start is None:
                self.current_pulse_start = rospy.Time.now()

            if (rospy.Time.now() - self.current_pulse_start).to_sec() > self.pulse_interval:
                self.current_pulse_start = rospy.Time.now()

                if self.current_state == self.pulsing_state:
                    self.pulse_interval = self.neutral_pulse_time
                    self.go_to_neutral()
                elif self.pulsing_state == self.state_fill_with_air:
                    self.pulse_interval = self.air_in_pulse_time
                    self.start_filling_ballast_with_air()
                elif self.pulsing_state == self.state_empty_ballast_air:
                    self.pulse_interval = self.air_out_pulse_time
                    self.start_emptying_ballast_air()
        else:
            if self.entered_state_time is not None:
                seconds_in_state = (rospy.Time.now() - self.entered_state_time).to_sec()

                if self.current_state == self.state_fill_with_air:
                    if seconds_in_state > config.BALLAST_AIR_FILL_TIME_SECONDS:
                        self.go_to_neutral()

                elif self.current_state == self.state_empty_ballast_air:
                    if seconds_in_state > self.empty_time:
                        self.go_to_neutral()

                elif self.current_state == self.state_neutral:
                    self.go_to_neutral()


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
