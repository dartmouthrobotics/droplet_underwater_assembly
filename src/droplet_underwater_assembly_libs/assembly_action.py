import rospy
import config
import numpy

class AssemblyAction(object):
    @classmethod
    def construct_change_platforms(cls, platform_id):
        return cls('change_platforms', [0.0] * 6, [1.0] * 6, to_platform_id=platform_id)

    @classmethod
    def construct_inflate_ballast(cls):
        return cls(
            'inflate_ballast',
            None,
            None
        )

    @classmethod
    def construct_deflate_ballast(cls):
        return cls(
            'deflate_ballast',
            None,
            None
        )

    @classmethod
    def construct_rotate_wrist(cls, pwm, pose):
        if pwm > config.MAX_SERVO_PWM or pwm < config.MIN_SERVO_PWM:
            raise Exception(
                "Given wrist rotation {pwm} is out of the valid range: ({min}, {max})".format(
                    pwm=pwm,
                    min=config.MIN_WRIST_PWM,
                    max=config.MAX_WRIST_PWM
                )
            )

        return cls('move_wrist', pose, [1.0] * 6, wrist_rotation_pwm=pwm)

    @classmethod
    def construct_move_left_right(cls, x, y):
        return cls(
            action_type='left_right_move', 
            goal_pose=[0.0]*6,
            pose_tolerance=config.ULTRA_COARSE_POSE_TOLERANCE,
            goal_x=x,
            goal_y=y
        )

    def __init__(self, action_type, goal_pose, pose_tolerance, position_hold_time=config.DEFAULT_POSITION_HOLD_TIME, **kwargs):
        self.valid_types = [
            'move',
            'open_gripper',
            'close_gripper',
            'move_wrist',
            'change_platforms',
            'binary_P_move',
            'hold',
            'inflate_ballast',
            'deflate_ballast',
            'change_buoyancy',
            'left_right_move',
            'bailing_release'
        ]

        self.action_type = action_type
        self.goal_pose = goal_pose
        self.start_time = None
        self.reached_goal_time = None
        self.high_level_build_step = None
        self.forced_complete = False

        self.position_hold_time = position_hold_time

        self.gripper_hold_time = config.GRIPPER_HOLD_TIME
        self.pose_tolerance = pose_tolerance
        self.gripper_handler = None
        self.ballast_handler = None

        self.sequence_id = 0

        self.timeout = None
        self.ballast_change_t_level = None
        self.ballast_change_direction = None

        if self.action_type == 'left_right_move':
            try:
                self.goal_x = kwargs['goal_x']
                self.goal_y = kwargs['goal_y']
            except:
                raise Exception("Invalid kwargs supplied to the move_left_right_action!")

        if self.action_type == 'bailing_release':
            self.bail_time = kwargs['bail_time']
            self.thrust_down_amount = kwargs['thrust_down_amount']
            self.pre_open_time = kwargs['pre_open_time']

        if 't_level_from_planner' in kwargs:
            self.ballast_change_t_level = kwargs['t_level_from_planner']

        if 'ballast_change_direction' in kwargs:
            self.ballast_change_direction = kwargs['ballast_change_direction']

        if self.action_type == 'change_buoyancy':
            assert(self.ballast_change_t_level is not None)
            assert(self.ballast_change_direction is not None)
            assert(self.ballast_change_direction in [-1, 1])
            assert(0.0 <= self.ballast_change_t_level  <= 1.0)

        if self.action_type == 'move_wrist':
            if 'wrist_rotation_pwm' not in kwargs:
                raise Exception("The wrist_rotation_pwm kwarg must be provided for the 'move_wrist' action.")
            else:
                self.wrist_rotation_pwm = kwargs['wrist_rotation_pwm']

        if 'to_platform_id' in kwargs:
            self.to_platform_id = kwargs['to_platform_id']

        else:
            if self.action_type == 'change_platforms':
                raise Exception("Destination platform must be provided for the change platforms action")

        assert(self.action_type in self.valid_types)

        if self.action_type != 'change_platforms':
            assert(len(self.goal_pose) == 6)

    def __str__(self):
        if self.action_type == "open_gripper" or self.action_type == "close_gripper":

            return "Action type: {}".format(self.action_type)

        if self.action_type == "change_platforms":
            return "Changing to platform {}".format(self.to_platform_id)

        if self.action_type == "move":
            return "Move to: {}".format(self.goal_pose)

        if self.action_type == "move_wrist":
            return "Rotate wrist to {}".format(self.wrist_rotation_pwm)

        if self.action_type == "binary_P_move":
            return "Coarse grained recenter to {}".format(self.goal_pose)

        if self.action_type == "hold":
            return "Hold for {} seconds".format(self.position_hold_time)

        if self.action_type == "left_right_move":
            return "Left right move to [{}, {}]".format(self.goal_x, self.goal_y)

        if self.action_type == 'inflate_ballast':
            return "Inflate ballast tanks"

        if self.action_type == 'deflate_ballast':
            return "Deflate ballast tanks"

        if self.action_type == 'bailing_release':
            return "Bailing release"

        if self.action_type == 'change_buoyancy':
            # can we do fully deflating or inflating
            # what about 
            return "Change buoyancy direction: {}. tlevel {}".format(
                self.ballast_change_direction,
                self.ballast_change_t_level
            )

        else:
            raise Exception("__str__ not implemented for assembly actions of type {}".format(self.action_type))

    def start(self):
        self.start_time = rospy.Time.now()

    @property
    def is_started(self):
        return self.start_time is not None

    def is_complete(self, pose_error, **kwargs):
        if self.start_time is None:
            rospy.logerr("Cannot complete an action that hasn't been started.")
            return False

        if self.timeout is not None and (rospy.Time.now() - self.start_time).to_sec() > self.timeout:
            rospy.logerr("Action timed out!")
            return True

        if self.forced_complete:
            return True

        if self.action_type == 'move':
            if self.reached_goal_time is None:
                reached_goal = all([abs(error) < tolerance for (error, tolerance) in zip(pose_error, self.pose_tolerance)])

                if reached_goal:
                    self.reached_goal_time = rospy.Time.now()

                return False
            else:
                return (rospy.Time.now() - self.reached_goal_time).to_sec() > self.position_hold_time

        if self.action_type == 'change_buoyancy':
            if self.ballast_change_direction > 0:
                # going up
                return pose_error[2] < 0
            if self.ballast_change_direction < 0:
                # going down
                return pose_error[2] > 0

        if self.action_type == 'open_gripper' or self.action_type == 'close_gripper':
            #elapsed_seconds = (rospy.Time.now() - self.start_time).to_sec()
            #complete = elapsed_seconds > self.gripper_handler.toggle_time_seconds + self.gripper_hold_time
            complete = not self.gripper_handler.is_opening and not self.gripper_handler.is_closing

            return complete

        if self.action_type == 'change_platforms':
            return kwargs['last_tracked_marker_time'] is not None and (rospy.Time.now() - kwargs['last_tracked_marker_time']).to_sec() < 0.5

        if self.action_type == 'binary_P_move' or self.action_type == 'left_right_move':
            reached_goal = all([abs(error) < tolerance for (error, tolerance) in zip(pose_error, self.pose_tolerance)])
            return reached_goal

        if self.action_type == 'hold':
            return (rospy.Time.now() - self.start_time).to_sec() > self.position_hold_time

        if self.action_type == 'bailing_release':
            return (rospy.Time.now() - self.start_time).to_sec() > max(config.GRIPPER_OPEN_TIME + self.pre_open_time, self.bail_time)

        if self.action_type == 'move_wrist':
            if self.gripper_handler.desired_rotation_position is None:
                return False

            return self.gripper_handler.desired_rotation_position == self.gripper_handler.current_rotation_position

        if self.action_type == 'inflate_ballast':
            return self.is_started and self.ballast_handler.current_state == self.ballast_handler.state_neutral

        if self.action_type == 'deflate_ballast':
            return self.is_started and self.ballast_handler.current_state == self.ballast_handler.state_neutral

        raise Exception("Unrecognized action type!")
