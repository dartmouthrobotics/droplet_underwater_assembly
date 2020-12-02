import rospy
import config

class AssemblyAction(object):
    def __init__(self, action_type, goal_pose, pose_tolerance, position_hold_time=6.0, **kwargs):
        self.valid_types = ['move', 'open_gripper', 'close_gripper', 'move_wrist', 'change_platforms']
        self.action_type = action_type
        self.goal_pose = goal_pose
        self.start_time = None
        self.reached_goal_time = None

        self.position_hold_time = position_hold_time

        self.gripper_hold_time = config.GRIPPER_HOLD_TIME
        self.pose_tolerance = pose_tolerance
        self.gripper_handler = None

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

        return "Move to: {}".format(self.goal_pose)

    def start(self):
        self.start_time = rospy.Time.now()

    @property
    def is_started(self):
        return self.start_time is not None

    def is_complete(self, pose_error, **kwargs):
        if self.start_time is None:
            rospy.logerr("Cannot complete an action that hasn't been started.")
            return False

        if self.action_type == 'move':
            if self.reached_goal_time is None:
                reached_goal = all([abs(error) < tolerance for (error, tolerance) in zip(pose_error, self.pose_tolerance)])

                if reached_goal:
                    self.reached_goal_time = rospy.Time.now()

                return False
            else:
                return (rospy.Time.now() - self.reached_goal_time).to_sec() > self.position_hold_time

        if self.action_type == 'open_gripper' or self.action_type == 'close_gripper':
            elapsed_seconds = (rospy.Time.now() - self.start_time).to_sec()
            complete = elapsed_seconds > self.gripper_handler.toggle_time_seconds + self.gripper_hold_time

            return complete

        if self.action_type == 'change_platforms':
            return kwargs['last_tracked_marker_time'] is not None and (rospy.Time.now() - kwargs['last_tracked_marker_time']).to_sec() < 0.5

        raise Exception("Unrecognized action type!")
