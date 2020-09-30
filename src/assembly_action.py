import rospy

class AssemblyAction(object):
    def __init__(self, action_type, goal_pose, pose_tolerance):
        self.valid_types = ['move', 'open_gripper', 'close_gripper']
        self.action_type = action_type
        self.goal_pose = goal_pose
        self.start_time = None
        self.reached_goal_time = None
        self.position_hold_time = 6.0
        self.gripper_hold_time = 2.5
        self.pose_tolerance = pose_tolerance
        self.gripper_handler = None

        assert(self.action_type in self.valid_types)
        assert(len(self.goal_pose) == 6)

    def __str__(self):
        if self.action_type == "open_gripper" or self.action_type == "close_gripper":

            return "Action type: {}".format(self.action_type)

        return "Move to: {}".format(self.goal_pose)

    def start(self):
        self.start_time = rospy.Time.now()

    @property
    def is_started(self):
        return self.start_time is not None

    def move_is_complete(self, pose_error):
        if self.reached_goal_time is None:
            reached_goal = all([abs(error) < tolerance for (error, tolerance) in zip(pose_error, self.pose_tolerance)])

            if reached_goal:
                self.reached_goal_time = rospy.Time.now()

            return False
        else:
            return (rospy.Time.now() - self.reached_goal_time).to_sec() > self.position_hold_time

    def gripper_action_is_complete(self):
        elapsed_seconds = (rospy.Time.now() - self.start_time).to_sec()
        complete = elapsed_seconds > self.gripper_handler.toggle_time_seconds + self.gripper_hold_time

        return complete
        

    def is_complete(self, pose_error):
        if self.start_time is None:
            rospy.logerr("Cannot complete an action that hasn't been started.")
            return False

        if self.action_type == 'move':
            return self.move_is_complete(pose_error)

        if self.action_type == 'open_gripper' or self.action_type == 'close_gripper':
            return self.gripper_action_is_complete(pose_error)

        raise Exception("Unrecognized action type!")
