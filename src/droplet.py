import trajectory_tracker
import gripper_handler
import utils
import deque
import sys
import rospy
import tf
import datetime


class Droplet(object):
    def __init__(self, config):
        self.times_tracked_marker_seen = 0
        self.tracked_marker_id = 0

        self.running_build_plan = False
        self.latest_marker_message = None

        self.latest_velocity = None
        self.velocity_history = []

        self.block_held_gains = config["controller_config"]["block_held_gains"]
        self.no_block_gains =   config["controller_config"]["no_block_gains"]
        self.main_loop_hz =   config["main_loop_rate"]
        self.main_loop_rate = rospy.Rate(self.main_loop_hz) 

        self.number_marker_readings_before_start = config["number_marker_readings_before_start"]

        self.marker_subscriber = rospy.Subscriber(
            "/bluerov_controller/ar_tag_detector",
            stag_ros.msg.StagMarkers,
            marker_callback,
            queue_size=1
        )

        self.imu_subscriber = rospy.Subscriber(
            "/imu/data",
            sensor_msgs.msg.Imu,
            imu_callback,
            queue_size=1
        )

        self.rc_override_publisher = rospy.Publisher(
            "/mavros/rc/override",
            mavros_msgs.msg.OverrideRCIn,
            queue_size=1
        )

        self.goal_pose_publisher = rospy.Publisher(
            "/goal_pose",
            geometry_msgs.msg.PoseStamped,
            queue_size=1
        )

        self.transform_broadcaster = tf.TransformBroadcaster()
        self.transform_listener = tf.TransformListener()

        self.controller = trajectory_tracker.PIDTracker(config)
        self.gripper_handler = gripper_handler.GripperHandler()
        self.publish_rate = rospy.Rate(config["main_loop_rate"])
        self.current_pose_xyzrpy = None


    def set_build_platform(self, build_platform):
        self.build_platform = build_platform


    def imu_callback(self, imu_message):
        self.latest_imu_message = imu_message
        self.controller.set_latest_imu_reading(imu_message)


    def marker_callback(self, marker_message):
        if self.latest_marker_message is not None:
            last_marker_pose = utils.to_xyzrpy(
                *utils.get_robot_pose_from_marker(self.latest_marker_message)
            )

        for marker in marker_message.markers:
            if marker.id == self.tracked_marker_id:
                if self.latest_marker_message is not None:
                    current_marker_pose = utils.to_xyzrpy(*utils.get_robot_pose_from_marker(marker))
                    distance_travelled = utils.get_error(last_marker_pose, current_marker_pose)
                    time_delta_seconds = (marker.header.stamp - LATEST_MARKER_MESSAGE.header.stamp).to_sec()
                    self.latest_velocity = [value / time_delta_seconds for value in distance_travelled]

                self.latest_marker_message = marker

                if self.running_build_plan:
                    self.velocity_history.append(list(self.latest_velocity))

                robot_pose = utils.get_robot_pose_from_marker(LATEST_MARKER_MESSAGE)
                self.current_pose_xyzrpy = utils.to_xyzrpy(*robot_pose)

                self.times_tracked_marker_seen = self.times_tracked_marker_seen + 1

                self.controller.set_current_position(self.robot_pose_xyzrpy)

                latest_velocity_average = utils.average_velocity(self.velocity_history, 2)

                if self.latest_imu_message is None:
                    self.controller.set_current_velocity(
                        [latest_velocity_average[0], latest_velocity_average[1], latest_velocity_average[2], 0.0, 0.0, 0.0]
                    )
                else:
                    self.controller.set_current_velocity(
                        [latest_velocity_average[0], latest_velocity_average[1], latest_velocity_average[2], 0.0, 0.0, -latest_imu_message.angular_velocity.z]
                    )

    def check_rc_message_safety(self, message, max_pwm):
        if any([abs(1500 - channel) > max_pwm for channel in go_message.channels]):
            rospy.logwarn("Too much pwm! Safety stopping on message: {}".format(go_message))

            if not self.dry_run:
                set_motor_arming(False)

            sys.exit(1)

    def publish_debug_info(self):
        self.goal_pose_publisher.publish(
            utils.pose_stamped_from_xyzrpy(
                xyzrpy=current_action.goal_pose,
                frame_id=config.PLATFORM_FRAME_ID,
                seq=0,
                stamp=rospy.Time.now()
            )
        )

        self.build_platform.send_to_tf_tree(self.transform_broadcaster)
        self.update_robot_platform_transform(self.robot_pose_xyzrpy)

    def update_controller(self):
        self.controller.set_goal_position(current_action.goal_pose)

    def update_current_action(self, build_plan, current_action):
        pose_error = self.controller.get_error()

        if not current_action.is_started:
            tolerance = current_action.pose_tolerance

            reached_goal = all([abs(error) < tol for (error, tol) in zip(pose_error, tolerance)])

            if current_action.action_type == 'open_gripper':
                if reached_goal:
                    self.gripper_handler.start_opening()
                    current_action.start()

                    self.controller.set_gains_from_config(self.no_block_gains)
                    self.controller.clear_error_integrals()

            elif current_action.action_type == 'close_gripper':
                if reached_goal:
                    self.gripper_handler.start_closing()
                    current_action.start()

                    self.controller.set_gains_from_config(self.block_held_gains)
                    self.controller.clear_error_integrals()
            else:
                current_action.start()

            if current_action.is_started:
                rospy.loginfo("Starting action: {}".format(current_action))
        else:
            if current_action.is_complete(pose_error):
                rospy.loginfo("Completed action: {}. Error: {}".format(current_action, pose_error))

                if build_plan.has_next_action():
                    current_action = build_plan.pop_next_action()

        if current_action.is_started and current_action.is_complete(pose_error) and not build_plan.has_next_action():
            rospy.loginfo("Completed all actions! Terminating")
            return None

        return current_action

    def publish_next_rc_message(self)
        go_message = self.controller.get_next_rc_override()
        self.gripper_handler.update()
        self.gripper_handler.mix_into_rc_override_message(go_message)
        self.check_message_safety(go_message, 150)
        self.rc_override_publisher.publish(go_message)

    def run_build_plan(self, build_plan):
        publish_rate = rospy.Rate(self.main_loop_rate)
        stop_message = utils.construct_stop_rc_message()

        self.rc_override_publisher.publish(stop_message)

        if not self.dry_run:
            utils.set_motor_arming(True)
            rospy.loginfo("Motors armed.")

        start_time = datetime.datetime.now()
        current_action = build_plan.pop_next_action()

        self.running_build_plan = True

        while ((datetime.datetime.now() - start_time).total_seconds() < float(self.max_build_plan_duration)) and not rospy.is_shutdown():
            loop_start = rospy.Time.now()

            self.controller.set_goal_position(current_action.goal_pose)

            current_action = self.update_current_action(build_plan, current_action)

            if current_action is None:
                break

            self.publish_next_rc_message()

            loop_end = rospy.Time.now()
            loop_time = (loop_end - loop_start).to_sec()
            if (loop_time > 1.0 / self.main_loop_hz):
                rospy.logwarn("Slow loop!")

            self.main_loop_rate.sleep()

        self.running_build_plan = False

        if not self.dry_run:
            try:
                utils.set_motor_arming(False)
            except:
                rospy.logwarn("Failed to disarm motors")

        self.rc_override_publisher.publish(stop_message)

    def wait_for_marker_data(self):
        poll_rate = rospy.Rate(10)
        while self.times_tracked_marker_seen < self.number_marker_readings_before_start and not rospy.is_shutdown():
            poll_rate.sleep()