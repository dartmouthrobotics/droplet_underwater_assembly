import assembly_action
import collections
import config
import utils
import numpy
import tf.transformations


class PlatformSlot(object):
    def __init__(self, row, column, location):
        self.row = row
        self.column = column
        self.location = location

    def __str__(self):
        return "slot_{row}_{column}".format(row=self.row, column=self.column)


class BuildPlatform(object):
    """
    Abstraction for a build platform. The slot number (0,0) is the lowest X and lowest Z.
    This means the farthest from the tag and the lowest down. Slots are numbered
    """
    def __init__(self, pickup_dimensions, drop_dimensions, row_spacing, column_spacing, min_pickup_slot, min_drop_slot, frame_id):
        self.pickup_dimensions = pickup_dimensions
        self.drop_dimensions = drop_dimensions
        self.row_spacing = row_spacing
        self.column_spacing = column_spacing
        self.min_pickup_slot = min_pickup_slot
        self.min_drop_slot = min_drop_slot
        self.frame_id = frame_id

        self.pickup_slot_matrix = self.construct_slot_matrix(
            min_slot=self.min_pickup_slot,
            slot_x_stride=self.column_spacing,
            slot_z_stride=self.row_spacing,
            number_x_strides=pickup_dimensions[1],
            number_z_strides=pickup_dimensions[0]
        )

        self.drop_slot_matrix = self.construct_slot_matrix(
            min_slot=self.min_drop_slot,
            slot_x_stride=self.column_spacing,
            slot_z_stride=self.row_spacing,
            number_x_strides=drop_dimensions[1],
            number_z_strides=drop_dimensions[0]
        )

        # running average of the platform's roll and pitch
        self.roll_pitch_estimate = [0.0, 0.0]


    def update_platform_roll_pitch_estimate(self, pose_reading, imu_reading):
        # we have base link in tag, need to compute tag in base link
        # then need to get inverse of imu to base link

        base_link_roll_world, base_link_pitch_world, _ = tf.transformations.euler_from_quaternion([
            imu_reading.orientation.x,
            imu_reading.orientation.y,
            imu_reading.orientation.z,
            imu_reading.orientation.w
        ])

        world_orientation_base_link = numpy.transpose(tf.transformations.euler_matrix(base_link_roll_world, base_link_pitch_world, 0.0))

        platform_orientation_base_link = tf.transformations.euler_matrix(pose_reading[3], pose_reading[4], pose_reading[5])

        platform_orientation_world = numpy.transpose(
            tf.transformations.concatenate_matrices(
                platform_orientation_base_link,
                world_orientation_base_link
            )
        )

        _, _, platform_orientation_world, _, _ = tf.transformations.decompose_matrix(platform_orientation_world)

        self.roll_pitch_estimate[0] = platform_orientation_world[0]
        self.roll_pitch_estimate[1] = platform_orientation_world[1]


    def publish_platform_transforms(self, current_pose, marker_stamp, transform_broadcaster):
        utils.send_transform_from_xyzrpy(
           transform_broadcaster=transform_broadcaster,
           xyzrpy=current_pose,
           parent_frame=self.frame_id,
           child_frame="/base_link",
           stamp=marker_stamp
        )

        for row_num, row in enumerate(self.pickup_slot_matrix):
            for col_num, col in enumerate(row):
                utils.send_transform_from_xyzrpy(
                    transform_broadcaster=transform_broadcaster,
                    xyzrpy=col.location,
                    parent_frame=self.frame_id,
                    child_frame="/pickup_" + str(col),
                    stamp=marker_stamp
                )

        for row_num, row in enumerate(self.drop_slot_matrix):
            for col_num, col in enumerate(row):
                utils.send_transform_from_xyzrpy(
                    transform_broadcaster=transform_broadcaster,
                    xyzrpy=col.location,
                    parent_frame=self.frame_id,
                    child_frame="/drop_" + str(col),
                    stamp=marker_stamp
                )


    def get_pickup_slot(self, row, column):
        return self.pickup_slot_matrix[row][column]


    def get_dropoff_slot(self, row, column):
        return self.drop_slot_matrix[row][column]


    def construct_slot_matrix(self, min_slot, slot_x_stride, slot_z_stride, number_x_strides, number_z_strides):
        slot_matrix = []
        for row in range(number_z_strides):
            current_row = []
            for col in range(number_x_strides):
                current_slot = list(min_slot)
                current_slot[0] = current_slot[0] + slot_x_stride * col
                current_slot[2] = current_slot[2] + row * slot_z_stride

                current_row.append(
                    PlatformSlot(
                        row=row,
                        column=col,
                        location=current_slot
                    )
                )

            slot_matrix.append(current_row)

        return slot_matrix


    def get_intermediate_moves(self, from_location, to_location, intermediate_distance):
        from_xyz = numpy.array(from_location[0:3])
        to_xyz = numpy.array(to_location[0:3])
        raw_distance = numpy.linalg.norm(to_xyz - from_xyz)
        direction_normalized = (to_xyz - from_xyz) / raw_distance

        number_intermediates = int(raw_distance / intermediate_distance)

        if number_intermediates < 1:
            return []

        intermediate_waypoints = []

        for intermediate_number in range(1, number_intermediates + 1):
            offset_amount = float(intermediate_number) * float(intermediate_distance)
            next_xyz = from_xyz + (direction_normalized * offset_amount)

            next_location = [
                next_xyz[0],    
                next_xyz[1],    
                next_xyz[2],
                to_location[3],
                to_location[4],
                to_location[5]
            ]

            intermediate_waypoints.append(next_location)

        return intermediate_waypoints


    def get_intermediate_move_actions(self, from_location, to_location, hold_time, tolerance):
        intermediate_moves = self.get_intermediate_moves(from_location, to_location, config.INTERMEDIATE_WAYPOINT_DISTANCE)

        return [
            assembly_action.AssemblyAction(
                action_type='move',
                goal_pose=intermediate,
                pose_tolerance=tolerance,
                position_hold_time=hold_time
            ) for intermediate in intermediate_moves
        ]


    def get_actions_for_slot_pair(self, from_slot_low, to_slot_low):
        from_slot_mid_low = list(from_slot_low)
        from_slot_mid_low[2] = from_slot_mid_low[2] + config.MID_LOW_Z_OFFSET

        from_slot_high = list(from_slot_low)
        from_slot_high[2] = from_slot_high[2] + config.HIGH_Z_OFFSET

        to_slot_mid_low = list(to_slot_low)
        to_slot_mid_low[2] = to_slot_mid_low[2] + config.MID_LOW_Z_OFFSET

        to_slot_high = list(to_slot_low)
        to_slot_high[2] = to_slot_high[2] + config.HIGH_Z_OFFSET

        # for any move over 15cm, break it down into 15cm chunks with a 3 second pause on each waypoint
        # how to break down motions into intermediate moves? 

        # center back to from
        center_back_to_pickup_intermediates = self.get_intermediate_move_actions(
            config.CENTER_BACK_POSE,
            from_slot_high,
            config.RAPID_MOVE_HOLD_TIME,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )
        # from to center back
        pickup_to_center_back_intermediates = self.get_intermediate_move_actions(
            from_slot_high,
            config.CENTER_BACK_POSE,
            config.RAPID_MOVE_HOLD_TIME,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        # center back to to
        center_back_to_drop_intermediates = self.get_intermediate_move_actions(
            config.CENTER_BACK_POSE,
            to_slot_high,
            config.RAPID_MOVE_HOLD_TIME,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        # to to center back
        drop_to_center_back_intermediates = self.get_intermediate_move_actions(
            to_slot_high,
            config.CENTER_BACK_POSE,
            config.RAPID_MOVE_HOLD_TIME,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        actions = []

        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_pickup_intermediates)
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE)) # here
        actions.append(assembly_action.AssemblyAction('move', from_slot_mid_low, config.COARSE_POSE_TOLERANCE)) # here
        actions.append(assembly_action.AssemblyAction('move', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('close_gripper', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(pickup_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE)) # here
        actions.extend(center_back_to_drop_intermediates)
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE)) #here
        actions.append(assembly_action.AssemblyAction('move', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('open_gripper', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(drop_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE)) # here

        return actions

    def convert_build_steps_into_assembly_actions(self, build_steps, gripper_handler):
        actions = []

        for step in build_steps:
            pickup_slot = self.get_pickup_slot(
                row=step.pickup_slot[0],
                column=step.pickup_slot[1]
            )

            drop_slot = self.get_dropoff_slot(
                step.drop_slot[0],
                step.drop_slot[1]
            )

            actions.extend(
                self.get_actions_for_slot_pair(pickup_slot.location, drop_slot.location)
            )

        for action in actions:
            action.gripper_handler = gripper_handler

        return actions
