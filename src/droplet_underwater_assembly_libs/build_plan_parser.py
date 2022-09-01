import numpy
import config
import assembly_action
import rospy
import config
import random


def displace_point_randomly(point, xyzyaw_range):
    assert(len(xyzyaw_range) == 4)

    return [
        point[0] + random.uniform(
            -xyzyaw_range[0], xyzyaw_range[0]
        ),
        point[1] + random.uniform(
            -xyzyaw_range[1], xyzyaw_range[1]
        ),
        point[2] + random.uniform(
            -xyzyaw_range[2], xyzyaw_range[2]
        ),
        point[3],
        point[4],
        point[5] + random.uniform(
            -xyzyaw_range[3], xyzyaw_range[3]
        ),
    ]

class BuildPlanParser(object):
    def __init__(self, build_platforms, high_offset, mid_low_offset):
        self.platforms_by_id = {
            platform.tag_id: platform for platform in build_platforms
        }
        self.high_offset = high_offset
        self.mid_low_offset = mid_low_offset

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


    def get_actions_to_pick_up_block(self, platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """

        actions = []

        platform = self.platforms_by_id[platform_id]

        center_back_pose = list(platform.center_back_pose)

        if config.RANDOMLY_DISPLACE_CENTER_BACK:
            center_back_pose = displace_point_randomly(center_back_pose, config.RANDOM_DISPLACEMENT_RANGE)

        from_slot_low = platform.get_slot_coordinates(slot_coords)
        from_slot_mid_low = list(from_slot_low)
        from_slot_mid_low[2] = from_slot_low[2] + self.mid_low_offset

        from_slot_high = list(from_slot_low)
        from_slot_high[2] = from_slot_high[2] + self.high_offset

        center_back_to_pickup_intermediates = self.get_intermediate_move_actions(
            center_back_pose,
            from_slot_high,
            1.0,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        pickup_to_center_back_intermediates = self.get_intermediate_move_actions(
            from_slot_high,
            center_back_pose,
            1.0,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        actions.append(assembly_action.AssemblyAction('move', center_back_pose, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_pickup_intermediates)
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_mid_low, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('close_gripper', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(pickup_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', center_back_pose, config.ULTRA_COARSE_POSE_TOLERANCE))

        return actions


    def get_actions_to_drop_block(self, platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """
        actions = []

        platform = self.platforms_by_id[platform_id]

        center_back_pose = list(platform.center_back_pose)

        if config.RANDOMLY_DISPLACE_CENTER_BACK:
            center_back_pose = displace_point_randomly(center_back_pose, config.RANDOM_DISPLACEMENT_RANGE)

        to_slot_low = platform.get_slot_coordinates(slot_coords)
        to_slot_mid_low = list(to_slot_low)
        to_slot_mid_low[2] = to_slot_low[2] + self.mid_low_offset

        to_slot_high = list(to_slot_low)
        to_slot_high[2] = to_slot_high[2] + self.high_offset

        center_back_to_drop_intermediates = self.get_intermediate_move_actions(
            center_back_pose,
            to_slot_high,
            1.0,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        drop_to_center_back_intermediates = self.get_intermediate_move_actions(
            to_slot_high,
            center_back_pose,
            1.0,
            config.ULTRA_COARSE_POSE_TOLERANCE
        )

        actions.append(assembly_action.AssemblyAction('move', center_back_pose, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_drop_intermediates)
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('open_gripper', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(drop_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', center_back_pose, config.ULTRA_COARSE_POSE_TOLERANCE))

        return actions


    def construct_actions_from_text(self, build_plan_file, start_platform):
        parsed_actions = []

        current_platform = start_platform

        holding_block = False
        last_pose = None
        center_back = None
        currently_selected_tolerance = config.COARSE_POSE_TOLERANCE

        with open(build_plan_file) as file_stream:
            file_lines = file_stream.readlines()

            for line_number, line in enumerate(file_lines):
                print line
                line = line.strip()
                command = line.split(';')[0]

                if command:
                    if command.startswith("PICKUP"):
                        if holding_block:
                            raise Exception("Cannot do a pickup action while a block is still picked up.")

                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        if next_platform_id not in self.platforms_by_id:
                            raise Exception("Attempting to pick up from platform {} which does not exist. Valid platforms: {}".format(next_platform_id, list(self.platforms_by_id.keys())))

                        holding_block = True

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = assembly_action.AssemblyAction.construct_change_platforms(next_platform_id)
                            next_action.high_level_build_step = line
                            parsed_actions.append(next_action)
                            current_platform = next_platform_id
                            next_action = assembly_action.AssemblyAction('binary_P_move', self.platforms_by_id[current_platform].center_back_pose, config.ULTRA_COARSE_POSE_TOLERANCE)
                            next_action.high_level_build_step = line
                            parsed_actions.append(next_action)

                        # add the actions to move to the point and then close the gripper
                        actions_to_pick_up_block = self.get_actions_to_pick_up_block(
                            current_platform,
                            (x_coord, y_coord, z_coord)
                        )
                        for action in actions_to_pick_up_block:
                            action.high_level_build_step = line

                        parsed_actions.extend(
                            actions_to_pick_up_block
                        )
                        last_pose = parsed_actions[-1].goal_pose 

                    elif command.startswith("DROP"):
                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        if not holding_block:
                            raise Exception("Cannot do a drop action while not holding a block.")

                        if next_platform_id not in self.platforms_by_id:
                            raise Exception("Attempting to drop on platform {} which does not exist. Valid platforms: {}".format(next_platform_id, list(self.platforms_by_id.keys())))
                        
                        holding_block = False

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = assembly_action.AssemblyAction.construct_change_platforms(next_platform_id)
                            next_action.high_level_build_step = line
                            current_platform = next_platform_id
                            parsed_actions.append(next_action)

                            next_action = assembly_action.AssemblyAction('binary_P_move', self.platforms_by_id[current_platform].center_back_pose, config.ULTRA_COARSE_POSE_TOLERANCE)
                            next_action.high_level_build_step = line
                            parsed_actions.append(next_action)

                        # add the actions to move to the point and then open the gripper
                        actions_to_drop_block = self.get_actions_to_drop_block(
                            current_platform,
                            (x_coord, y_coord, z_coord)
                        )
                        for action in actions_to_drop_block:
                            action.high_level_build_step = line

                        parsed_actions.extend(
                            actions_to_drop_block
                        )

                        last_pose = parsed_actions[-1].goal_pose 

                    elif command.startswith("MOVE"):
                        """
                        move to an xyzrpy location relative to the platform
                        """
                        platform_id = int(command.split(" ")[1])

                        x, y, z, roll, pitch, yaw = map(float, command.split(" ")[2:8])

                        target_location = [
                            x,y,z,roll,pitch,yaw
                        ]

                        parsed_actions.append(
                            assembly_action.AssemblyAction('move', target_location, currently_selected_tolerance, position_hold_time=0.0)
                        )

                        last_pose = parsed_actions[-1].goal_pose 
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("SET_CENTER_BACK"):
                        center_back = list(map(float, command.split(" ")[1:]))

                    elif command.startswith("BAILING_RELEASE"):
                        thrust_down_amount = float(command.split(" ")[1])
                        bail_time = float(command.split(" ")[2])
                        pre_open_time = float(command.split(" ")[3])

                        action = assembly_action.AssemblyAction(
                            action_type='bailing_release',
                            goal_pose=last_pose,
                            pose_tolerance=config.ULTRA_COARSE_POSE_TOLERANCE,
                            thrust_down_amount=thrust_down_amount,
                            bail_time=bail_time,
                            pre_open_time=pre_open_time,
                        )

                        parsed_actions.append(
                            action
                        )
                        parsed_actions[-1].high_level_build_step = line

                        if 'SHIFT_RIGHT' in command:
                            action.shift_left = True
                            action.shift_right = False
                        elif 'SHIFT_LEFT' in command:
                            action.shift_left = False
                            action.shift_right = True
                        else:
                            action.shift_left = False
                            action.shift_right = False

                    elif command.startswith("SET_TOLERANCE"):
                        tolerance_level = command.split(" ")[1].strip().lower()

                        if tolerance_level == 'tight':
                            currently_selected_tolerance = config.TIGHT_POSE_TOLERANCE
                        elif tolerance_level == 'coarse':
                            currently_selected_tolerance = config.COARSE_POSE_TOLERANCE
                        elif tolerance_level == 'ultra_coarse':
                            currently_selected_tolerance = config.ULTRA_COARSE_POSE_TOLERANCE
                        else:
                            raise Exception("Invalid tolerance level given in SET_TOLERANCE command! {}".format(tolerance_level))

                    elif command.startswith("HOLD"):
                        hold_time = float(command.split(" ")[1].strip())

                        parsed_actions.append(
                            assembly_action.AssemblyAction(
                                'hold',
                                last_pose,
                                currently_selected_tolerance,
                                position_hold_time=hold_time
                            )
                        )
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("ROTATE_WRIST"):
                        # add a rotate wrist command
                        target_pose = last_pose

                        if last_pose is None:
                            rospy.logwarn("ROTATE_WRIST called before a last pose is known.")
                            target_pose = self.platforms_by_id[current_platform].center_back_pose

                        target_pwm = int(command.split(" ")[1])

                        parsed_actions.append(
                            assembly_action.AssemblyAction.construct_rotate_wrist(
                                pwm=target_pwm,
                                pose=target_pose
                            )
                        )
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("INFLATE_BALLAST"):
                        parsed_actions.append(
                            assembly_action.AssemblyAction(
                                'inflate_ballast',
                                last_pose,
                                currently_selected_tolerance,
                                position_hold_time=0.0
                            )
                        )
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("CHANGE_BUOYANCY"):
                        # t_level # z
                        tokens = command.split(" ")[1:] 
                        t_level = float(tokens[0])
                        z = float(tokens[1])
                        direction = int(tokens[2])

                        new_pose = list(last_pose)
                        if direction < 0:
                            new_pose = list(center_back)
                        new_pose[2] = z
                        print("Creating new pose: {}".format(new_pose))
                        print("Direction", direction)
                        print("t_level", t_level)

                        new_action = assembly_action.AssemblyAction(
                            'change_buoyancy',
                            new_pose,
                            currently_selected_tolerance,
                            position_hold_time=0.0,
                            t_level_from_planner=t_level,
                            ballast_change_direction=direction 
                        )

                        parsed_actions.append(new_action)
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("DEFLATE_BALLAST"):
                        parsed_actions.append(
                            assembly_action.AssemblyAction(
                                'deflate_ballast',
                                last_pose,
                                config.COARSE_POSE_TOLERANCE,
                                position_hold_time=0.0
                            )
                        )
                        parsed_actions[-1].high_level_build_step = line
                    elif command.startswith("LEFT_RIGHT_MOVE"):
                        tokens = command.split(" ")[1:]
                        parsed_actions.append(
                            assembly_action.AssemblyAction.construct_move_left_right(
                                x=float(tokens[0]),
                                y=float(tokens[1])
                            )
                        )
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("OPEN_GRIPPER"):
                        action = assembly_action.AssemblyAction(
                            'open_gripper',
                            last_pose,
                            currently_selected_tolerance,
                            position_hold_time=0.0
                        )

                        if 'SHIFT_RIGHT' in command:
                            action.shift_left = True
                            action.shift_right = False
                        elif 'SHIFT_LEFT' in command:
                            action.shift_left = False
                            action.shift_right = True
                        else:
                            action.shift_left = False
                            action.shift_right = False
                        
                        parsed_actions.append(
                            action
                        )
                        parsed_actions[-1].high_level_build_step = line

                    elif command.startswith("CLOSE_GRIPPER"):
                        parsed_actions.append(
                            assembly_action.AssemblyAction(
                                'close_gripper',
                                last_pose,
                                currently_selected_tolerance,
                                position_hold_time=0.0
                            )
                        )
                        parsed_actions[-1].high_level_build_step = line

                    else:
                        raise Exception("Unrecognized command in line {}: {}".format(line_number, line))

        for idx, action in enumerate(parsed_actions):
            action.sequence_id = idx

        return parsed_actions
