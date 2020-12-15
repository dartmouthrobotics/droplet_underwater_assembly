import numpy
import config
import assembly_action

class BuildPlanParser(object):
    def __init__(self, build_platforms):
        self.platforms_by_id = {
            platform.tag_id: platform for platform in build_platforms
        }

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


    def get_actions_to_approach_pickup_slot(self, platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """

        platform = self.platforms_by_id[platform_id]

        center_back_pose = platform.center_back_pose

        slot_location = platform.get_slot_coordinates(slot_coords)

        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_pickup_intermediates)
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_mid_low, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('close_gripper', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(pickup_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))


    def get_actions_to_approach_drop_slot(platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_drop_intermediates)
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('open_gripper', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(drop_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))


    def construct_actions_from_text(build_plan_file, start_platform, start_wrist_pwm):
        parsed_actions = []

        current_platform = start_platform
        current_pwm = start_wrist_pwm

        holding_block = False

        with open(build_plan_file) as file_stream:
            file_lines = file_stream.readlines()

            for line_number, line in enumerate(file_lines):
                line = line.strip()
                command, _comment = line.split(';')

                if command:
                    if command.startswith("PICKUP"):
                        if holding_block:
                            raise Exception("Cannot do a pickup action while a block is still picked up.")

                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        holding_block = True

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = AssemblyAction.construct_change_platforms(next_platform_id),
                            parsed_actions.append(next_action)
                            current_platform = next_platform_id

                        # add the actions to move to the point and then close the gripper

                    elif command.startswith("DROP"):
                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        if not holding_block:
                            raise Exception("Cannot do a drop action while not holding a block.")

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = AssemblyAction.construct_change_platforms(next_platform_id),
                            current_platform = next_platform_id

                        # add the actions to move to the point and then open the gripper

                    elif command.startswith("ROTATE_WRIST"):
                        # add a rotate wrist command
                        pass
 
class BuildPlanParser(object):
    def __init__(self, build_platforms):
        self.platforms_by_id = {
            platform.tag_id: platform for platform in build_platforms
        }

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


    def get_actions_to_approach_pickup_slot(self, platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """

        platform = self.platforms_by_id[platform_id]

        center_back_pose = platform.center_back_pose

        slot_location = platform.get_slot_coordinates(slot_coords)

        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_pickup_intermediates)
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_mid_low, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('close_gripper', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(pickup_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))


    def get_actions_to_approach_drop_slot(platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_drop_intermediates)
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('open_gripper', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(drop_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))


    def construct_actions_from_text(build_plan_file, start_platform, start_wrist_pwm):
        parsed_actions = []

        current_platform = start_platform
        current_pwm = start_wrist_pwm

        holding_block = False

        with open(build_plan_file) as file_stream:
            file_lines = file_stream.readlines()

            for line_number, line i                   else:
                        raise Exception("Unrecognized command in line {line_number}: {bad_line}".format(line_number=line_number, bad_line=line))
n enumerate(file_lines):
                line = line.strip()
                command, _comment = line.split(';')

                if command:
                    if command.startswith("PICKUP"):
                        if holding_block:
                            raise Exception("Cannot do a pickup action while a block is still picked up.")

                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        holding_block = True

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = AssemblyAction.construct_change_platforms(next_platform_id),
                            parsed_actions.append(next_action)
                            current_platform = next_platform_id

                        # add the actions to move to the point and then close the gripper

                    elif command.startswith("DROP"):
                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        if not holding_block:
                            raise Exception("Cannot do a drop action while not holding a block.")

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = AssemblyAction.construct_change_platforms(next_platform_id),
                            current_platform = next_platform_id

                        # add the actions to move to the point and then open the gripper

                    elif command.startswith("ROTATE_WRIST"):
                        # add a rotate wrist command
                        pass
                    else:
                        raise Exception("Unrecognized command in line {line_number}: {bad_line}".format(line_number=line_number, bad_line=line))

class BuildPlanParser(object):
    def __init__(self, build_platforms):
        self.platforms_by_id = {
            platform.tag_id: platform for platform in build_platforms
        }

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


    def get_actions_to_approach_pickup_slot(self, platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """

        platform = self.platforms_by_id[platform_id]

        center_back_pose = platform.center_back_pose

        slot_location = platform.get_slot_coordinates(slot_coords)

        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_pickup_intermediates)
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_mid_low, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('close_gripper', from_slot_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', from_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(pickup_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))


    def get_actions_to_approach_drop_slot(platform_id, slot_coords):
        """
        approach the slot then move back to center back pos
        """
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))
        actions.extend(center_back_to_drop_intermediates)
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('open_gripper', to_slot_mid_low, config.TIGHT_POSE_TOLERANCE))
        actions.append(assembly_action.AssemblyAction('move', to_slot_high, config.COARSE_POSE_TOLERANCE))
        actions.extend(drop_to_center_back_intermediates)
        actions.append(assembly_action.AssemblyAction('move', config.CENTER_BACK_POSE, config.ULTRA_COARSE_POSE_TOLERANCE))


    def construct_actions_from_text(build_plan_file, start_platform, start_wrist_pwm):
        parsed_actions = []

        current_platform = start_platform
        current_pwm = start_wrist_pwm

        holding_block = False

        with open(build_plan_file) as file_stream:
            file_lines = file_stream.readlines()

            for line_number, line in enumerate(file_lines):
                line = line.strip()
                command, _comment = line.split(';')

                if command:
                    if command.startswith("PICKUP"):
                        if holding_block:
                            raise Exception("Cannot do a pickup action while a block is still picked up.")

                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        holding_block = True

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = AssemblyAction.construct_change_platforms(next_platform_id),
                            parsed_actions.append(next_action)
                            current_platform = next_platform_id

                        # add the actions to move to the point and then close the gripper

                    elif command.startswith("DROP"):
                        next_platform_id, x_coord, y_coord, z_coord = map(int, line.split(" ")[1:5])

                        if not holding_block:
                            raise Exception("Cannot do a drop action while not holding a block.")

                        if next_platform_id != current_platform:
                            # switch platform
                            next_action = AssemblyAction.construct_change_platforms(next_platform_id),
                            current_platform = next_platform_id

                        # add the actions to move to the point and then open the gripper

                    elif command.startswith("ROTATE_WRIST"):
                        # add a rotate wrist command
                        pass
                    else:
                        raise Exception("Unrecognized command in line {line_number}: {bad_line}".format(line_number=line_number, bad_line=line))