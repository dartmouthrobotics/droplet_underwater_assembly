import collections
import assembly_action
import tf

class BuildPlan(object):
    def __init__(self, config_json, transform_listener):
        self.actions = collections.deque()
        self.construct_from_dictionary(config_json, transform_listener)

    def construct_from_dictionary(self, config_json, transform_listener):
        plan = config_json["build_plan"]

        tight_tolerance =   config_json["tight_tolerance_xyzrpy"]
        loose_tolerance =   config_json["loose_tolerance_xyzrpy"]
        grab_height =       config_json["grab_height"]
        hover_height =      config_json["above_slot_hover_height"]
        grab_height =       config_json["grab_height"]
        center_back_pose =  config_json["center_back_pose"]

        gripper_height = transform_listener.lookupTransform(
            "/base_link",
            "/gripper_center"
        )[0][2]

        for step_dict in plan:
            pickup_slot = step_dict["pickup_slot"]
            drop_slot =   step_dict["drop_slot"]

            pickup_slot_location = transform_listener.lookupTransform(
                "/build_platform",
                "/pickup_slot_{}".format(pickup_slot)
            )[0]

            dropoff_slot_location = transform_listener.lookupTransform(
                "/build_platform",
                "/dropoff_slot_{}".format(drop_slot)
            )[0]

            pre_pickup_hover = pickup_slot_location + [0.0, 0.0, 0.0]
            pre_pickup_hover[2] = pre_pickup_hover[2] + hover_height + gripper_height
            pre_grab_location = pickup_slot_location + [0.0, 0.0, 0.0]
            pre_grab_location[2] = pre_grab_location[2] + grab_height + gripper_height

            pre_drop_hover = dropoff_slot_location + [0.0, 0.0, 0.0]
            pre_drop_hover[2] = pre_drop_hover[2] + hover_height + gripper_height

            # pickup
            # move to center back
            self.actions.append(
                assembly_action.AssemblyAction("move", center_back_pose, loose_tolerance)
            )

            # move to over block
            self.actions.append(
                assembly_action.AssemblyAction("move", pre_pickup_hover, loose_tolerance)
            )
            # move to grab height
            self.actions.append(
                assembly_action.AssemblyAction("move", pre_grab_location, loose_tolerance)
            )
            # grab the block
            self.actions.append(
                assembly_action.AssemblyAction("close_gripper", pre_grab_location, tight_tolerance)
            )
            # move to over block
            self.actions.append(
                assembly_action.AssemblyAction("move", pre_pickup_hover, loose_tolerance)
            )
            # move to center back
            self.actions.append(
                assembly_action.AssemblyAction("move", center_back_pose, loose_tolerance)
            )

            # drop
            # move to center back
            self.actions.append(
                assembly_action.AssemblyAction("move", center_back_pose, loose_tolerance)
            )
            # move to over block
            self.actions.append(
                assembly_action.AssemblyAction("move", pre_drop_hover, loose_tolerance)
            )
            # drop the block
            self.actions.append(
                assembly_action.AssemblyAction("open_gripper", pre_drop_hover, tight_tolerance)
            )
            # move to center back
            self.actions.append(
                assembly_action.AssemblyAction("move", center_back_pose, loose_tolerance)
            )

    def has_next_action(self):
        return len(self.actions) > 0

    def pop_next_action(self):
        return self.actions.popleft()