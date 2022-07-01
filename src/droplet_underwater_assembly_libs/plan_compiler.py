# we will keep this very simple, so we pickup, transport, buoyancy
from collections import namedtuple
from tf import transformations
import collections
import math
import sys
import numpy

import config

#CLAW_TWIST_OFFSET = -0.034  # rads to accout for twist of the claw on frame
CLAW_TWIST_OFFSET = -0.000  # rads to accout for twist of the claw on frame

BuildStep = collections.namedtuple("BuildStep", ["from_platform", "from_slot", "to_platform", "to_slot", "pickup_buoyancy", "after_drop_buoyancy", "is_cinder_block"])

build_sequence_baby = [
    BuildStep(
        from_platform=4,
        from_slot=[7,2],
        is_cinder_block=False,
        to_platform=4,
        to_slot=[6,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0
    ),
    BuildStep(
        from_platform=4,
        from_slot=[0,2],
        is_cinder_block=False,
        to_platform=4,
        to_slot=[0,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0
    ),
]

class Params:
    slot_stride_x = 0.205
    slot_stride_y = 0.3
    block_height = 0.191
    cone_height = 0.058
    #cone_height = block_height
    center_back = [-2.6, -0.13, -0.15, 0.0, 0.0, 0.0]

    # distance above slot to drop a block
    block_grasp_height = 0.01
    block_drop_height = 0.12
    base_link_to_gripper_height_open = 0.39
    base_link_to_gripper_height_closed = 0.42

    pre_drop_hold_time = 10.0
    pre_grasp_hold_time = 10.0

    after_reposition_hold_time = 20.0

    number_slots_vertical = 4
    number_slots_x = 8


class Platform:
    # for markers frame, forward (out of paper) is +z, up facing paper is +y, right facing paper is +x
    def __init__(self, to_world_tform, first_slot_coords, platform_id):
        # first slot coords in xyz (the intuitive: -x is out from tag, +y is left of tag, -z is down)
        self.to_world_tform = transformations.inverse_matrix(to_world_tform)
        self.platform_id = platform_id

        self.first_slot_coords = first_slot_coords
        self.slot_matrix = self.get_slot_matrix_world_frame()

    def get_slot_matrix_world_frame(self):
        first_slot = numpy.array(
            [self.first_slot_coords[0], self.first_slot_coords[1], self.first_slot_coords[2], 1.0]
        )

        stride_lateral = numpy.array([0.0, Params.slot_stride_x, 0.0, 0.0])
        stride_vertical_block = numpy.array([0.0, 0.0, Params.block_height, 0.0])
        stride_vertical_cone = numpy.array([0.0, 0.0, Params.cone_height, 0.0])

        slot_matrix_platform_frame = []
        height_vector = numpy.array([0.0, 0.0, 0.0, 0.0])

        for y_idx in range(Params.number_slots_vertical):
            current_row = []

            if y_idx > 0:
                if y_idx % 2 == 1:
                    height_vector = stride_vertical_block + height_vector
                else:
                    height_vector = stride_vertical_cone + height_vector

            for x_idx in range(Params.number_slots_x):
                slot_loc = first_slot + (float(x_idx) * stride_lateral) + height_vector
                current_row.append(slot_loc)

            #if y_idx % 2 == 1:
            #    height_vector = stride_vertical_block + height_vector
            #else:
            #    height_vector = stride_vertical_cone + height_vector

            slot_matrix_platform_frame.append(current_row)

        return [
            list(map(self.to_world_frame, row)) for row in slot_matrix_platform_frame
        ]

    def to_world_frame(self, location):
        return self.to_world_tform.dot(location)


def compile_slot_scan(platform, output_file):
    platform_yaw = get_platform_yaw(platform.to_world_tform)

    height_offset = Params.base_link_to_gripper_height_closed + 0.04
    outfile_lines = []

    rows = platform.slot_matrix[1:3]

    for row in rows:
        for point in row:
            mv = "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=point[0],
                y=point[1],
                z=point[2] + height_offset,
                yaw=platform_yaw,
            )
            hld = "HOLD 20.0"
            outfile_lines.append(mv)
            outfile_lines.append(hld)

    with open(output_file, "w") as f:
        file_str = "\n".join(outfile_lines)
        f.write(file_str)


def get_platform_yaw(to_world):
    platform_yaw = transformations.euler_from_matrix(to_world, 'szyx')
    print "PLATFORM YAW {}".format(platform_yaw)
    platform_yaw = platform_yaw[0] + CLAW_TWIST_OFFSET

    return platform_yaw



def compile_build_plan(sequence, platform, output_file):
    outfile_lines = []

    #platform_yaw = transformations.decompose_matrix(platform.to_world_tform)[2][2]
    platform_yaw = get_platform_yaw(platform.to_world_tform )
    
    for i, step in enumerate(sequence):
        offset = numpy.array([0.0, 0.0, 0.0, 0.0])

        if step.is_cinder_block:
            offset = numpy.array([Params.slot_stride_x / 2.0, 0.0, 0.0, 0.0])

        from_slot_world = platform.slot_matrix[step.from_slot[1]][step.from_slot[0]] - offset
        to_slot_world = platform.slot_matrix[step.to_slot[1]][step.to_slot[0]] - offset

        to_slot_high_z = to_slot_world[2] + Params.base_link_to_gripper_height_closed + Params.block_drop_height + 0.05
        to_slot_pre_drop_z = to_slot_world[2] + Params.base_link_to_gripper_height_closed + Params.block_drop_height

        from_slot_high_z = from_slot_world[2] + Params.base_link_to_gripper_height_open + Params.block_grasp_height + 0.10
        from_slot_pre_grasp_z = from_slot_world[2] + Params.base_link_to_gripper_height_open + Params.block_grasp_height

        outfile_lines.append("; BUILD STEP {}".format(i))
        outfile_lines.append(";;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        outfile_lines.append("; SEQUENCE GRASP {}".format(step.from_slot))
        # center back
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=Params.center_back[2],
                yaw=platform_yaw
            )
        )

        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time/2))

        # over target high
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=from_slot_world[0],
                y=from_slot_world[1],
                z=from_slot_high_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time))
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=from_slot_world[0],
                y=from_slot_world[1],
                z=from_slot_pre_grasp_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(Params.pre_grasp_hold_time))
        outfile_lines.append("CLOSE_GRIPPER")

        outfile_lines.append("CHANGE_BUOYANCY {} {} 1".format(
            step.pickup_buoyancy,
            from_slot_pre_grasp_z + 0.1 
        ))

        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=Params.center_back[2],
                yaw=platform_yaw,
            )
        )

        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time/2))

        outfile_lines.append(";")
        outfile_lines.append(";;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        outfile_lines.append("; SEQUENCE DROP {}".format(step.to_slot))

        # over target high
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=to_slot_world[0],
                y=to_slot_world[1],
                z=to_slot_high_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time))
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=to_slot_world[0],
                y=to_slot_world[1],
                z=to_slot_pre_drop_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(Params.pre_grasp_hold_time))
        outfile_lines.append("OPEN_GRIPPER")

        outfile_lines.append("CHANGE_BUOYANCY {} {} -1".format(
            step.after_drop_buoyancy,
            to_slot_pre_drop_z - 0.05 
        ))

        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=Params.center_back[2],
                yaw=platform_yaw,
            )
        )

        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time))

    with open(output_file, "w") as f:
        file_str = "\n".join(outfile_lines)
        f.write(file_str)

def format_point(point):
    return "{:.3f},{:.3f},{:.3f}".format(point[0], point[1], point[2])

if __name__ == '__main__':
    p4 = Platform(
        to_world_tform=config.PLATFORM_TO_WORLD_MATRIX,
        #first_slot_coords=[1.654, -0.508, 0.184],
        first_slot_coords=[-0.414, -0.907, -0.508],
        platform_id=4,
    )

    print ""
    for row in p4.slot_matrix:
        print " || ".join(map(format_point, row))
    print ""

    compile_build_plan(
        build_sequence_baby,
        p4,
        "/home/sam/Dev/ros-catkin-workspace/src/droplet_underwater_assembly/param/test_compiled_build_plan.txt"
    )
    #compile_slot_scan(
    #    p4,
    #    "/home/sam/Dev/ros-catkin-workspace/src/droplet_underwater_assembly/param/test_compiled_build_plan.txt"
    #)
