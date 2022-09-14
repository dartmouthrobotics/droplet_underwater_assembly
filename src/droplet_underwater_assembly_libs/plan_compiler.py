# we will keep this very simple, so we pickup, transport, buoyancy
from collections import namedtuple
from tf import transformations
import collections
import math
import sys
import numpy

import config

# need the block pickup locations and the cone pickup locations

#CLAW_TWIST_OFFSET = -0.034  # rads to accout for twist of the claw on frame
CLAW_TWIST_OFFSET = -0.000  # rads to accout for twist of the claw on frame

BuildStep = collections.namedtuple("BuildStep", ["from_platform", "to_platform", "to_slot", "pickup_buoyancy", "after_drop_buoyancy", "is_cinder_block", "is_half_block", "shift_drop_left", "shift_drop_right"])

build_sequence_two_half_blocks = [
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=False,
        is_half_block=True,
        to_slot=[4,1],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=False,
        is_half_block=True,
        to_slot=[5,1],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=True,
    ),
]

build_sequence_three_block_column = [
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_slot=[4,1],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_slot=[4,2],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_slot=[5,2],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_slot=[4,3],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_slot=[4,3],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_slot=[5,3],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        to_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_slot=[4,4],
        pickup_buoyancy=0.6,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
]

#build_sequence_baby = [ # one block on top of the other
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=True,
#        is_half_block=False,
#        to_platform=4,
#        to_slot=[6,1],
#        pickup_buoyancy=0.4,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=False,
#        to_platform=4,
#        to_slot=[7,2],
#        pickup_buoyancy=0.7,
#        after_drop_buoyancy=0.0,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=False,
#        to_platform=4,
#        to_slot=[6,2],
#        pickup_buoyancy=0.7,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=True,
#        to_platform=4,
#        to_slot=[6,3],
#        pickup_buoyancy=0.5,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#]
#
#build_sequence_two_base_pyramid = [ # for testing side by side block placement
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=True,
#        to_platform=4,
#        to_slot=[6,1],
#        pickup_buoyancy=0.8,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=True,
#        to_platform=4,
#        to_slot=[4,1],
#        pickup_buoyancy=0.8,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=True,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=False,
#        to_platform=4,
#        to_slot=[6,2],
#        pickup_buoyancy=0.7,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=False,
#        to_platform=4,
#        to_slot=[5,2],
#        pickup_buoyancy=0.7,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#    BuildStep(
#        from_platform=4,
#        is_cinder_block=True,
#        to_platform=4,
#        to_slot=[5,3],
#        pickup_buoyancy=0.8,
#        after_drop_buoyancy=0.0,
#        shift_drop_left=False,
#        shift_drop_right=False,
#    ),
#]
#
build_sequence_five_block_pyramid = [
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[2,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
]

build_sequence_wall_segment = [
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[2,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ), # first layer of cinder blocks
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[7,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[2,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ), # second row cones
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ), # third row full blocks
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=True,
        to_platform=4,
        to_slot=[7,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=True,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=True,
        to_platform=4,
        to_slot=[2,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ), # third row half blocks
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[7,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[2,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ), # fourth row cones
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,4],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ), # last row blocks
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,4],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[2,4],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
]

build_sequence_six_block_pyramid = [
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[2,1],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[6,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,2],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=True,
        is_half_block=False,
        to_platform=4,
        to_slot=[3,3],
        pickup_buoyancy=0.8,
        after_drop_buoyancy=0.0,
        shift_drop_left=True,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[4,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    BuildStep(
        from_platform=4,
        is_cinder_block=False,
        is_half_block=False,
        to_platform=4,
        to_slot=[5,3],
        pickup_buoyancy=0.7,
        after_drop_buoyancy=0.0,
        shift_drop_left=False,
        shift_drop_right=False,
    ),
    #BuildStep(
    #    from_platform=4,
    #    is_cinder_block=True,
    #    is_half_block=False,
    #    to_platform=4,
    #    to_slot=[4,4],
    #    pickup_buoyancy=0.8,
    #    after_drop_buoyancy=0.0,
    #    shift_drop_left=False,
    #    shift_drop_right=False,
    #),
]

class Params:
    is_test_run = False
    slot_stride_x = 0.205
    slot_stride_y = 0.3
    block_height = 0.230
    cone_height = 0.100
    #cone_height = block_height
    center_back = [-2.6, 0.05, 0.60, 0.0, 0.0, 0.0]

    # distance above slot to drop a block
    block_grasp_height = 0.01
    block_drop_height = 0.12
    base_link_to_gripper_height_open = 0.39
    base_link_to_gripper_height_closed = 0.42

    pre_drop_hold_time = 10.0
    pre_grasp_hold_time = 10.0

    after_reposition_hold_time = 15.0

    number_slots_vertical = 5
    number_slots_x = 8

    intermediate_factor = 0.5

    shift_over_amount_pre_drop = 0.06

    # 68 inches to l bracket plus half block width
    farthest_l_bracket_x_dist_meters = 1.7272 + 0.02
    block_half_depth = 0.144/2.0
    tslot_left_of_first_slot = 0.139

    release_down_thrust_amount = 0.1
    release_bail_time = 10.0
    pre_open_time = 3.0

    back_bottom_block_pickup = []
    back_bottom_cone_pickup = []


class Platform:
    # for markers frame, forward (out of paper) is +z, up facing paper is +y, right facing paper is +x
    def __init__(self, to_world_tform, first_slot_coords, manual_yaw_bump_rads, platform_id):
        # first slot coords in xyz (the intuitive: -x is out from tag, +y is left of tag, -z is down)
        self.to_world_tform = transformations.inverse_matrix(to_world_tform)

        self.yaw_bump_matrix = transformations.euler_matrix(
            0.0,
            0.0,
            manual_yaw_bump_rads,
            'sxyz'
        )
        self.pitch_bump_matrix = transformations.euler_matrix(
            0.0,
            0.00,
            0.0,
            'sxyz'
        )

        self.to_world_tform = transformations.concatenate_matrices(
            self.yaw_bump_matrix,
            self.to_world_tform
        )

        self.to_world_tform = transformations.concatenate_matrices(
            self.pitch_bump_matrix,
            self.to_world_tform
        )

        self.platform_id = platform_id

        self.first_slot_coords = first_slot_coords
        self.slot_matrix = self.get_slot_matrix_world_frame()
        self.recenter_position_local = numpy.array(
            [-1.25, -0.35, 0.40, 1.0]
        )
        self.recenter_position_world = self.to_world_frame(self.recenter_position_local)

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


def get_block_pickup_locations(platform):
    first_slot_coords = platform.first_slot_coords
    pickup_slot_stride_x = 12.0 * 0.0254
    z_offset = -0.06
    y_offset = 0.08

    return [
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 2*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + 1.0 * pickup_slot_stride_x,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 2*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + 2.0 * pickup_slot_stride_x,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 2*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + 3.0 * pickup_slot_stride_x,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 2*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 1.0*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + 1.0 * pickup_slot_stride_x,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 1.0*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + 2.0 * pickup_slot_stride_x,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 1.0*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + 3.0 * pickup_slot_stride_x,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + Params.slot_stride_x/2.0 + y_offset,
            first_slot_coords[2] + 1.0*Params.block_height + z_offset,
            1.0
        ])),
    ]


def get_half_block_pickup_locations(platform):
    first_slot_coords = platform.first_slot_coords
    pickup_slot_stride_x = 12.0 * 0.0254

    x_offset = 0.01
    y_offset = -0.13
    z_offset = -0.06

    return [
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + 4.0 * Params.slot_stride_x + y_offset,
            first_slot_coords[2] + 1*Params.block_height + z_offset,
            1.0
        ])),
        platform.to_world_frame(numpy.array([
            first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset,
            first_slot_coords[1] - Params.tslot_left_of_first_slot + 3.0 * Params.slot_stride_x + y_offset,
            first_slot_coords[2] + 1*Params.block_height + z_offset,
            1.0
        ])),
    ]


def get_cone_pickup_locations(platform):
    first_slot_coords = platform.first_slot_coords
    pickup_slot_stride_x = 12.0 * 0.0254
    y_offset = 0.14
    z_offset = 0.06
    x_offset = 0.03

    return [
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset,
                platform.first_slot_coords[1] + (7.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset,
                platform.first_slot_coords[1] + (6.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset,
                platform.first_slot_coords[1] + (5.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset,
                platform.first_slot_coords[1] + (4.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),

        # SECOND ROW
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 1.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (7.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 1.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (6.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 1.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (5.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 1.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (4.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        # THIRD ROW
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 2.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (7.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 2.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (6.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 2.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (5.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
        platform.to_world_frame(
            numpy.array([
                first_slot_coords[0] - Params.farthest_l_bracket_x_dist_meters - Params.block_half_depth + x_offset + 2.0 * pickup_slot_stride_x,
                platform.first_slot_coords[1] + (4.0*Params.slot_stride_x) + y_offset,
                first_slot_coords[2] + 1.0*Params.block_height + 1.0 * Params.cone_height - z_offset,
                1.0
            ])
        ),
    ]


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

    Params.center_back[0] = platform.recenter_position_world[0]
    Params.center_back[1] = platform.recenter_position_world[1]
    Params.center_back[2] = platform.recenter_position_world[2]

    current_block_idx = 0
    current_cone_idx = 0
    current_half_block_idx = 0

    #platform_yaw = transformations.decompose_matrix(platform.to_world_tform)[2][2]
    platform_yaw = get_platform_yaw(platform.to_world_tform)
    after_place_surface_clear_xy = [platform.slot_matrix[0][2][0], platform.slot_matrix[0][2][1]]
    outfile_lines.append("SET_CENTER_BACK {}".format(str(Params.center_back).replace("[", "").replace("]","").replace(",","")))

    block_pickup_locations = get_block_pickup_locations(platform)
    cone_pickup_locations = get_cone_pickup_locations(platform)
    half_block_pickup_locations = get_half_block_pickup_locations(platform)
    
    for i, step in enumerate(sequence):
        offset = numpy.array([0.0, 0.0, 0.0, 0.0])

        if step.is_cinder_block:
            offset = numpy.array([0.0, -Params.slot_stride_x / 2.0, 0.0, 0.0])

        #from_slot_world = platform.slot_matrix[step.from_slot[1]][step.from_slot[0]] - offset
        from_slot_world = None
        if step.is_cinder_block:
            from_slot_world = block_pickup_locations[current_block_idx]
            current_block_idx = current_block_idx + 1
        elif step.is_half_block:
            from_slot_world = half_block_pickup_locations[current_half_block_idx]
            current_half_block_idx = current_half_block_idx + 1
        else:
            from_slot_world = cone_pickup_locations[current_cone_idx]
            current_cone_idx = current_cone_idx + 1

        to_slot_world = platform.slot_matrix[step.to_slot[1]][step.to_slot[0]] - offset

        to_slot_high_z = to_slot_world[2] + Params.base_link_to_gripper_height_closed + Params.block_drop_height + 0.08
        to_slot_pre_drop_z = to_slot_world[2] + Params.base_link_to_gripper_height_closed + Params.block_drop_height

        if step.is_cinder_block:
            to_slot_high_z = to_slot_world[2] + Params.base_link_to_gripper_height_closed + Params.block_drop_height + 0.08
            to_slot_pre_drop_z = to_slot_world[2] + Params.base_link_to_gripper_height_closed + Params.block_drop_height

        from_slot_high_z = from_slot_world[2] + Params.base_link_to_gripper_height_open + Params.block_grasp_height + 0.14
        from_slot_pre_grasp_z = from_slot_world[2] + Params.base_link_to_gripper_height_open + Params.block_grasp_height

        outfile_lines.append("; BUILD STEP {}".format(i))
        outfile_lines.append(";;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        outfile_lines.append("; SEQUENCE GRASP {}".format(i))
        # center back
        # intermediate point
        # on approach right after center back
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; center back".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=Params.center_back[2],
                yaw=platform_yaw
            )
        )

        # on approach add a midpoint
        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time/4.0))

        # intermediate point here.
        outfile_lines.append("SET_TOLERANCE ULTRA_COARSE")
        "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; intermediate to approach pickup".format(
            x=from_slot_world[0],
            y=Params.center_back[1],
            z=from_slot_high_z + 0.02,
            yaw=platform_yaw,
        )

        #outfile_lines.append("SET_TOLERANCE ULTRA_COARSE")
        #outfile_lines.append(
        #    "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; intermediate to approach pickup".format(
        #        x=Params.center_back[0] + Params.intermediate_factor * (from_slot_world[0] - Params.center_back[0]),
        #        y=Params.center_back[1] + Params.intermediate_factor * (from_slot_world[1] - Params.center_back[1]),
        #        z=from_slot_high_z + 0.05,
        #        yaw=platform_yaw,
        #    )
        #)
        outfile_lines.append("HOLD {}".format(7.0))
        outfile_lines.append("SET_TOLERANCE ULTRA_COARSE")
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; intermediate to approach pickup".format(
                x=from_slot_world[0],
                y=Params.center_back[1] + 0.5 * (from_slot_world[1] - Params.center_back[1]),
                z=from_slot_high_z + 0.02,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(7.0))

        # over target high
        #if step.is_cinder_block:
        outfile_lines.append("SET_TOLERANCE COARSE")
        #else:
        #    outfile_lines.append("SET_TOLERANCE TIGHT")

        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=from_slot_world[0],
                y=from_slot_world[1],
                z=from_slot_high_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time))

        if step.is_cinder_block:
            outfile_lines.append("SET_TOLERANCE TIGHT")
            outfile_lines.append(
                "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                    x=from_slot_world[0],
                    y=from_slot_world[1],
                    z=from_slot_pre_grasp_z,
                    yaw=platform_yaw,
                )
            )
            outfile_lines.append("HOLD {}".format(Params.pre_grasp_hold_time))
        else:
            outfile_lines.append("SET_TOLERANCE TIGHT")
            outfile_lines.append(
                "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                    x=from_slot_world[0],
                    y=from_slot_world[1],
                    z=from_slot_high_z - 0.13,
                    yaw=platform_yaw,
                )
            )
            outfile_lines.append("HOLD {}".format(Params.pre_grasp_hold_time))

        if not Params.is_test_run:
            outfile_lines.append("CLOSE_GRIPPER")

        outfile_lines.append("SET_TOLERANCE COARSE")
        outfile_lines.append("CHANGE_BUOYANCY {} {} 1".format(
            step.pickup_buoyancy,
            from_slot_pre_grasp_z + 0.1 
        ))

        outfile_lines.append("SET_TOLERANCE ULTRA_COARSE")
        #if not step.is_cinder_block and not step.is_half_block:
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=from_slot_world[0],
                y=from_slot_world[1],
                z=from_slot_high_z + 0.25,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(10.0))
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=from_slot_world[0],
                y=from_slot_world[1],
                z=from_slot_high_z + 0.25,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(10.0))

        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw}".format(
                x=from_slot_world[0],
                y=Params.center_back[1],
                z=max(Params.center_back[2], from_slot_high_z + 0.25),
                yaw=platform_yaw,
            )
        )

        #outfile_lines.append("LEFT_RIGHT_MOVE {x} {y}".format(
        #    x=from_slot_world[0],
        #    y=Params.center_back[1],
        #))

        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; center_back".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=Params.center_back[2],
                yaw=platform_yaw,
            )
        )

        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time/4.0))

        outfile_lines.append(";")
        outfile_lines.append(";;~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        outfile_lines.append("; SEQUENCE DROP {}".format(step.to_slot))

        # over target high
        drop_offset = 0
        additional_comment = ""
        if step.shift_drop_right:
            drop_offset = Params.shift_over_amount_pre_drop
            additional_comment = "to right shifted position"
        elif step.shift_drop_left:
            drop_offset = -Params.shift_over_amount_pre_drop
            additional_comment = "to left shifted position"

        outfile_lines.append("HOLD {}".format(7.0))
        outfile_lines.append("SET_TOLERANCE ULTRA_COARSE")
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; intermediate to approach drop".format(
                x=Params.center_back[0] + 0.5 * (to_slot_world[0] - Params.center_back[0]),
                y=to_slot_world[1] + drop_offset,
                z=to_slot_high_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(4.0))
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; intermediate to approach drop".format(
                x=Params.center_back[0] + 0.7 * (to_slot_world[0] - Params.center_back[0]),
                y=to_slot_world[1] + drop_offset,
                z=to_slot_high_z,
                yaw=platform_yaw,
            )
        )
        outfile_lines.append("HOLD {}".format(4.0))

        outfile_lines.append("SET_TOLERANCE COARSE")
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; {cmt}".format(
                x=to_slot_world[0],
                y=to_slot_world[1] + drop_offset,
                z=to_slot_high_z,
                yaw=platform_yaw,
                cmt=additional_comment
            )
        )
        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time))

        outfile_lines.append("SET_TOLERANCE TIGHT")
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; {cmt}".format(
                x=to_slot_world[0],
                y=to_slot_world[1] + drop_offset,
                z=to_slot_pre_drop_z,
                yaw=platform_yaw,
                cmt=additional_comment
            )
        )
        outfile_lines.append("HOLD {}".format(Params.pre_grasp_hold_time))

        preamble = "OPEN_GRIPPER"
        if step.is_cinder_block or step.is_half_block:
            preamble = "BAILING_RELEASE {} {} {}".format(
                Params.release_down_thrust_amount,
                Params.release_bail_time,
                Params.pre_open_time
            )

        if not Params.is_test_run:
            if step.shift_drop_left:
                outfile_lines.append("{} SHIFT_LEFT".format(preamble))
            elif step.shift_drop_right:
                outfile_lines.append("{} SHIFT_RIGHT".format(preamble))
            else:
                outfile_lines.append("{}".format(preamble))

        outfile_lines.append("SET_TOLERANCE COARSE")
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; {cmt}".format(
                x=to_slot_world[0],
                y=to_slot_world[1],
                z=to_slot_high_z + 0.00,
                yaw=platform_yaw,
                cmt=""
            )
        )
        outfile_lines.append("HOLD {}".format(8.0))

        #outfile_lines.append("LEFT_RIGHT_MOVE {x} {y}".format(
        #    x=after_place_surface_clear_xy[0],
        #    y=after_place_surface_clear_xy[1],
        #))
        #outfile_lines.append("LEFT_RIGHT_MOVE {x} {y}".format(
        #    x=after_place_surface_clear_xy[0] - 0.35,
        #    y=after_place_surface_clear_xy[1],
        #))
        #outfile_lines.append("CHANGE_BUOYANCY {} {} -1".format(
        #    step.after_drop_buoyancy,
        #    to_slot_pre_drop_z - 0.05 
        #))

        outfile_lines.append("SET_TOLERANCE ULTRA_COARSE")
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; {cmt}".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=to_slot_high_z,
                yaw=platform_yaw,
                cmt=""
            )
        )
        outfile_lines.append(
            "MOVE 0 {x} {y} {z} 0.0 0.0 {yaw} ; center_back".format(
                x=Params.center_back[0],
                y=Params.center_back[1],
                z=Params.center_back[2],
                yaw=platform_yaw,
            )
        )

        outfile_lines.append("HOLD {}".format(Params.after_reposition_hold_time/4.0))

    with open(output_file, "w") as f:
        file_str = "\n".join(outfile_lines)
        f.write(file_str)

def format_point(point):
    return "{:.3f},{:.3f},{:.3f}".format(point[0], point[1], point[2])

if __name__ == '__main__':
    p4 = Platform(
        to_world_tform=config.PLATFORM_TO_WORLD_MATRIX,
        manual_yaw_bump_rads=-0.01,
        #first_slot_coords=[1.654, -0.508, 0.184],
        first_slot_coords=[-0.380, -0.970, -0.508],
        platform_id=4,
    )

    print ""
    for row in p4.slot_matrix:
        print " || ".join(map(format_point, row))
    print ""

    compile_build_plan(
        #build_sequence_six_block_pyramid,
        build_sequence_wall_segment,
        #build_sequence_three_block_column,
        p4,
        "/home/sam/Dev/ros-catkin-workspace/src/droplet_underwater_assembly/param/test_compiled_build_plan.txt"
    )
    #compile_slot_scan(
    #    p4,
    #    "/home/sam/Dev/ros-catkin-workspace/src/droplet_underwater_assembly/param/test_compiled_build_plan.txt"
    #)
