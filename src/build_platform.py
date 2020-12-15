import assembly_action
import collections
import config
import utils
import numpy
import tf.transformations


class BuildPlatform(object):
    """
    Abstraction for a build platform. The slot number (0,0) is the lowest X and lowest Z.
    This means the farthest from the tag and the lowest down. Slots are numbered
    """
    def __init__(self, tag_id, slot_dimensions, bottom_back_left_slot_location, dimensions, center_back_pose):
        self.roll_pitch_estimate = [0.0, 0.0] # unused
        self.frame_id = "/platform_{}".format(tag_id)
        self.tag_id = tag_id
        self.center_back_pose = center_back_pose
        self.slot_matrix = self.construct_slot_matrix(
            platform_dimensions=dimensions,
            bottom_back_left_location=bottom_back_left_slot_location,
            slot_dimensions=slot_dimensions
        )


    def visualize_slots(self):
        pass


    def get_slot_coordinates(self, indices):
        """
        returns xyzrpy location for the given slot. indices is an indexable with format (x,y,z)
        """
        return self.slot_matrix[indices[2]][indices[1]][indices[0]]


    def construct_slot_matrix(self, platform_dimensions, slot_dimensions, bottom_back_left_location):
        slot_matrix = []

        for z_index in range(platform_dimensions[2]):
            z_value = bottom_back_left_location[2] + slot_dimensions[2] * z_index
            current_z_level = []

            for y_index in range(platform_dimensions[1]):
                y_value = bottom_back_left_location[1] + slot_dimensions[1] * y_index
                current_y_level = []

                for x_index in range(platform_dimensions[0]):
                    x_value = bottom_back_left_location[0] + slot_dimensions[0] * x_index
                    current_y_level.append([x_value, y_value, z_value, 0.0, 0.0, 0.0])

                current_z_level.append(current_y_level)

            slot_matrix.append(current_z_level)

        return slot_matrix
