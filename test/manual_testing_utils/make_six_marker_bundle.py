import numpy as np

tag_side_length = 0.169333333
tag_spacing_horizontal = 0.0254
tag_spacing_vertical = 0.042418

n_tags_horizontal = 3
n_tags_vertical = 2

# order for bundle: top right, bottom left, bottom right, top left

bundle_shape = [
    [0, 1, 2],
    [3, 9, 8]
]

def get_tag_corners(top_left):
    tag_center = top_left + np.array([tag_side_length/2.0, -tag_side_length/2.0])
    return [ # ccw order
        tag_center + np.array([tag_side_length/2.0, tag_side_length/2.0]),
        tag_center + np.array([-tag_side_length/2.0, -tag_side_length/2.0]),
        tag_center + np.array([tag_side_length/2.0, -tag_side_length/2.0]),
        tag_center + np.array([-tag_side_length/2.0, tag_side_length/2.0]),
    ]

# +x to right
# +y up

def get_marker_bundle_item_from_corners(corners, tag_id):
    corner_order = [0, 1, 2, 3]
    corners_formatted = []

    for c_num in corner_order:
        corners_formatted.append(
            [corners[c_num][0], corners[c_num][1], 0.0]
        )

    return {
        "corners": corners_formatted,
        "id": tag_id
    }

def print_marker_bundle(tags):
    output = "[\n"

    for i, m in enumerate(tags):
        end = "\n"
        if i < len(tags) - 1:
            end = ",\n"

        output = output + "    " + str(m) + end

    output = output + "]"
    print(output)

bundleized_tags = []

for horiz_num in range(n_tags_horizontal):
    for vert_num in range(n_tags_vertical):
        top_left = np.array([0.0, 0.0])
        x_offset = np.array(
            [
                (float(horiz_num) * tag_spacing_horizontal) + (float(horiz_num) * tag_side_length),
                0.0
            ]
        )
        y_offset = np.array(
            [
                0.0,
                (-float(vert_num) * tag_spacing_vertical) - (float(vert_num) * tag_side_length),
            ]
        )

        tag_id = bundle_shape[vert_num][horiz_num]
        top_left = top_left + x_offset + y_offset
        corners = get_tag_corners(top_left)

        bundleized_tags.append(get_marker_bundle_item_from_corners(
            corners, tag_id
        ))

bundleized_tags.sort(key=lambda x: x['id'])
print_marker_bundle(bundleized_tags)
