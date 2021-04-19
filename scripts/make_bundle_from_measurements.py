# makes a marker bundle
import json

marker_width = (5.0 * 25.4) / 1000.0 # width in meters
vertical_spacing = (0.5 * 25.4) / 1000.0
horizontal_spacing = (0.5 * 25.4) / 1000.0

# top right
# bottom left
# bottom right
# top left

markers = [
    {
        "id": 4,
        "corners": [
            [marker_width, 0.0, 0.0],
            [0.0, -marker_width, 0.0],
            [marker_width, -marker_width, 0.0],
            [0.0, 0.0, 0.0]
        ],
    },
    {
        "corners": [
            [marker_width, -marker_width - vertical_spacing, 0.0],
            [0.0, (-marker_width - vertical_spacing) - marker_width, 0.0],
            [marker_width, (-2.0 * marker_width) - vertical_spacing, 0.0],
            [0.0, -marker_width - vertical_spacing, 0.0],

        ],
        "id": 5
    },
    {
        "id": 6,
        "corners": [
            [2.0 * marker_width + horizontal_spacing, -marker_width - vertical_spacing, 0.0],
            [marker_width + horizontal_spacing, -2.0 * marker_width - vertical_spacing, 0.0],
            [marker_width + horizontal_spacing + marker_width, -2.0 * marker_width - vertical_spacing, 0.0],
            [1.0 * marker_width + horizontal_spacing, -marker_width - vertical_spacing, 0.0],
        ]
    },
    {
        "id": 7,
        "corners": [
            [marker_width + marker_width + horizontal_spacing, 0.0, 0.0],
            [marker_width + horizontal_spacing, -marker_width, 0.0],
            [marker_width + marker_width + horizontal_spacing, -marker_width, 0.0],
            [marker_width + horizontal_spacing, 0.0, 0.0]
        ]
    }
]

print json.dumps(markers, indent=4)
