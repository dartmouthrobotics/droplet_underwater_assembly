# what would be a good way to show this?

phase_times = {
    "approach_ultra_coarse": 0.0,
    "approach_coarse": 3.0,
    "approach_fine": 10.0,
    "close_gripper": 20.0,
    "approach_coarse": 25.0,
    "approach_ultra_coarse": 35.0
    "approach_coarse": 45.0,
    "approach_fine": 55.0,
    "open_gripper": 65.0,
    "approach_coarse": 75.0,
    "approach_ultra_coarse": 90.0
}

# need to record the timing of the phases, so how can we do that? We can just know which locations belong to which
# so first, we can just run the code to get the waypoints and record the phases like that
