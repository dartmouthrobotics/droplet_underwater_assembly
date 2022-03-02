def construct_tolerance_motion_experiment(cube_dimensions, cube_center, n_samples_per_tol, tol_step, tol_range, cutoff_seconds):
    actions = []

    initial_tolerance = [
        tol_range[1],
        tol_range[1],
        0.03,
        1000.0,
        1000.0,
        0.017
    ]

    tol_samples = []

    n_tolerance_steps = int((tol_range[1] - tol_range[0]) / tol_step)

    for tol_step_x in range(n_tolerance_steps):
        for tol_step_y in range(n_tolerance_steps):
            tol_samples.append(
                [
                    initial_tolerance[0] - float(tol_step_x) * tol_step,
                    initial_tolerance[1] - float(tol_step_y) * tol_step,
                    initial_tolerance[2],
                    initial_tolerance[3],
                    initial_tolerance[4],
                    initial_tolerance[5],
                ]
            )

    rospy.loginfo("Sampling {} tolerances".format(tol_samples))

    goal_moves_with_tol = []
    
    pt_1 = list(cube_center)
    pt_1[1] = pt_1[1] + 0.20
    pt_2 = list(cube_center)
    pt_2[0] = pt_2[0] + 0.20

    move_triangle = [
        cube_center,
        pt_1,
        pt_2
    ]

    assert(n_samples_per_tol == 3)
    for sample in tol_samples:
        for goal_idx in range(n_samples_per_tol):
            next_point = move_triangle[goal_idx]

            goal_move = [
                next_point[0],
                next_point[1],
                next_point[2],
                0.0,
                0.0,
                0.0,
            ]

            goal_moves_with_tol.append(
                (goal_move, sample)
            )

    rospy.loginfo("Number moves {}".format(len(goal_moves_with_tol)))
    new_action = assembly_action.AssemblyAction(
        action_type="move",
        goal_pose=[move_triangle[2][0], move_triangle[2][1], move_triangle[2][2], 0.0, 0.0, 0.0],
        pose_tolerance=initial_tolerance,
        position_hold_time=6.0
    )

    for move, tol in goal_moves_with_tol:
        new_action = assembly_action.AssemblyAction(
            action_type="move",
            goal_pose=move,
            pose_tolerance=tol,
            position_hold_time=6.0
        )

        actions.append(new_action)

        new_action = assembly_action.AssemblyAction(
            action_type="move",
            goal_pose=move,
            pose_tolerance=tol,
            position_hold_time=0.0
        )

        new_action.timeout = cutoff_seconds
        actions.append(new_action)

    for idx, action in enumerate(actions):
        action.high_level_build_step = "TOL_SAMPLING"
        action.sequence_id = idx

    return actions


ACTIONS = construct_tolerance_motion_experiment(
    cube_dimensions=[0.5,0.5,0.5,0.15],
    cube_center=[-1.6, -0.3, -0.3, 0.0],
    n_samples_per_tol=3,
    tol_step=0.0025,
    tol_range=[0.0075,0.03],
    cutoff_seconds=90.0
)
