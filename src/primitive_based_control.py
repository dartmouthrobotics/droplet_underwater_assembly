ACTIONS = [
    "+X", # forward on X axis
    "-X", # backward on X axis
    "NULL" # do nothing
]


# stores mapping of actions to motor speeds.
MOTORS = {
    "+X": numpy.array([0,0,0,60,0,0,60,0]), # +X is motors 4 and 7 at 60PWM
    "-X": numpy.array([0,0,0,60,0,0,60,0]), # -X is motors 4 and 7 at 60PWM
    "NULL": numpy.array([0,0,0,0,0,0,0,0])
}


# stores empirically determined mapping of primitives to accelerations
ACCELERATIONS = {
    "+X": numpy.array([1.0,0.0,0.0]), # empirically determined 3d acceleration from +X primitive (x,y,yaw)
    "-X": numpy.array([-1.0,0.0,0.0]), # empirically determined 3d acceleration from +X primitive (x,y,yaw)
    "NULL": numpy.array([0.0,0.0,0.0]) # NULL means all motors off. Zero acceleration
}

# time a motor primitive is run
PRIMITIVE_TIME = 0.1

# given the current velocity and the acceleration induced by the motion primitive, find the next predicted position 
def predict_forward(current_position, current_velocity, acceleration, thrust_time):
    predicted_position = current_position + current_velocity * thrust_time + 0.5 * (acceleration * math.pow(thrust_time, 2))

    return predicted_position


# current velocity could be from IMU integration, or delta between tag measurements
# for now, lets do delta between tag measurements
def next_action(current_velocity, current_position, goal, current_acceleration):
    # goal is the goal position (x,y,yaw)
    # current position is position from tag
    # current acceleration is acceleration measured from IMU

    best_candidate = None
    best_error = float("inf")

    # iterate over each action and select the one that moves the robot the closest
    # to the goal position
    for candidate_action in ACTIONS:
        candidate_acceleration = current_acceleration + ACCELERATIONS[candidate_action] 

        predicted_next_position = predict_forward(
            current_position,
            current_velocity,
            candidate_acceleration,
            PRIMITIVE_TIME
        )

        predicted_error = numpy.linalg.norm(
            predicted_next_position - goal
        )

        if predicted_error < best_error:
            best_error = predicted_error
            best_candidate = candidate_action

    return best_candidate
