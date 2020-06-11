import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot
import subprocess
import datetime
import os
import rospkg


POSITION_AXIS_LABELS = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]


def get_goal_crossing_point_index(history):
    for (previous_idx, current_idx) in zip(range(len(history)-1), range(1,len(history))):
        previous_sign = history[previous_idx]
        current_sign = history[current_idx]

        if (previous_sign < 0.0 and current_sign >= 0.0) or (previous_sign >= 0.0 and current_sign < 0.0):
            return current_idx

    return None


def get_overshoot(error_history):
    crossing_point_index = get_goal_crossing_point_index(error_history)

    if crossing_point_index is None:
        return 0.0

    containing_interval = [float("inf"), float("-inf")]
    for value in error_history[crossing_point_index:]:
        if value < containing_interval[0]:
            containing_interval[0] = value

        if value > containing_interval[1]:
            containing_interval[1] = value

    return abs(containing_interval[1] - containing_interval[0])


def get_average_frame_rate(marker_history):
    number_samples = float(len(marker_history))

    total_time_gap = 0.0
    for (previous, current) in zip(range(0,len(marker_history)-1), range(1, len(marker_history))):
        previous_marker = marker_history[previous].header.stamp
        current_marker = marker_history[current].header.stamp

        time_gap = (current_marker - previous_marker).to_sec()

        total_time_gap = total_time_gap + time_gap

    return 1.0 / (total_time_gap / number_samples)


def write_str_to_file(string, file_name):
    f = open(file_name, "w")
    f.write(string)
    f.close()


def write_history_csv_files(output_directory, position_history, error_history, response_history, marker_history, imu_history):
    position_history_data = [
       ",".join(
           ["{0:.3f}".format(axis) for axis in pos]
        ) for pos in position_history
    ]

    error_history_data = [
       ",".join(
           ["{0:.3f}".format(axis) for axis in pos]
        ) for pos in error_history
    ]

    response_history_data = [
       ",".join(
            ["{0}".format(channel) for channel in response.channels]
        ) for response in response_history
    ]

    imu_history_data = [
        ",".join(
            [
                "{0:.3f}".format(imu_message.linear_acceleration.x),
                "{0:.3f}".format(imu_message.linear_acceleration.y),
                "{0:.3f}".format(imu_message.linear_acceleration.z), 
                "{0:.3f}".format(imu_message.angular_velocity.x),
                "{0:.3f}".format(imu_message.angular_velocity.y),
                "{0:.3f}".format(imu_message.angular_velocity.z), 
            ]
        ) for imu_message in imu_history
    ]

    position_history_csv_str = "\n".join(position_history_data)
    error_history_csv_str = "\n".join(error_history_data)
    response_history_csv_str = "\n".join(response_history_data)
    imu_history_csv_str = "\n".join(imu_history_data)

    position_history_outfile = "{output_directory}/position_history.csv".format(output_directory=output_directory)
    error_history_outfile = "{output_directory}/error_history.csv".format(output_directory=output_directory)
    response_history_outfile = "{output_directory}/response_history.csv".format(output_directory=output_directory)
    imu_history_outfile = "{output_directory}/imu_history.csv".format(output_directory=output_directory)

    write_str_to_file(position_history_csv_str, position_history_outfile)
    write_str_to_file(error_history_csv_str, error_history_outfile)
    write_str_to_file(response_history_csv_str, response_history_outfile)
    write_str_to_file(imu_history_csv_str, imu_history_outfile)


def report_results(vehicle_config, position_history, response_history, error_history, goal_position, notes, output_directory, data_collection_time, report_output, marker_history, imu_history):
    overshoots = {}

    position_plots = []
    motor_speed_plots = []

    for (axis_index, axis_name) in enumerate(POSITION_AXIS_LABELS):
        fig, ax = pyplot.subplots()

        axis_goal = goal_position[axis_index]
        axis_positions = [position[axis_index] for position in position_history]
        axis_errors = [error[axis_index] for error in error_history]

        ax.plot(axis_positions, label=axis_name)
        ax.axhline(y=goal_position[axis_index], color="r", label="goal ({0:.3f})".format(axis_goal))

        ax.set_title("{} position".format(axis_name))
        ax.legend()

        pos_plot_output = "{output_directory}/{axis_name}_position_v_goal.svg".format(axis_name=axis_name, output_directory=output_directory)
        fig.savefig(pos_plot_output)

        position_plots.append(pos_plot_output)

        overshoots[axis_name] = get_overshoot(axis_errors)

    for motor_index in range(8):
        motor_speed_history = [response.channels[motor_index] for response in response_history]

        fig, ax = pyplot.subplots()

        ax.plot(motor_speed_history, label="pwm")
        ax.set_title("Motor {} speed history".format(motor_index + 1))
        ax.legend()

        motor_speed_output = "{output_directory}/motor_{motor_index}_history.svg".format(
            motor_index=motor_index,
            output_directory=output_directory
        )

        motor_speed_plots.append(motor_speed_output)

        fig.savefig(
            motor_speed_output
        )

    marker_frame_rate = get_average_frame_rate(marker_history)

    overshoot_stats = []
    for key, val in overshoots.items():
        overshoot_stats.append("    {}: {}".format(key, val))

    run_stats = [
        "Notes: {}".format(notes),
        "Timestamp: {}".format(datetime.datetime.now().strftime("%a %b %d %Y, %I:%M %p")),
        "Run duration: {} seconds".format(data_collection_time),
        "Number position samples: {}".format(len(position_history)),
        "Marker frame rate (average): {}".format(marker_frame_rate),
        "Overshoot: ",
    ]
    run_stats.extend(overshoot_stats)
    
    run_stats_str = "\n".join(run_stats)

    write_str_to_file(run_stats_str, report_output)

    write_history_csv_files(
        output_directory=output_directory,
        position_history=position_history,
        error_history=error_history,
        response_history=response_history,
        marker_history=marker_history,
        imu_history=imu_history
    )
