import rospy
import rosbag
import math
from matplotlib import pyplot

air_times = [1.0, 1.5, 2.0]

directory = "/media/sam/storage/uw-assembly-experiments/heavy_manipulation/"

bag_files = [
    "10_26_2021_1_0_sec_air_18_mark_lateral.bag",
    "10_26_2021_1_5_sec_air_24_mark_lateral.bag",
    "10_26_2021_2_0_sec_air_all_mark_lateral.bag",
]

lateral_motors = [1, 2, 3, 5]
updown_motors = [0, 4, 6, 7]
all_motors = [0, 1, 2, 3, 4, 5, 6, 7]

selected_motors = lateral_motors


def sublist(to_sublist, indices):
    return [
        to_sublist[x] for x in indices
    ]


def l2_norm(vector):
    return math.sqrt(
        sum(math.pow(i, 2) for i in vector)
    )


def get_pwm_magnitude(pwm):
    return abs(pwm - 1500)


def get_pwm_signals(bag_file):
    pwm_readings = []
    for topic, message, time in bag_file.read_messages(topics=['/mavros/rc/override']):
        pwm_readings.append(
            [get_pwm_magnitude(x) for x in message.channels]
        )

    return pwm_readings


def filter_and_get_l2_norm(pwm_signals):
    return list(
        map(
            l2_norm,
            [sublist(x, selected_motors) for x in pwm_signals]
        )
    )


signals_1_5 = get_pwm_signals(
    rosbag.Bag(directory + bag_files[0])
)

signals_1_7 = get_pwm_signals(
    rosbag.Bag(directory + bag_files[1])
)

signals_2_0 = get_pwm_signals(
    rosbag.Bag(directory + bag_files[2])
)

pyplot.plot(
    filter_and_get_l2_norm(signals_1_5),
    label="1.5 seconds air"
)

pyplot.plot(
    filter_and_get_l2_norm(signals_1_7),
    label="1.7 seconds air"
)

pyplot.plot(
    filter_and_get_l2_norm(signals_2_0),
    label="2.0 seconds air"
)

pyplot.title("PWM signal strength for lateral motors")
pyplot.ylabel("PWM Magnitude (l2-norm)")
pyplot.xlabel("Fourtieths of seconds")
pyplot.legend()
pyplot.show()
