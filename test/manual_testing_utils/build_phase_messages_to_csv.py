import rospy
import droplet_underwater_assembly.msg

OUTPUT_FILE = "tolerance_timing_test.csv"

build_phase_topic = "/droplet_underwater_assembly/build_phase"
inches_away = 2.25

header = "x,y,z,roll,pitch,yaw,inches_away"

fd = None

def build_phase_callback(build_phase_message):
    cp = build_phase_message.current_location
    csv_line = list(cp)
    csv_line.append(inches_away)
    fd.write(",".join(map(str, csv_line)) + "\n")

def main():
    global fd
    rospy.init_node("csv_storer")

    fd = open(OUTPUT_FILE, "a") 

    subscriber = rospy.Subscriber(
        build_phase_topic,
        droplet_underwater_assembly.msg.BuildPhase,
        build_phase_callback
    )

    rospy.spin()

    fd.close()

if __name__ == '__main__':
    main()
