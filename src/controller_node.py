import rospy
import droplet_underwater_assembly.srv

from droplet_underwater_assembly_libs import trajectory_tracker

class ControllerNode(object):
    def __init__(self):
        self.set_strategy_service = rospy.Service('set_strategy', droplet_underwater_assembly.srv.SetControlStrategy, self.set_control_strategy)
        self.set_target_service = rospy.Service('set_target', droplet_underwater_assembly.srv.SetControlTarget, self.set_target)
        self.location_stream_subscriber = rospy.Subscriber('location_stream', droplet_underwater_assembly.msg.LocationStream, self.location_stream_callback)

        self.current_pose = None

    def location_stream_callback(self, msg):
        pass

    def set_target(self, request):
        self.target = request.target

    def parse_gains_dict(self, request_gains):
        pass

    def set_control_strategy(self, request):
        self.active_gains = self.parse_gains_dict(request.gains)

    def run(self):
        pass

if __name__ == '__main__':
    controller_node = ControllerNode()
    controller_node.run()