import tf

class BuildPlatform(object):
    def __init__(self, config):
        # bakes in the assumption that the slots are in a line
        self.slot_length = config["build_platform_config"]["slot_length"]
        self.slot_height = config["build_platform_config"]["slot_height"]

        self.number_pickup_rows = config["build_platform_config"]["number_pickup_rows"]
        self.number_pickup_columns = config["build_platform_config"]["number_pickup_columns"]
        
        self.number_dropoff_rows = config["build_platform_config"]["number_dropoff_rows"]
        self.number_dropoff_columns = config["build_platform_config"]["number_dropoff_columns"]

        self.transform_broadcaster = tf.TransformBroadcaster()

        self.marker_id = config["build_platform_config"]["marker_id"]
        self.frame_id = config["build_platform_config"]["frame_id"]

    def send_to_tf_tree(self, transform_broadcaster):
        # sends slots to the tf tree
        pass