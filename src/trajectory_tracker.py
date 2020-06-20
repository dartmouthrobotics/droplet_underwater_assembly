import collections
import utils

NUMBER_MOTORS = 8

class MotionPrimitive(object):
    def __init__(self, name, motor_speeds, acceleration):
        self.name = name
        self.motor_speeds = motor_speeds
        self.acceleration = acceleration

        assert(len(motor_speeds) == NUMBER_MOTORS)

    @property
    def as_rc_override(self):
        stop_message = utils.construct_stop_rc_message()
        return utils.construct_rc_message(self.motor_speeds)


class PredictivePositionTracker(object):
    # starting with 1-d
    def __init__(self, motion_primitives, forward_prediction_time):
        self.current_acceleration = None
        self.current_position = None
        self.current_velocity = None
        self.goal_position = None

        self.forward_prediction_time = forward_prediction_time
        self.motion_primitives = motion_primitives


    def set_current_velocity(self, velocity):
        self.current_velocity = velocity


    def set_current_acceleration(self, current_acceleration):
        self.current_acceleration = current_acceleration


    def set_goal_position(self, goal_position):
        self.goal_position = goal_position


    def predict_primitive_effect(self, primitive):
        predicted_acceleration = self.current_acceleration + primitive.acceleration
        predicted_next_position = self.current_position + (self.current_velocity * self.forward_prediction_time) + predicted_acceleration * self.forward_prediction_time ** 2 

        return predicted_next_position

    def get_next_primitive():
        best_next_action = None
        best_next_error = float("inf")

        for primitive in self.motion_primitives:
            predicted_result = self.predict_primitive_effect(primitive)

            predicted_error = self.get_error(self.goal_position, predicted_result)

            if predicted_error < best_next_error:
                best_next_error = predicted_error
                best_next_action = primitive

        return primitive


class SinglePrimitiveTracker(object):
    def __init__(self, primitives):
        self.primitives = primitives

    def set_goal_position(self, _):
        pass

    def set_current_position(self, _):
        pass

    def get_next_motion_primitive(self):
        return self.primitives[0]


class BinaryTrajectoryTracker(object):
    def __init__(self, primitives):
        self.goal_position = None
        self.current_position = None
        self.primitives = primitives

    def set_goal_position(self, goal):
        self.goal_position = goal

    def set_current_position(self, position):
        self.current_position = position

    def get_error(self):
        return utils.get_error(self.current_position, self.goal_position)

    def get_next_motion_primitive(self):
        error = self.get_error()

        if abs(error[5]) >= 0.05:
            if error[5] < 0:
                return [primitive for primitive in self.primitives if primitive.name == "+Yaw"][0]

            return [primitive for primitive in self.primitives if primitive.name == "-Yaw"][0]

        if abs(error[0]) < abs(error[1]):
            if error[1] < 0:
                return [primitive for primitive in self.primitives if primitive.name == "+Y"][0]

            return [primitive for primitive in self.primitives if primitive.name == "-Y"][0]

        # X axis
        if error[0] > 0:
            return [primitive for primitive in self.primitives if primitive.name == "+X"][0]

        return [primitive for primitive in self.primitives if primitive.name == "-X"][0]

