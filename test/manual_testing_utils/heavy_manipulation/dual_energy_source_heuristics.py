from matplotlib import pyplot
import math
import numpy

NUMBER_SLOTS               = 100
BATTERY_CAPACITY           = 10.0
NUMBER_TANKS               = 5.0
TANK_TO_BATTERY_MULTIPLIER = 0.75
DISTANCE_TO_ENERGY_COST    = 0.25
PALLET_XY                  = (0.0,0.0)

slot_dist = 0.250
SLOTS = [
    (float(i) * slot_dist, 1.0) for i in range(NUMBER_SLOTS)
]



def distance(point, point2):
    total = 0.0
    for (a, b) in zip(point, point2):
        total = total + math.pow(b - a, 2)

    return math.sqrt(total)


def get_cost(from_loc, to_loc, tank_level):
    dist = distance(from_loc, to_loc)
    to_cost = DISTANCE_TO_ENERGY_COST

    return (to_cost * dist) * (1.0 - (tank_level * TANK_TO_BATTERY_MULTIPLIER))


def deplete_battery(tank_level):
    # place blocks until the battery is empty
    current_capacity = BATTERY_CAPACITY
    num_completed = 0

    remaining_tank_capacity = NUMBER_TANKS
    battery_levels = []
    battery_levels.append(current_capacity)

    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print(tank_level)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    for i, slot in enumerate(SLOTS):
        selected_tank_level = tank_level
        if tank_level > remaining_tank_capacity:
            selected_tank_level = remaining_tank_capacity

        remaining_tank_capacity = remaining_tank_capacity - selected_tank_level

        capacity_depletion = get_cost(PALLET_XY, slot, selected_tank_level)
        print("Depletion: ", capacity_depletion)
        current_capacity = current_capacity - capacity_depletion
        print("Capacity: ", current_capacity)
        print("Number ", i)
        print("")
        if current_capacity > 0.0:
            battery_levels.append(current_capacity)
            num_completed = i
        else:
            print("Killed", num_completed)
            break
    print("*****************************************")

    return int(num_completed), battery_levels

tank_level_samples = numpy.arange(0.0, 0.5, 0.05)
slot_samples = [
    deplete_battery(level) for level in tank_level_samples
]

for i in slot_samples:
    print(slot_samples)
    print("")

for idx, sample in enumerate(slot_samples):
    pyplot.plot(sample[1], label=str(tank_level_samples[idx]))

pyplot.title("Number of blocks with different tank levels")
pyplot.xlabel("# blocks placed")
pyplot.ylabel("Battery level")
pyplot.legend()
pyplot.show()
