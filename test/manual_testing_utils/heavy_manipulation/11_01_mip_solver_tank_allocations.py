import mip
import numpy
import math
import random
from matplotlib import pyplot, patches


class Constants(object):
    tank_capacity = 20.0
    battery_capacity = 30.0
    raw_battery_consumption_per_meter = 2.0

    tank_levels = numpy.arange(0.0, 1.01, 0.1)
    tank_consumption_per_level = 5.0
    tank_energy_savings_max = 0.80

    slot_stride = 0.5
    number_slots = 30

    pallet_locations = [
        [0.0, 0.0]
    ]

    @classmethod
    def get_tank_modifier(cls, tank_level):
        return (tank_level**2) * cls.tank_energy_savings_max


class Scene(object):
    def __init__(self, slots, pallets):
        self.slots = slots
        self.pallets = pallets


class Slot(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other):
        return math.sqrt(
            math.pow(self.x - other.x, 2.0) + math.pow(self.y - other.y, 2.0)
        )

    def to_solver_name(self, slot_idx, level_idx):
        return 'slot_{}_tank_{}'.format(slot_idx, level_idx)



class Pallet(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


def initialize_scene_single_pallet_line_of_slots():
    # just does a line
    slots = []
    for i in range(Constants.number_slots):
        random.seed()
        slots.append(
            Slot(random.uniform(0.0, 20.0), random.uniform(1.0, 2.0))
        )

    print("Constructed slots ", slots)

    pallet = Pallet(
        Constants.pallet_locations[0][0],
        Constants.pallet_locations[0][1]
    )

    scene = Scene(
        slots=slots,
        pallets=[pallet]
    )

    return scene
        

def construct_model(scene):
    solver_model = mip.Model(sense=mip.MAXIMIZE, solver_name=mip.CBC)
    batt_cost_matrix = []
    tank_cost_matrix = []
    indicator_var_matrix = []

    # indicator variables
    for slot_idx, slot in enumerate(scene.slots):
        slot_indicators = []
        batt_costs = []
        tank_costs = []

        for (level_idx, tank_level) in enumerate(Constants.tank_levels):
            var = solver_model.add_var(
                var_type=mip.BINARY,
                name=slot.to_solver_name(slot_idx, level_idx)
            )
            slot_indicators.append(var)

            pallet_distance = slot.distance_to(scene.pallets[0])
            batt_depletion_multiplier = Constants.raw_battery_consumption_per_meter * (1.0 - Constants.get_tank_modifier(tank_level))

            batt_depletion = batt_depletion_multiplier * pallet_distance
            tank_depletion = Constants.tank_consumption_per_level * tank_level

            batt_costs.append(batt_depletion)
            tank_costs.append(tank_depletion)


        batt_cost_matrix.append(batt_costs)
        tank_cost_matrix.append(tank_costs)
        indicator_var_matrix.append(slot_indicators)

        # limits to single tank choice
        solver_model += mip.xsum(slot_indicators) <= 1, "slot_{}_single_choice".format(slot_idx)
    
    num_levels = len(indicator_var_matrix[0])
    num_slots = len(indicator_var_matrix)
    print("Tank costs")
    for x in tank_cost_matrix:
        print(x)

    print("Batt costs")
    for x in batt_cost_matrix:
        print(x)

    # battery cost constraint
    solver_model += mip.xsum(indicator_var_matrix[i][j] * batt_cost_matrix[i][j] for i in range(num_slots) for j in range(num_levels)) <= Constants.battery_capacity, "battery_capacity"

    # tank cost constraint
    solver_model += mip.xsum(indicator_var_matrix[i][j] * tank_cost_matrix[i][j] for i in range(num_slots) for j in range(num_levels)) <= Constants.tank_capacity, "tank_capacity"

    # maximize number selected
    solver_model.objective = mip.xsum(indicator_var_matrix[i][j] for i in range(num_slots) for j in range(num_levels))

    return solver_model


def draw_results(scene, model):
    pallet = scene.pallets[0]
    color_by_level = [
        'black',
        'cyan',
        'red',
        'green',
        'blue',
        'pink',
        'yellow',
        'lime',
        'grey',
        'magenta',
        'magenta'
    ]

    levels_labelled = [False for _ in Constants.tank_levels]

    pyplot.scatter(
        [slot.x for slot in scene.slots],
        [slot.y for slot in scene.slots], 
        label="slots",
        zorder=1,
        s=40
    )
    pyplot.scatter(
        [pallet.x], [pallet.y], label="pallet",
        zorder=2,
        s=40
    )

    for slot_idx in range(Constants.number_slots):
        slot = scene.slots[slot_idx]
        for level_idx in range(len(Constants.tank_levels)):
            solver_var_name = scene.slots[slot_idx].to_solver_name(
                slot_idx, level_idx
            )
            solver_var = model.var_by_name(solver_var_name)

            if solver_var.x == 1.0:
                level_label = None

                if levels_labelled[level_idx] == False:
                    level_label = "tank filled {:.1f}%".format(
                        Constants.tank_levels[level_idx] * 100.0
                    )
                    levels_labelled[level_idx] = True

                pyplot.plot(
                    [pallet.x, slot.x],
                    [pallet.y, slot.y],
                    color=color_by_level[level_idx],
                    label=level_label,
                    zorder=0
                )


    pyplot.xlabel("Meters")
    pyplot.ylabel("Meters")
    pyplot.title("Tank allocations to maximize number of blocks placed.")
    pyplot.legend()
    pyplot.show()


def main():
    scene = initialize_scene_single_pallet_line_of_slots()
    model = construct_model(scene)
    status = model.optimize(max_seconds=120.0)

    if status == mip.OptimizationStatus.OPTIMAL or status == mip.OptimizationStatus.FEASIBLE:
        draw_results(scene, model)
    else:
        print("No solution found...")

if __name__ == "__main__":
    main()
