from matplotlib import pyplot
import csv
import pandas
import numpy
import itertools
import matplotlib.colors

csv_file = "Manual-dropping-experiment-2-17-2021.csv"


def is_surprising(x,y,success):
    success = success.lower()
    if x in range(2,5) and y in range(2,5) and success == "yes":
        return False

    if x in range(2,5) and y in range(2,5) and success == "no":
        return True

    if success != "no":
        return True

    return False

def yes_no_to_float(val):
    val = val.lower()
    if val == "no":
        return 0.0

    if val == "yes":
        return 1.0

    raise Exception("AAAAA {}".format(val))

def get_surprising_drop_results(csv_file, height_selector):
    data_9_75_in = []
    with open(csv_file) as f:
        reader = csv.reader(f, delimiter=",", quotechar='"')
             
        for row in reader:
            if row[0] == height_selector:
                data_for_row = []
                location = list(map(int, row[1].split(",")))

                if location[1] in range(1,6):
                    data_for_row.append(
                        location
                    )
                    data_for_row.append(row[5])

                    data_9_75_in.append(data_for_row) 

        print(data_9_75_in)
        print(len(data_9_75_in))

    current_y_val = 1

    rows = []
    for y_val, y_val_data in itertools.groupby(data_9_75_in, lambda r: r[0][1]):
        row = []

        for x_val, x_val_data in itertools.groupby(y_val_data, lambda r: r[0][0]):
            data = list(x_val_data)

            if len(data) != 2:
                print("Bad data row :((((")

            x_coord_trial_0 = data[0][0]
            y_coord_trial_0 = data[0][1]

            x_coord_trial_1 = data[1][0]
            y_coord_trial_1 = data[1][1]

            surprise_0 = is_surprising(x_coord_trial_0,y_coord_trial_0,data[0][1])
            surprise_1 = is_surprising(x_coord_trial_1,y_coord_trial_1,data[1][1])

            x_result = None
            if surprise_0 and not surprise_1:
                x_result = yes_no_to_float(data[0][1])

            if surprise_1 and not surprise_0:
                x_result = yes_no_to_float(data[1][1])

            if surprise_1 and surprise_0:
                assert(data[0][1] == data[1][1])
                x_result = yes_no_to_float(data[1][1])

            x_result = yes_no_to_float(data[1][1])

            row.append(x_result)

        rows.append(row)

    return rows

data_exp = []
for y in range(1,6):
    row = []
    for x in range(7):
        if x in range(2,5) and y in range(2,5):
            row.append(1.0)
        else:
            row.append(0.0)

    data_exp.append(row)

color_list = ['silver', 'lime']
colors = matplotlib.colors.ListedColormap(color_list)

fig, ax = pyplot.subplots(nrows=1, ncols=3, figsize=(5.5, 2.4))
xlabels = [-3,-2,-1,0,1,2,3]
xlabels = ["{:.0f}".format(x * 2.54) for x in xlabels]
ax_exp = ax[0]
ax[0].scatter(0,0,c=color_list[0], zorder=0, label="Fail")
ax[0].scatter(0,0,c=color_list[1], zorder=0 , label="Success")
ax[0].legend(loc=(1.35,-0.80))
#ax[0].legend(loc="upper left")
ax_exp.imshow(numpy.array(data_exp), zorder=1, cmap=colors)
ax_exp.set_xticks(list(range(7)))
ax_exp.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
ax_exp.set_xticks(numpy.arange(-0.5,7,1))
ax_exp.set_yticks(numpy.arange(-0.5,5,1))
ax_exp.set_xticklabels(xlabels)
ax_exp.set_yticklabels(["{:.0f}".format(y) for y in [-2*2.54,-1*2.54,0,1*2.54,2*2.54]])
ax_exp.yaxis.set_tick_params(width=0)
ax_exp.xaxis.set_tick_params(width=0)
ax_exp.set_ylabel("Error (cm)")
ax_exp.set_title("Ideal")

ax_res = ax[2]
ax_res.set_title("{:.0f} cm".format(9.75*2.54))
ax_res.set_xticks(numpy.arange(-0.5,7,1))
ax_res.set_yticks(numpy.arange(-0.5,5,1))
ax_res.set_xticklabels(xlabels)
#ax_res.set_yticklabels([-2,-1,0,1,2])
ax_res.xaxis.set_tick_params(width=0)
ax_res.yaxis.set_tick_params(width=0)
ax_res.set_yticklabels([])
ax_res.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)

rows = get_surprising_drop_results(csv_file, "9.75 in")
rows = numpy.array(rows)
ax_res.imshow(rows, cmap=colors)

rows = get_surprising_drop_results(csv_file, "5.0 in")
ax_5_in = ax[1]
ax_5_in.imshow(rows, cmap=colors)
ax_5_in.set_title("{:.0f} cm".format(5.0 * 2.54))
ax_5_in.set_xticks(numpy.arange(-0.5,7,1))
ax_5_in.set_yticks(numpy.arange(-0.5,5,1))
ax_5_in.set_xticklabels(xlabels)
#ax_5_in.set_yticklabels([-2,-1,0,1,2])
ax_5_in.yaxis.set_tick_params(width=0)
ax_5_in.xaxis.set_tick_params(width=0)
ax_5_in.set_yticklabels([])
ax_5_in.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)

fig.savefig("manual_dropping_results.pdf")

pyplot.show()
