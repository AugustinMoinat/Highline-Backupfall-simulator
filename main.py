import os
from line_and_slacker import *
from data_visualisation import *
from parameters import *
from datetime import datetime

# General folder of results:
general = './results/'
if not os.path.exists(general):
    os.mkdir(general)
# Name for the files. It is set to show the date (unique for all tests)
folder = general + datetime.now().strftime("%y-%m-%d-%H-%M-%S/")
os.mkdir(folder)

load = False # either load a previous save state, or calculate a line with parameters from parameters.py
# This is the main function execution
if load:
    save_file = './save_states/world_record.pk'  # save state path, must be a pickle file
    n, data = load_and_play(save_file, dt, steps, max_steps, folder)
else:
    n, data = run_and_play(length, height, setup, maxt, leg_length, weight, leash_length,
                           power, position, dt, steps, max_steps, folder, settle_time)

# Data analysis: producing graphs
data_visual(data, folder, n, True, True)  # Bool parameters: display and save tension graphs
