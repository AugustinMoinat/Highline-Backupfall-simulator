from line_and_slacker import Webbing, Segment
# Implement different webbing types to be used later

nylon_heavy = Webbing(0.075, 0.18, 10000, 0.1)
nylon_light = Webbing(0.05, 0.2, 10000, 0.1)
polyester_heavy = Webbing(0.07, 0.04, 10000, 0.1)
polyester_light = Webbing(0.05, 0.1, 10000, 0.1)
Kill_Bill = Webbing(0.15, 0.02, 10000, 0.2)
dyneema_light = Webbing(0.03, 0.02, 10000, 0.05)
dyneema = Webbing(0.05, 0.005, 10000, 0.1)



# Define a section of line (main, length, backup, length, number of modelisation points)

p = Segment(polyester_light,50,polyester_light,60,10)
s = Segment(nylon_heavy,85,nylon_light,95,10)
n = Segment(nylon_light,30,dyneema,34,10)
d = Segment(dyneema_light,50,dyneema_light,53,10)
s2 = Segment(nylon_heavy,100,nylon_light,112,10)
f = Segment(nylon_light,30,dyneema_light,33.5,10)


# Define a setup as a list of sections

freestyle = [f, f]
length_freestyle = 64
backup_phi = [Segment(polyester_light,257,polyester_light,275,25)]
length_backup_phi = 262
world_record_setup = [d for i in range(27)]
world_record_length = 2717



maxt = 10000  # maxt is used for the color scale (white: 0 N -> red: maxt N)

setup = freestyle

# Define a spot (has to  be consistent with length)

length = length_freestyle
height = 30  # if height > length/2, will not show on display

# Slackliner's characteristics
leg_length = 1
weight = 82
leash_length = 1.5
power = 2.5  # fraction of body weight that the legs can hold.
position = [1, 5]  # section number and model unit in that section

# Define time computation parameters
dt = 0.01   # time increments for measure and display update
steps = 50   # number of computing steps between each test point
# From experiment, it works well when steps/dt > 5000 Hz
settle_time = 10  # time for the line to settle before putting the highliner on it

max_steps = -1   # Negative values : no max time.
