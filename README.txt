This is a project to modelise backup falls and general dynamics in highlines.
Created by Augustin Moinat for International Slackline Association purposes.

To run the modelisation, run the file "main.py" in python.

************************** CONTROLS ***********************

ESCAPE or BACKSPACE: Stop modelisation, show tension graph and save it
LEFT and RIGHT ARROW: Move slackliner left or right
DOWN ARROW: Crouch (to generate bounce)
SPACEBAR: Leashfall
F1 - F12: Trigger mainline failure on corresponding segment
ENTER: Save current state of modelisation

**************************** LOAD *************************

The state of a modelisation at a time point can be saved, if you want to use one:
- change the parameter "load" to True in main.py
- specify the path as "save_file" in main.py

************************* PARAMETERS **********************

Else, a modelisation will be created using the file parameters.py
In there, there is already:
-Webbing types: you can create more by specifying 
	-weight per meter in kg
	-stretch
	-force in N at which this stretch is measured
	-drag constant
-Segment styles: you can create more by specifying
	-main webbing type
	-main webbing length
	-backup webbing type
	-backup webbing  length
	-number of points for the modelisation
-A set up is a list of section. There is a few specified, you can write more
-"setup" is the setup used in the modelisation. Modify this line to use a different setup
-"maxt" is used for the color scale of the animation.

The next parameters describe the spot. Modify at will.
-"length"
-"height"

The next parameters describe the slackliner. Modify at will.
-"leg_length"
-"weight"
-"leash_length"
-"power"
-"position"

The next parameters describe the computation parameters. Be carefull when modifying.
-"dt"
-"steps"
-"settle_time"
-"max_steps"