# Highline Backupfall Simulator

This is a project to moderlize backup falls and general dynamics in highlines.
Created by [@AugustinMoinat](https://github.com/AugustinMoinat) for International Slackline Association purposes.


## Project setup

1. Create a virtual environment:
```bash
python -m venv .venv
```
2. Activate it:
```bash
source .venv/bin/activate
```
3. Install the needed dependencies:
```bash
pip install -r requirements.txt
```


## Running 

To run the modelization:
```bash
python main.py
```


## Controls

|Keys|Description|
|---|---|
|<kbd>ESCAPE</kbd> or <kbd>BACKSPACE</kbd> | Stop modelization, show tension graph and save it|
|<kbd>LEFT</kbd> and <kbd>RIGHT</kbd> ARROW | Move slackliner left or right|
|<kbd>DOWN</kbd> ARROW | Crouch (to generate bounce)|
|<kbd>SPACEBAR</kbd> | Leashfall|
|<kbd>F1 - F12</kbd> | Trigger mainline failure on corresponding segment|
|<kbd>ENTER</kbd> | Save current state of modelization|


## Load

The state of a modelization at a time point can be saved, if you want to use one:
- change the parameter `load` to `True` in [main.py](main.py)
- specify the path as `save_file` in [main.py](main.py)


## Parameters

Else, a modelization will be created using the file parameters.py
In there, there is already:
- **Webbing types**: you can create more by specifying 
	- weight per meter in kg
	- stretch
	- force in _N_ at which this stretch is measured
	- drag constant
- **Segment styles**: you can create more by specifying
	- main webbing type
	- main webbing length
	- backup webbing type
	- backup webbing  length
	- number of points for the modelization
A setup is a list of sections. There are a few specified, you can write more
- `setup` is the setup used in the modelization. Modify this line to use a different setup
- `maxt` is used for the color scale of the animation.

The next parameters describe the spot. Modify at will.
- `length`
- `height`

The next parameters describe the slackliner. Modify at will.
- `leg_length`
- `weight`
- `leash_length`
- `power`
- `position`

The next parameters describe the computation parameters. Be careful when modifying.
- `dt`
- `steps`
- `settle_time`
- `max_steps`
