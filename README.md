# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

## Project Description
The project "unscented Kalman filter" is based on the same structure as the extended Kalman filter.
It uses a main file that calls a function called ProcessMeasurement. Anything important happens in this function. The function is part of the class ukf.

## Goal
- The overall processing chain (prediction, laser update or radar update depending on measurement type) must be correct.
- The student is not allowed to use values from the future to reason about the current state.
- It must be possible to run the project in three different modes: considering laser only, with considering radar only, or with using both sensors.
- For every mode, the overall RMSE (2d position only) may not be more than 10% increased to what the original solution is able to reach (this number depends on the individual measurement sequence)
- The RMSE of laser AND radar must be lower than radar only or laser only
- The NIS of radar measurements must be between 0.35 and 7.81 in at least 80% of all radar update steps.

## Visualization

![Visualization](./Simulation.png)

## Achieving RMSE and NIS values

The following parameters were tuned using a grid-search method:

	- process noise
		- std_a		
		- std_yawdd
	- laser measurement noise
		- std_laspx
		- std_laspy
		- both were fixed to a single value std_laspxy
	- radar measurement noise
		- std_radr	
		- std_radphi
		- std_radrd

The grid search method iterates through ranges of values and evaluates performance for each set of values.

It gathers the following metrics at each step:

	- rmse_px
	- rmse_py
	- rmse_vx
	- rmse_vy
	- nis_percentage (% of measurements with 0.35 > NIS < 7.81)
	- nis_lidar (breakdown)
	- nis_radar (breakdown)

[Here is sample output from grid-search output.](data/gridsearch_fragment.txt)

### Analysis

- Select fixed values of laser and radar noise
- Varying process noise (std-a and std-yawdd) in grid search
- Tabulate grid search results and sort them (in Excel) by RMSE to pick best process noise values.
- Plot process noise (std-a and sts-yawdd) against NIS values in a 3D plot.

Pandas dataframes and pyplot are used in plot.py to 3D-plot:
X-axis: std-a, Y-axis: std_yawdd and Z-axis: NIS %.
![3D-plot](./NIS-plot.png)

## Results
After studying the data sets and grid search results, I obtained the following results:

### Lowest RMSE

- Process noise
	- std_a = 1.05
	- std_yawdd = 0.5

- Measurement noise
	- std_lasp_x = std_lasp_y = 0.3
	- std_radr = 0.11
	- std_radphi = 0.001
	- std_radrd = 0.11

Data | PX | PY | VX | VY
--- | --- | --- | --- | ---
"sample 1" | 0.036368 | 0.032266 | 0.460496 | 0.459381
"sample 2" | 0.181998 | 0.183915 | 0.388531 | 0.489551

** NIS:** Radar NIS too low on average: between 0.35 and 7.81 in only 26% of cases

### Using noise values provided in EKF project

- Process noise
	- std_a = 1.041
	- std_yawdd = 0.25

- Measurement noise
	- std_lasp_x = std_lasp_y = 0.0225
	- std_radr = 0.09
	- std_radphi = 0.009
	- std_radrd = 0.09

Data | PX | PY | VX | VY
--- | --- | --- | --- | ---
"sample 1" | 0.0439351 | 0.0541247 | 0.533012 | 0.539625
"sample 2" | 0.199834 | 0.197363 | 0.376363 | 0.463544

** NIS:** Radar NIS between 0.35 and 7.81 in only 42% of measurements

### Good NIS

- Process noise
	- std_a = 0.12
	- std_yawdd = 0.54

- Measurement noise
	- std_lasp_x = std_lasp_y = 0.0825
	- std_radr = 0.25
	- std_radphi = 0.0125
	- std_radrd = 0.25

Data | PX | PY | VX | VY
--- | --- | --- | --- | ---
"sample 1" | 0.0533476 | 0.0627673 | 0.55714 | 0.551546
"sample 2" | 0.198674 | 0.191792 | 0.474725 | 0.5422

** NIS:** Radar NIS between 0.35 and 7.81 in 81% of measurement


## Command line to run grid search
```
./UnscentedKF gridsearch ../data/good_values.txt ../data/sample-laser-radar-measurement-data-2.txt 0.2 0.55 ../data/sample-laser-radar-measurement-data-1.txt 0.09 0.65
```
## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`
