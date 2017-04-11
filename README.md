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


## Achieving RMSE values in rubrik
The following parameters were tuned using a grid-search method:
	std_a		
	std_yawdd	
	std_laspx
	std_laspy	
	std_radr	
	std_radphi
	std_radrd

The grid search method iterates through ranges of values and evaluates performance for each set of values.
It gathers the following metrics at each step:
	rmse_px
	rmse_py
	rmse_vx
	rmse_vy
	nis_lidar
	nis_radar
	nis_total (average of lidar + radar)

[Here is a small set of samples from grid-search output.](data/gridsearch_fragment.txt)

From the gathered data, parameters that meet the project goal could be easily selected.


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
