#Spatio-temporal topological exploration

This package provides the nodes necessary to run spatio-temporal exploration over the nodes in the topological map.

###Parameters

* `tag_node`: nodes where is not possible to build a ground truth. The exploration task is set by an exploration routine.
* `tag_ground_node`: nodes where a ground truth will be build. The exploration task is sent to the scheduler.
* `schedule_directory`: directory where the schedules are saved.
* `grids_directory`: directory where the 3D grids are saved.
* `resolution`: voxel size.
* `dimX`: number of cells along the axis X.
* `dimY`: number of cells along the axis Y.
* `dimZ`: number of cells along the axis Z.
* `sweep_type`: *do_sweep* action type (complete, medium, short, shortest). 
* `taskDuration`: exploration task time duration in seconds. 

###Running
`roslaunch topological_exploration topological_exploration.launch`
