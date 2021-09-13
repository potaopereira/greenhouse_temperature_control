SHELL:=/bin/bash

# source install/local_setup.bash

all: runlaunch

# if we don't source, ros2 does not find the nodes, messages, services, etc
# sourcing is always necessary
run=\
source /opt/ros/galactic/setup.bash \
&& \
source ./install/local_setup.bash \
&& \
$(1)

run_agnostic=\
source /opt/ros/galactic/setup.bash \
&& \
$(1)

source:
	$(call run, echo sourced)

runlaunch: colcon_build
	# $(call run, ros2 launch greenhouse_simulator sim.launch.py)
	# $(call run, ros2 launch greenhouse_simulator sim_contr_temp.launch.py)
	# $(call run, ros2 launch gui_monitor contr_and_sensors.launch.py)
	$(call run, ros2 launch gui_monitor monitor.launch.py)

runrqt_graph:
	$(call run, rqt_graph)

runrqt:
	$(call run, rqt)

stop:
	$(call run, ros2 lifecycle set /cpa/mynaric_cpa shutdown)
	$(call run, ros2 lifecycle set /fpa/mynaric_fpa shutdown)
	$(call run, ros2 lifecycle set /mynaric_tcc shutdown)

colcon_build:
	@echo If you add dependencies from the "outside", please do: make dependencies
	# $(call run_agnostic, colcon build)
	$(call run_agnostic, colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo)

colcon_build_no_debug:
	@echo If you add dependencies from the "outside", please do: make dependencies
	$(call run_agnostic, colcon build)

# check packages dependencies
dependencies:
	$(call run_agnostic, rosdep update && rosdep install -i --from-path src --rosdistro galactic -y)