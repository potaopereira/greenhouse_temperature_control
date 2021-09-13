ifndef DEPENDENCIES
DEPENDENCIESALL=
else
DEPENDENCIESALL=--dependencies $(DEPENDENCIES)
endif

ifndef PACKAGE
$(error PACKAGE not defined, make PACKAGE=package-name DEPENDENCIES='dependency1 dependency2')
endif

all: create
	
create:
	ros2 pkg create --build-type ament_cmake $(PACKAGE) $(DEPENDENCIESALL)