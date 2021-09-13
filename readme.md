# How to run application

```make```

1. It will create the image
2. It will create the container
3. It will build the project using colcon_build
4. It will call the launch file that starts all the necessary nodes

When you are finished

```make stop_container```

# To generate documentation

```make --directory=doxygen```