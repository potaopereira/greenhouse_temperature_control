WORKSPACE=workspace_greenhouse
IMG=ros_custom_image
CNT=ros_custom_cnt_$(WORKSPACE)

# without privileged rqt, rqt_graph does not work
create = \
docker \
create \
--name $(CNT) \
--cap-add=SYS_PTRACE \
--network=host \
-ti \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v "$(PWD)"/$(WORKSPACE):/home/$(shell id -nu)/workspace \
--privileged \
--workdir /home/$(shell id -nu)/workspace \
$(IMG) \
$(1)

exec = \
docker \
exec \
$(CNT) \
$(1)

all: build runlaunch

start:
	xhost +local:docker
	# @docker inspect --format="{{.Id}}" $(CNT) || (echo first you must create container: make create_container && exit 1)
	docker inspect --format="{{.Id}}" $(CNT) || make create_container
	docker start $(CNT)

inspect: start
	docker \
	exec \
	-it \
	$(CNT) \
	/usr/bin/zsh

build: start
	$(call exec, make dependencies colcon_build)

just_build: start
	$(call exec, make colcon_build)

runlaunch: start
	$(call exec, make runlaunch)

runrqt: start
	$(call exec, make runrqt)

stop: start
	$(call exec, make stop)

create_container: $(IMG)
	xhost +local:docker
	$(call create, /usr/bin/zsh)

stop_container:
	docker container stop $(CNT)

rm_container:
	docker container stop $(CNT) \
	&& \
	docker container rm $(CNT) \

rm_image:
	docker image rm $(IMG)

run: $(IMG)
	xhost +local:docker
	$(call exec, /usr/bin/zsh)

$(IMG):
	docker image inspect $@ > /dev/null \
	|| \
	make build_img

build_img: dockerfile.df
	docker build \
	--tag $(IMG) \
	--build-arg user=$(shell id -nu) \
	--file $^ \
	.