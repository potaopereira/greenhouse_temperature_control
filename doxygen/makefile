IMAGE=doxygen_img
BROWSER=google-chrome
# this is where we find the Doxyfile
PROJECT=../workspace_greenhouse
# this is where we will put the output from doxygen
DOXYDIR=doxygen

all: doxygen_html

################################################################################
#                                  Doxygen                                     #
################################################################################

doxygen_html: doxygen_run
	# need to copy images to html directory
	cp -r $(PROJECT)/images $(DOXYDIR)/html
	$(BROWSER) $(DOXYDIR)/html/index.html

# doxygen_latex: doxygen_run
# 	cp -r $(PROJECT)/images $(DOXYDIR)/latex
# 	cd $(DOXYDIR)/latex && pdflatex refman.tex
# 	$(PDF_VIEWER) $(DOXYDIR)/latex/refman.pdf

doxygen_run: $(PROJECT)/Doxyfile doxygen
	docker \
	run \
	--rm \
	-it \
	-v "$(PWD)"/$(PROJECT):/home/$(shell id -nu)/project \
	-v "$(PWD)"/doxygen:/home/$(shell id -nu)/doxygen \
	-w /home/$(shell id -nu)/project \
	$(IMAGE) \
	/bin/bash -c " \
	doxygen Doxyfile \
	"

inspect: $(PROJECT)/Doxyfile doxygen
	docker \
	run \
	--rm \
	-it \
	-v "$(PWD)"/$(PROJECT):/home/$(shell id -nu)/project \
	-v "$(PWD)"/doxygen:/home/$(shell id -nu)/doxygen \
	-w /home/$(shell id -nu)/project \
	$(IMAGE) \
	/bin/bash

doxygen:
	mkdir $@

$(PROJECT)/project/Doxyfile: .build
	# docker \
	# run \
	# --rm \
	# -it \
	# -v "$(PWD)"/$(PROJECT):/home/$(shell id -nu)/project \
	# -w /home/$(shell id -nu)/project \
	# $(IMAGE) \
	# /bin/bash -c " \
	# doxygen -g Doxyfile OUTPUT_DIRECTORY=doxygen \
	# "

.build: dockerfile.df
	docker build \
	--tag $(IMAGE) \
	--build-arg user=$(shell id -nu) \
	--file $^ \
	.
	touch $@

clean:
	rm -r $(DOXYDIR)

################################################################################
# Phony targets                                                                #
################################################################################

.PHONY: start stop