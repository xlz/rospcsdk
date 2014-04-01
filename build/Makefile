.PHONY: clean cleanall build install all pcsdk

all: install_isolated

cache:
	mkdir -p cache

cache/mingw/.stamp: | cache
	make -f Makefile.mxe compile MINGW_INSTALL_PREFIX=$(CURDIR)/mingw JOBS=$(shell grep -c ^processor /proc/cpuinfo)
	mv mingw cache
	touch $@

cache/python/.stamp: | cache
	virtualenv python
	mv python cache
	touch $@

cache/src/.stamp: | cache python
	. python/bin/activate; wstool init -j8 src groovy-minimal.rosinstall
	mv src cache
	touch $@

python: | cache/python/.stamp
	cp -af cache/python .
	. python/bin/activate; pip install wstool rosdep empy

mingw: | cache/mingw/.stamp
	cp -af cache/mingw .

.bashrc: | mingw
	HOME=$(CURDIR) make -f Makefile.mxe install MINGW_INSTALL_PREFIX=$(CURDIR)/mingw


src: | cache/src/.stamp
	cp -af cache/src .
	rm -f src/.stamp
	patch -d src -p1 <groovy-minimal.patch
	ln -snf ../.. src/pcsdk
	touch src/.stamp

clean:
	rm -rf src mingw *_isolated python .bashrc

cleanall: clean
	rm -rf cache

install_isolated: src | mingw .bashrc python
	. ./.bashrc; . python/bin/activate; ./src/catkin/bin/catkin_make_isolated --install --cmake-args -DCMAKE_TOOLCHAIN_FILE=$$MINGW_CMAKE_TOOLCHAIN_FILE -C$$MINGW_CMAKE_ROS_CONFIG_FILE

install_isolated/lib/pcsdk: ../src/* ../srv/* ../package.xml ../CMakeLists.txt | install_isolated
	. ./.bashrc; . python/bin/activate; ./src/catkin/bin/catkin_make_isolated --pkg pcsdk --install --cmake-args -DCMAKE_TOOLCHAIN_FILE=$$MINGW_CMAKE_TOOLCHAIN_FILE -C$$MINGW_CMAKE_ROS_CONFIG_FILE

pcsdk: install_isolated/lib/pcsdk
