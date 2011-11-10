all: installed
	rm -rf build

TARBALL = build/eml-r36.tar.bz2
TARBALL_URL = http://pr.willowgarage.com/downloads/eml-r36.tar.bz2
SOURCE_DIR = build/eml-svn
UNPACK_CMD = tar xjf
TARBALL_PATCH = eml-r36.patch
include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked Makefile
	cd $(SOURCE_DIR) && \
	mkdir -p build && \
	cd build && \
	cmake -D CMAKE_INSTALL_PREFIX=`rospack find eml`/eml .. && \
	make install && \
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-make -C $(SOURCE_DIR)/build clean
	rm -rf eml installed

wipe: clean
	rm -rf build



