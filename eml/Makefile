all: installed

ARCH = $(shell uname -m)
ifeq ($(ARCH), i686)
TARBALL_URL = http://pr.willowgarage.com/downloads/eml-prebuilt-i686.tgz
TARBALL = build/eml-prebuilt-i686.tgz
else ifeq ($(ARCH), i386)
TARBALL_URL = http://pr.willowgarage.com/downloads/eml-prebuilt-i686.tgz
TARBALL = build/eml-prebuilt-i686.tgz
else ifeq ($(ARCH), x86_64)
TARBALL_URL = http://pr.willowgarage.com/downloads/eml-prebuilt-x86_64.tgz
TARBALL = build/eml-prebuilt-x86_64.tgz
else
	@echo "Unsupported architecture $(ARCH)"
	@exit 2
endif
SOURCE_DIR = build/eml

include $(shell rospack find mk)/download_unpack_build.mk

installed: wiped $(SOURCE_DIR)/unpacked Makefile
	mkdir -p $(CURDIR)/eml
	cp -R $(SOURCE_DIR)/lib $(SOURCE_DIR)/include $(CURDIR)/eml
	touch installed

wiped: Makefile 
	make wipe 
	touch wiped 

clean:
	rm -rf eml installed

wipe: clean
	rm -rf build



