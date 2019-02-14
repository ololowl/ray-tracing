# This makefile is bridge between
# a library stuff and stuff that the library exports to world
# and, particularly, exports into our project.
# This makefile manages header and library files we use.
# Every file in these folders updates with owner Makefile.

INCLUDE_DIR = include
LIBRARY_DIR = lib
EXTERNALS_BASEDIR = ../externals

TARGET = easybmp tinyobjloader inih
.DEFAULT_GOAL := $(TARGET)

# Avoid useless remaking of this Makefile.
Makefile : ;

EASYBMP_LOCATION = $(EXTERNALS_BASEDIR)/EasyBMP
EASYBMP_MAKEFILE = Makefile
EASYBMP_HEADERS = include/EasyBMP_BMP.h \
		  include/EasyBMP_DataStructures.h \
		  include/EasyBMP.h \
		  include/EasyBMP_VariousBMPutilities.h
EASYBMP_LIBRARY = lib/libeasybmp.a

TINYOBJ_LOCATION = $(EXTERNALS_BASEDIR)/tinyobjloader
TINYOBJ_MAKEFILE = Makefile
TINYOBJ_HEADERS = tiny_obj_loader.h
TINYOBJ_LIBRARY = libtinyobjloader.a

INIH_LOCATION = $(EXTERNALS_BASEDIR)/inih
INIH_MAKEFILE = Makefile
INIH_HEADERS = cpp/INIReader.h
INIH_LIBRARY = libinih.a

# EasyBMP

.PHONY: easybmp
easybmp: $(EASYBMP_HEADERS) $(EASYBMP_LIBRARY)
	cp $(addprefix $(EASYBMP_LOCATION)/, $(EASYBMP_HEADERS)) $(INCLUDE_DIR)/
	cp $(addprefix $(EASYBMP_LOCATION)/, $(EASYBMP_LIBRARY)) $(LIBRARY_DIR)/

.PHONY: $(EASYBMP_HEADERS)
$(EASYBMP_HEADERS):
	make -C $(EASYBMP_LOCATION) -f $(EASYBMP_MAKEFILE) $@

.PHONY: $(EASYBMP_LIBRARY)
$(EASYBMP_LIBRARY):
	make -C $(EASYBMP_LOCATION) -f $(EASYBMP_MAKEFILE) $@

# TinyObjReader

.PHONY: tinyobjloader
tinyobjloader: $(TINYOBJ_HEADERS) $(TINYOBJ_LIBRARY)
	cp $(addprefix $(TINYOBJ_LOCATION)/, $(TINYOBJ_HEADERS)) $(INCLUDE_DIR)/
	cp $(addprefix $(TINYOBJ_LOCATION)/, $(TINYOBJ_LIBRARY)) $(LIBRARY_DIR)/

.PHONY: $(TINYOBJ_HEADERS)
$(TINYOBJ_HEADERS):
	make -C $(TINYOBJ_LOCATION) -f $(TINYOBJ_MAKEFILE) $@

.PHONY: $(TINYOBJ_LIBRARY)
$(TINYOBJ_LIBRARY):
	make -C $(TINYOBJ_LOCATION) -f $(TINYOBJ_MAKEFILE) $@

# Inih

.PHONY: inih
inih: $(INIH_HEADERS) $(INIH_LIBRARY)
	cp $(addprefix $(INIH_LOCATION)/, $(INIH_HEADERS)) $(INCLUDE_DIR)/
	cp $(addprefix $(INIH_LOCATION)/, $(INIH_LIBRARY)) $(LIBRARY_DIR)/

.PHONY: $(INIH_HEADERS)
$(INIH_HEADERS):
	make -C $(INIH_LOCATION) -f $(INIH_MAKEFILE) $@

.PHONY: $(INIH_LIBRARY)
$(INIH_LIBRARY):
	make -C $(INIH_LOCATION) -f $(INIH_MAKEFILE) $@


.PHONY: clean
clean:
	rm -f $(wildcard $(INCLUDE_DIR)/*) $(wildcard $(LIBRARY_DIR)/*)
	make -C $(EASYBMP_LOCATION) -f $(EASYBMP_MAKEFILE) $@
	make -C $(TINYOBJ_LOCATION) -f $(TINYOBJ_MAKEFILE) $@
	make -C $(INIH_LOCATION) -f $(INIH_MAKEFILE) $@
