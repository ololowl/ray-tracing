.DEFAULT_GOAL := libtinyobjloader.a
# Supress makefile rescanning
Makefile: ;

# Compile static library
libtinyobjloader.a:
	gcc tiny_obj_loader.cc -c -o libtinyobjloader.o
	ar rcs libtinyobjloader.a libtinyobjloader.o

.PHONY: clean
clean:
	rm -f libtinyobjloader.a libtinyobjloader.o