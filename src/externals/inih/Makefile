# Simple makefile to build inih as a static library using g++

SRC = ini.c cpp/INIReader.cpp
OBJ = ini.o INIReader.o
OUT = libinih.a

.DEFAULT_GOAL := $(OUT)
# Supress makefile rescanning
Makefile: ;

ini.o:
	gcc ini.c -c -o ini.o

INIReader.o:
	g++ cpp/INIReader.cpp -c -o INIReader.o

$(OUT): $(OBJ)
	ar rcs $(OUT) $(OBJ)

.PHONY: clean
clean:
	rm -f $(OBJ) $(OUT)
