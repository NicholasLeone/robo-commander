SHELL= /bin/sh

#REFER: https://www.cs.umd.edu/class/fall2002/cmsc214/Tutorial/makefile.html
#REFER: https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html
#REFER: http://make.mad-scientist.net/papers/multi-architecture-builds/

#Author: Hunter Young
VERSION = 1.0.0

#Executables name
EXEC=TestUDP
#Object file s
OBJ_FOLDER=obj
PROJ_DIR=../..
#default - DO NOT CHANGE
default: obj udp
all: udp

#Compiler to use for CPP
# CC= @arm-linux-gnueabihf-g++
CC=g++
#Compiler to use for C
# CC1= @arm-linux-gnueabihf-gcc
CC1=gcc

LIBS+= -lpigpiod_if2 -Wall -pthread -lboost_system -lboost_thread
# INCLUDEPATH+= -I$(PROJ_DIR)/include -I$(PROJ_DIR) -I../../include

CFLAGS+= -c -w -pthread -lrt -O2 -std=c++11
CFLAGS+= $(CPPFLAGS)
BUILD_FLAGS= -o $(EXEC) -O2 -pthread -lrt -Wall $(objects)

#Required object files - Add new objects here
# objects= $(addprefix $(OBJ_FOLDER)/, serial_dev.o)
objects= $(OBJ_FOLDER)/udp.o $(OBJ_FOLDER)/test_udp.o



#Make recipies - Add object files here with necessary dependiences
objectfiles: header_print $(objects)

obj:
	@mkdir -p $@

link:
	@echo Building executable ...
	$(CC) $(BUILD_FLAGS) $(LIBS)
	@echo Build complete ...

udp: header_print $(objects)
	@echo Building executable ...
	$(CC) $(BUILD_FLAGS) $(LIBS)
	@echo Build complete ...

$(OBJ_FOLDER)/udp.o: udp.cpp udp.h
	@echo Building udp.cpp ...
	$(CC) $(CFLAGS) udp.cpp -o $(OBJ_FOLDER)/udp.o

$(OBJ_FOLDER)/test_udp.o: test_udp.cpp
	@echo Building test_udp.cpp ...
	$(CC) $(CFLAGS) test_udp.cpp -o $(OBJ_FOLDER)/test_udp.o

header_print:
	@echo "************************************************"
	@echo "* DDDD      A     SSSS  L          A     BBBB  *"
	@echo "* D   D    A A   S      L         A A    B   B *"
	@echo "* D    D  AAAAA   SSSS  L        AAAAA   BBBB  *"
	@echo "* D   D  A     A      S L       A     A  B   B *"
	@echo "* DDDD  A       A SSSS  LLLLLL A       A BBBB  *"
	@echo "***************Drivers v$(VERSION)*****************"

showincludepath:
	$(CC) -xc++ -E -v -

#Prints all files in need of updating
update_log: $(all_dep)
	lpr -p $?
	@touch update_log

#File clean recipie
.PHONY : clean
clean :
	@rm -rf $(OBJ_FOLDER)
	@mkdir -p $(OBJ_FOLDER)

#for debugging variables (e.g. make print-OBJ_FOLDER)
print-%: ; @echo $*=$($*)
