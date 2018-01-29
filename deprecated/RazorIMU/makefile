SHELL= /bin/sh

#REFER: https://www.cs.umd.edu/class/fall2002/cmsc214/Tutorial/makefile.html
#REFER: https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html
#REFER: http://make.mad-scientist.net/papers/multi-architecture-builds/

#Author: Hunter Young
VERSION = 1.0.0

#Executables name
EXEC=TestRazor

OBJ_FOLDER=obj
HOME_DIR=../..
UTIL_DIR=$(HOME_DIR)/include/Utilities
SER_DIR=$(HOME_DIR)/Communications/SerialDev
#default - DO NOT CHANGE
default: obj mpu9250
all: mpu9250

#Compiler to use for CPP
# CC= @arm-linux-gnueabihf-g++
CC=g++
#Compiler to use for C
# CC1= @arm-linux-gnueabihf-gcc
CC1=gcc

LIBS+= -lpigpiod_if2 -Wall -pthread -lboost_system -lboost_thread
# INCLUDEPATH+= -I$(PROJ_DIR)/include -I$(PROJ_DIR) -I../../include

CFLAGS+= -c -w -pthread -lrt -O2
CFLAGS+= $(CPPFLAGS)
BUILD_FLAGS= -o $(EXEC) -O2 -pthread -lrt -Wall $(objects)

#Required object files - Add new objects here
objects= $(addprefix $(OBJ_FOLDER)/, mpu9250.o test_imu.o serial_dev.o utils.o)
# objects= $(OBJ_FOLDER)/mpu9250.o $(OBJ_FOLDER)/test_imu.o

#Make recipies - Add object files here with necessary dependiences
objectfiles: header_print $(objects)

obj:
	@mkdir -p $@

link:
	@echo Building executable ...
	$(CC) $(BUILD_FLAGS) $(LIBS)
	@echo Build complete ...

mpu9250: header_print $(objects)
	@echo Building executable ...
	$(CC) $(BUILD_FLAGS) $(LIBS)
	@echo Build complete ...

$(OBJ_FOLDER)/serial_dev.o: $(addprefix $(SER_DIR)/, serial_dev.cpp serial_dev.h)
	@echo Building serial_dev.cpp ...
	$(CC) $(CFLAGS) $(SER_DIR)/serial_dev.cpp -o $(OBJ_FOLDER)/serial_dev.o

$(OBJ_FOLDER)/utils.o: $(addprefix $(UTIL_DIR)/, utils.cpp utils.h)
	@echo Building utils.cpp ...
	$(CC) $(CFLAGS) $(UTIL_DIR)/utils.cpp -o $(OBJ_FOLDER)/utils.o

$(OBJ_FOLDER)/mpu9250.o: mpu9250.cpp mpu9250.h
	@echo Building mpu9250.cpp ...
	$(CC) $(CFLAGS) mpu9250.cpp -o $(OBJ_FOLDER)/mpu9250.o

$(OBJ_FOLDER)/test_imu.o: test_read.cpp
	@echo Building test_imu.cpp ...
	$(CC) $(CFLAGS) test_read.cpp -o $(OBJ_FOLDER)/test_imu.o

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
