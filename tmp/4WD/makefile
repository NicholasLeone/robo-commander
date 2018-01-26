SHELL= /bin/sh

#REFER: https://www.cs.umd.edu/class/fall2002/cmsc214/Tutorial/makefile.html
#REFER: https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html
#REFER: http://make.mad-scientist.net/papers/multi-architecture-builds/

#Author: Hunter Young
VERSION = 1.0.0

#Executables name
EXEC=Test4WD
#Object file s
OBJ_FOLDER=obj
PROJ_DIR=../..
MOTOR_DIR=$(PROJ_DIR)/Actuators/DC_Motor
ROBOCLAW_DIR=$(PROJ_DIR)/Actuators/RoboClaw
UTIL_DIR=$(PROJ_DIR)/include/Utilities
ENC_DIR=../../Sensors/Encoder
UDP_DIR=../../Communications/UDP
PID_DIR=../PID

#default - DO NOT CHANGE
default: obj 4wd
all: 4wd

#Compiler to use for CPP
# CC= @arm-linux-gnueabihf-g++
CC=g++
#Compiler to use for C
# CC1= @arm-linux-gnueabihf-gcc
CC1=gcc

LIBS+= -lpigpiod_if2 -Wall -pthread -lboost_system -lboost_thread -larmadillo
# INCLUDEPATH+= -I$(PROJ_DIR)/include -I$(PROJ_DIR) -I../../include

CFLAGS+= -c -w -pthread -lrt -O2 -std=c++11
CFLAGS+= $(CPPFLAGS)
BUILD_FLAGS= -o $(EXEC) -O2 -pthread -lrt -Wall $(objects)

#Required object files - Add new objects here
# objects= $(addprefix $(OBJ_FOLDER)/, serial_dev.o)
objects=$(OBJ_FOLDER)/dc_motor.o $(OBJ_FOLDER)/encoder.o $(OBJ_FOLDER)/utils.o \
$(OBJ_FOLDER)/udp.o $(OBJ_FOLDER)/pid.o $(OBJ_FOLDER)/roboclaw.o \
$(OBJ_FOLDER)/four_wd.o $(OBJ_FOLDER)/test_4wd.o

#Make recipies - Add object files here with necessary dependiences
objectfiles: header_print $(objects)

obj:
	@mkdir -p $@

link:
	@echo Building executable ...
	$(CC) $(BUILD_FLAGS) $(LIBS)
	@echo Build complete ...

4wd: header_print $(objects)
	@echo Building executable ...
	$(CC) $(BUILD_FLAGS) $(LIBS)
	@echo Build complete ...

$(OBJ_FOLDER)/utils.o: $(UTIL_DIR)/utils.cpp $(UTIL_DIR)/utils.h
	@echo Building utils.cpp ...
	$(CC) $(CFLAGS) $(UTIL_DIR)/utils.cpp -o $(OBJ_FOLDER)/utils.o

$(OBJ_FOLDER)/roboclaw.o: $(ROBOCLAW_DIR)/roboclaw.cpp $(ROBOCLAW_DIR)/roboclaw.h
	@echo Building roboclaw.cpp ...
	$(CC) $(CFLAGS) $(ROBOCLAW_DIR)/roboclaw.cpp -o $(OBJ_FOLDER)/roboclaw.o

$(OBJ_FOLDER)/udp.o: $(UDP_DIR)/udp.cpp $(UDP_DIR)/udp.h
	@echo Building udp.cpp ...
	$(CC) $(CFLAGS) $(UDP_DIR)/udp.cpp -o $(OBJ_FOLDER)/udp.o

$(OBJ_FOLDER)/pid.o: $(PID_DIR)/pid.cpp $(PID_DIR)/pid.h
	@echo Building pid.cpp ...
	$(CC) $(CFLAGS) $(PID_DIR)/pid.cpp -o $(OBJ_FOLDER)/pid.o

$(OBJ_FOLDER)/encoder.o: $(ENC_DIR)/encoder.cpp $(ENC_DIR)/encoder.h
	@echo Building encoder.cpp ...
	$(CC) $(CFLAGS) $(ENC_DIR)/encoder.cpp -o $(OBJ_FOLDER)/encoder.o

$(OBJ_FOLDER)/dc_motor.o: $(MOTOR_DIR)/dc_motor.cpp $(MOTOR_DIR)/dc_motor.h
	@echo Building dc_motor.cpp ...
	$(CC) $(CFLAGS) $(MOTOR_DIR)/dc_motor.cpp -o $(OBJ_FOLDER)/dc_motor.o

$(OBJ_FOLDER)/four_wd.o: four_wd.cpp four_wd.h
	@echo Building four_wd.cpp ...
	$(CC) $(CFLAGS) four_wd.cpp -o $(OBJ_FOLDER)/four_wd.o

$(OBJ_FOLDER)/test_4wd.o: test_4wd.cpp
	@echo Building test_4wd.cpp ...
	$(CC) $(CFLAGS) test_4wd.cpp -o $(OBJ_FOLDER)/test_4wd.o

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
