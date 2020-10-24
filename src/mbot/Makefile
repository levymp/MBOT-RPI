include ../common.mk

CFLAGS  =  $(CFLAGS_STD)  $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
CXXFLAGS = $(CXXFLAGS_STD) $(CFLAGS_COMMON)  $(CFLAGS_LCMTYPES) -O3
LDFLAGS = $(LDFLAGS_STD) $(LDFLAGS_COMMON) $(LDFLAGS_LCMTYPES) $(LDFLAGS_RPLIDAR) 
LIBDEPS = $(call libdeps, $(LDFLAGS))

BIN_DRIVE_SQUARE = $(BIN_PATH)/drive_square
BIN_RPLIDAR_DRIVER = $(BIN_PATH)/rplidar_driver
BIN_MOTION_CONTROLLER = $(BIN_PATH)/motion_controller
BIN_MOTION_CONTROLLER_NEW = $(BIN_PATH)/motion_controller_new
BIN_TIMESYNC = $(BIN_PATH)/timesync
BIN_WIFI_READER = $(BIN_PATH)/wifi_reader
BIN_OLED_EXAMPLE = $(BIN_PATH)/oled_example

BIN_DRIVE_STRAIGHT = $(BIN_PATH)/drive_straight
BIN_DRIVE_WAVE = $(BIN_PATH)/drive_wave
BIN_DRIVE_HOME = $(BIN_PATH)/dive_home
BIN_DRIVE_COMP1 = $(BIN_PATH)/comp_1
BIN_DRIVE_COMP3 = $(BIN_PATH)/comp_3
ALL = $(BIN_DRIVE_WAVE) $(BIN_DRIVE_HOME) $(BIN_DRIVE_COMP1) $(BIN_DRIVE_COMP3) $(BIN_RPLIDAR_DRIVER) $(BIN_SWEEP_DRIVER) $(BIN_WIFI_READER) $(BIN_TIMESYNC) $(BIN_DRIVE_SQUARE) $(BIN_MOTION_CONTROLLER) $(BIN_MC_SMOOTH) $(BIN_OLED_EXAMPLE) $(BIN_DRIVE_STRAIGHT) $(BIN_MOTION_CONTROLLER_NEW)

all: $(ALL)

laptop-only: $(BIN_TIMESYNC) $(BIN_DRIVE_SQUARE) $(BIN_MOTION_CONTROLLER) $(BIN_DRIVE_STRAIGHT)

$(BIN_DRIVE_SQUARE): drive_square.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_DRIVE_STRAIGHT): drive_straight.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_DRIVE_WAVE): drive_wave.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_DRIVE_HOME): dive_home.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_DRIVE_COMP1): comp_1.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_DRIVE_COMP3): comp_3.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MOTION_CONTROLLER): motion_controller.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_MOTION_CONTROLLER_NEW): motion_controller_new.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_RPLIDAR_DRIVER): rplidar_driver.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_TIMESYNC): timesync.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_WIFI_READER): wifi_reader.o $(LIBDEPS)
	@echo "    $@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_OLED_EXAMPLE): oled_example.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
