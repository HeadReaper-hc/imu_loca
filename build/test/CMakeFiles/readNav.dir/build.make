# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/huchao/imu_localization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/huchao/imu_localization/build

# Include any dependencies generated for this target.
include test/CMakeFiles/readNav.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/readNav.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/readNav.dir/flags.make

test/CMakeFiles/readNav.dir/readNav.cpp.o: test/CMakeFiles/readNav.dir/flags.make
test/CMakeFiles/readNav.dir/readNav.cpp.o: ../test/readNav.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huchao/imu_localization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/readNav.dir/readNav.cpp.o"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readNav.dir/readNav.cpp.o -c /home/huchao/imu_localization/test/readNav.cpp

test/CMakeFiles/readNav.dir/readNav.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readNav.dir/readNav.cpp.i"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huchao/imu_localization/test/readNav.cpp > CMakeFiles/readNav.dir/readNav.cpp.i

test/CMakeFiles/readNav.dir/readNav.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readNav.dir/readNav.cpp.s"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huchao/imu_localization/test/readNav.cpp -o CMakeFiles/readNav.dir/readNav.cpp.s

test/CMakeFiles/readNav.dir/readNav.cpp.o.requires:

.PHONY : test/CMakeFiles/readNav.dir/readNav.cpp.o.requires

test/CMakeFiles/readNav.dir/readNav.cpp.o.provides: test/CMakeFiles/readNav.dir/readNav.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/readNav.dir/build.make test/CMakeFiles/readNav.dir/readNav.cpp.o.provides.build
.PHONY : test/CMakeFiles/readNav.dir/readNav.cpp.o.provides

test/CMakeFiles/readNav.dir/readNav.cpp.o.provides.build: test/CMakeFiles/readNav.dir/readNav.cpp.o


test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o: test/CMakeFiles/readNav.dir/flags.make
test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o: ../src/imu_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huchao/imu_localization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readNav.dir/__/src/imu_math.cpp.o -c /home/huchao/imu_localization/src/imu_math.cpp

test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readNav.dir/__/src/imu_math.cpp.i"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huchao/imu_localization/src/imu_math.cpp > CMakeFiles/readNav.dir/__/src/imu_math.cpp.i

test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readNav.dir/__/src/imu_math.cpp.s"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huchao/imu_localization/src/imu_math.cpp -o CMakeFiles/readNav.dir/__/src/imu_math.cpp.s

test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.requires:

.PHONY : test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.requires

test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.provides: test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/readNav.dir/build.make test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.provides.build
.PHONY : test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.provides

test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.provides.build: test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o


test/CMakeFiles/readNav.dir/__/src/imu.cpp.o: test/CMakeFiles/readNav.dir/flags.make
test/CMakeFiles/readNav.dir/__/src/imu.cpp.o: ../src/imu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huchao/imu_localization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object test/CMakeFiles/readNav.dir/__/src/imu.cpp.o"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readNav.dir/__/src/imu.cpp.o -c /home/huchao/imu_localization/src/imu.cpp

test/CMakeFiles/readNav.dir/__/src/imu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readNav.dir/__/src/imu.cpp.i"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huchao/imu_localization/src/imu.cpp > CMakeFiles/readNav.dir/__/src/imu.cpp.i

test/CMakeFiles/readNav.dir/__/src/imu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readNav.dir/__/src/imu.cpp.s"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huchao/imu_localization/src/imu.cpp -o CMakeFiles/readNav.dir/__/src/imu.cpp.s

test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.requires:

.PHONY : test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.requires

test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.provides: test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/readNav.dir/build.make test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.provides.build
.PHONY : test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.provides

test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.provides.build: test/CMakeFiles/readNav.dir/__/src/imu.cpp.o


test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o: test/CMakeFiles/readNav.dir/flags.make
test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o: ../IMU_READ/imuDataRead.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/huchao/imu_localization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o -c /home/huchao/imu_localization/IMU_READ/imuDataRead.cpp

test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.i"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/huchao/imu_localization/IMU_READ/imuDataRead.cpp > CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.i

test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.s"
	cd /home/huchao/imu_localization/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/huchao/imu_localization/IMU_READ/imuDataRead.cpp -o CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.s

test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.requires:

.PHONY : test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.requires

test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.provides: test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/readNav.dir/build.make test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.provides.build
.PHONY : test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.provides

test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.provides.build: test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o


# Object files for target readNav
readNav_OBJECTS = \
"CMakeFiles/readNav.dir/readNav.cpp.o" \
"CMakeFiles/readNav.dir/__/src/imu_math.cpp.o" \
"CMakeFiles/readNav.dir/__/src/imu.cpp.o" \
"CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o"

# External object files for target readNav
readNav_EXTERNAL_OBJECTS =

../bin/readNav: test/CMakeFiles/readNav.dir/readNav.cpp.o
../bin/readNav: test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o
../bin/readNav: test/CMakeFiles/readNav.dir/__/src/imu.cpp.o
../bin/readNav: test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o
../bin/readNav: test/CMakeFiles/readNav.dir/build.make
../bin/readNav: test/CMakeFiles/readNav.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/huchao/imu_localization/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ../../bin/readNav"
	cd /home/huchao/imu_localization/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/readNav.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/readNav.dir/build: ../bin/readNav

.PHONY : test/CMakeFiles/readNav.dir/build

test/CMakeFiles/readNav.dir/requires: test/CMakeFiles/readNav.dir/readNav.cpp.o.requires
test/CMakeFiles/readNav.dir/requires: test/CMakeFiles/readNav.dir/__/src/imu_math.cpp.o.requires
test/CMakeFiles/readNav.dir/requires: test/CMakeFiles/readNav.dir/__/src/imu.cpp.o.requires
test/CMakeFiles/readNav.dir/requires: test/CMakeFiles/readNav.dir/__/IMU_READ/imuDataRead.cpp.o.requires

.PHONY : test/CMakeFiles/readNav.dir/requires

test/CMakeFiles/readNav.dir/clean:
	cd /home/huchao/imu_localization/build/test && $(CMAKE_COMMAND) -P CMakeFiles/readNav.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/readNav.dir/clean

test/CMakeFiles/readNav.dir/depend:
	cd /home/huchao/imu_localization/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/huchao/imu_localization /home/huchao/imu_localization/test /home/huchao/imu_localization/build /home/huchao/imu_localization/build/test /home/huchao/imu_localization/build/test/CMakeFiles/readNav.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/readNav.dir/depend
