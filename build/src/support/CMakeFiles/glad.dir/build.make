# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/frank/15618_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/frank/15618_project/build

# Include any dependencies generated for this target.
include src/support/CMakeFiles/glad.dir/depend.make

# Include the progress variables for this target.
include src/support/CMakeFiles/glad.dir/progress.make

# Include the compile flags for this target's objects.
include src/support/CMakeFiles/glad.dir/flags.make

src/support/CMakeFiles/glad.dir/glad/glad.c.o: src/support/CMakeFiles/glad.dir/flags.make
src/support/CMakeFiles/glad.dir/glad/glad.c.o: ../src/support/glad/glad.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object src/support/CMakeFiles/glad.dir/glad/glad.c.o"
	cd /home/frank/15618_project/build/src/support && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glad.dir/glad/glad.c.o   -c /home/frank/15618_project/src/support/glad/glad.c

src/support/CMakeFiles/glad.dir/glad/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glad.dir/glad/glad.c.i"
	cd /home/frank/15618_project/build/src/support && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/frank/15618_project/src/support/glad/glad.c > CMakeFiles/glad.dir/glad/glad.c.i

src/support/CMakeFiles/glad.dir/glad/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glad.dir/glad/glad.c.s"
	cd /home/frank/15618_project/build/src/support && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/frank/15618_project/src/support/glad/glad.c -o CMakeFiles/glad.dir/glad/glad.c.s

src/support/CMakeFiles/glad.dir/glad/glad.c.o.requires:

.PHONY : src/support/CMakeFiles/glad.dir/glad/glad.c.o.requires

src/support/CMakeFiles/glad.dir/glad/glad.c.o.provides: src/support/CMakeFiles/glad.dir/glad/glad.c.o.requires
	$(MAKE) -f src/support/CMakeFiles/glad.dir/build.make src/support/CMakeFiles/glad.dir/glad/glad.c.o.provides.build
.PHONY : src/support/CMakeFiles/glad.dir/glad/glad.c.o.provides

src/support/CMakeFiles/glad.dir/glad/glad.c.o.provides.build: src/support/CMakeFiles/glad.dir/glad/glad.c.o


# Object files for target glad
glad_OBJECTS = \
"CMakeFiles/glad.dir/glad/glad.c.o"

# External object files for target glad
glad_EXTERNAL_OBJECTS =

lib/libglad.so: src/support/CMakeFiles/glad.dir/glad/glad.c.o
lib/libglad.so: src/support/CMakeFiles/glad.dir/build.make
lib/libglad.so: /usr/lib/x86_64-linux-gnu/libGL.so
lib/libglad.so: /usr/lib/x86_64-linux-gnu/libGLU.so
lib/libglad.so: src/support/CMakeFiles/glad.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library ../../lib/libglad.so"
	cd /home/frank/15618_project/build/src/support && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glad.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/support/CMakeFiles/glad.dir/build: lib/libglad.so

.PHONY : src/support/CMakeFiles/glad.dir/build

src/support/CMakeFiles/glad.dir/requires: src/support/CMakeFiles/glad.dir/glad/glad.c.o.requires

.PHONY : src/support/CMakeFiles/glad.dir/requires

src/support/CMakeFiles/glad.dir/clean:
	cd /home/frank/15618_project/build/src/support && $(CMAKE_COMMAND) -P CMakeFiles/glad.dir/cmake_clean.cmake
.PHONY : src/support/CMakeFiles/glad.dir/clean

src/support/CMakeFiles/glad.dir/depend:
	cd /home/frank/15618_project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/frank/15618_project /home/frank/15618_project/src/support /home/frank/15618_project/build /home/frank/15618_project/build/src/support /home/frank/15618_project/build/src/support/CMakeFiles/glad.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/support/CMakeFiles/glad.dir/depend

