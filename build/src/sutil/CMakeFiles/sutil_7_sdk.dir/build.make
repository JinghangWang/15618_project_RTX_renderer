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
include src/sutil/CMakeFiles/sutil_7_sdk.dir/depend.make

# Include the progress variables for this target.
include src/sutil/CMakeFiles/sutil_7_sdk.dir/progress.make

# Include the compile flags for this target's objects.
include src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make

lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/internal/optix_7_device_impl.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/internal/optix_7_device_impl_exception.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/internal/optix_7_device_impl_transformations.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/optix.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/optix_7_device.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/optix_7_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../include/optix_device.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/BufferView.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/GeometryData.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/Light.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/LocalGeometry.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/MaterialData.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/random.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/util.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/whitted.cu
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/whitted.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/sutil/Matrix.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/sutil/Preprocessor.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/sutil/sutilapi.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/sutil/vec_math.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/alloca.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/assert.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/concept_check.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/cpp_type_traits.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/cxxabi_init_exception.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/exception.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/exception_defines.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/exception_ptr.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/hash_bytes.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/move.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/nested_exception.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/bits/std_abs.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/cmath
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/cstdio
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/cstdlib
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/exception
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/ext/type_traits.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/initializer_list
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/math.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/new
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/stdlib.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/type_traits
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/c++/7/typeinfo
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/endian.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/features.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/limits.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/linux/limits.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/math.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/stdc-predef.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/stdint.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/stdio.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/stdlib.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/string.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/strings.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/time.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/_G_config.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/byteswap-16.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/byteswap.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/endian.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/floatn-common.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/floatn.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/flt-eval-method.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/fp-fast.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/fp-logb.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/iscanonical.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/libc-header-start.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/libio.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/libm-simd-decl-stubs.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/local_lim.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/long-double.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/math-vector.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/mathcalls-helper-functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/mathcalls.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/posix1_lim.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/posix2_lim.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/pthreadtypes-arch.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/pthreadtypes.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/select.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/stdint-intn.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/stdint-uintn.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/stdio_lim.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/stdlib-float.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/sys_errlist.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/sysmacros.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/thread-shared-types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/time.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/timex.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/FILE.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/__FILE.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/__locale_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/__mbstate_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/__sigset_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/clock_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/clockid_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/locale_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/sigset_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/struct_itimerspec.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/struct_timespec.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/struct_timeval.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/struct_tm.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/time_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/types/timer_t.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/typesizes.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/uintn-identity.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/uio_lim.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/waitflags.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/waitstatus.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/wchar.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/wordsize.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/bits/xopen_lim.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/c++/7/bits/c++config.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/c++/7/bits/cpu_defines.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/c++/7/bits/os_defines.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/gnu/stubs-64.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/gnu/stubs.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/sys/cdefs.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/sys/select.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/sys/sysmacros.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/include/x86_64-linux-gnu/sys/types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/lib/gcc/x86_64-linux-gnu/7/include-fixed/limits.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/lib/gcc/x86_64-linux-gnu/7/include-fixed/syslimits.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/lib/gcc/x86_64-linux-gnu/7/include/stdarg.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/lib/gcc/x86_64-linux-gnu/7/include/stddef.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/lib/gcc/x86_64-linux-gnu/7/include/stdint.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/builtin_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/channel_descriptor.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/common_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/device_double_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/device_double_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/device_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/device_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/host_config.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/host_defines.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/math_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/math_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/sm_70_rt.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/crt/sm_70_rt.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/cuda_device_runtime_api.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/cuda_runtime.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/cuda_runtime_api.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/cuda_surface_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/cuda_texture_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/device_atomic_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/device_atomic_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/device_launch_parameters.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/device_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/driver_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/driver_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/library_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_20_atomic_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_20_atomic_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_20_intrinsics.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_20_intrinsics.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_30_intrinsics.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_30_intrinsics.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_32_atomic_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_32_atomic_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_32_intrinsics.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_32_intrinsics.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_35_atomic_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_35_intrinsics.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_60_atomic_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_60_atomic_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_61_intrinsics.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/sm_61_intrinsics.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/surface_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/surface_indirect_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/surface_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/texture_fetch_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/texture_indirect_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/texture_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/vector_functions.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/vector_functions.hpp
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: /usr/local/cuda/include/vector_types.h
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: src/sutil/CMakeFiles/cuda_compile_ptx.dir/__/cuda/cuda_compile_ptx_generated_whitted.cu.ptx.cmake
lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx: ../src/cuda/whitted.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building NVCC ptx file lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx"
	cd /home/frank/15618_project/build/src/sutil/CMakeFiles/cuda_compile_ptx.dir/__/cuda && /usr/bin/cmake -E make_directory /home/frank/15618_project/build/lib/ptx
	cd /home/frank/15618_project/build/src/sutil/CMakeFiles/cuda_compile_ptx.dir/__/cuda && /usr/bin/cmake -D verbose:BOOL=$(VERBOSE) -D check_dependencies:BOOL=OFF -D build_configuration:STRING= -D generated_file:STRING=/home/frank/15618_project/build/lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx -D generated_cubin_file:STRING=/home/frank/15618_project/build/lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx.cubin.txt -D generated_fatbin_file:STRING=/home/frank/15618_project/build/lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx.fatbin.txt -P /home/frank/15618_project/build/src/sutil/CMakeFiles/cuda_compile_ptx.dir/__/cuda/cuda_compile_ptx_generated_whitted.cu.ptx.cmake

src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o: src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make
src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o: ../src/sutil/Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o -c /home/frank/15618_project/src/sutil/Camera.cpp

src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sutil_7_sdk.dir/Camera.cpp.i"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frank/15618_project/src/sutil/Camera.cpp > CMakeFiles/sutil_7_sdk.dir/Camera.cpp.i

src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sutil_7_sdk.dir/Camera.cpp.s"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frank/15618_project/src/sutil/Camera.cpp -o CMakeFiles/sutil_7_sdk.dir/Camera.cpp.s

src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.requires:

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.provides: src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.requires
	$(MAKE) -f src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.provides.build
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.provides

src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.provides.build: src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o


src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o: src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make
src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o: ../src/sutil/GLDisplay.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o -c /home/frank/15618_project/src/sutil/GLDisplay.cpp

src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.i"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frank/15618_project/src/sutil/GLDisplay.cpp > CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.i

src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.s"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frank/15618_project/src/sutil/GLDisplay.cpp -o CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.s

src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.requires:

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.provides: src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.requires
	$(MAKE) -f src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.provides.build
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.provides

src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.provides.build: src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o


src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o: src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make
src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o: ../src/sutil/PPMLoader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o -c /home/frank/15618_project/src/sutil/PPMLoader.cpp

src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.i"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frank/15618_project/src/sutil/PPMLoader.cpp > CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.i

src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.s"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frank/15618_project/src/sutil/PPMLoader.cpp -o CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.s

src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.requires:

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.provides: src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.requires
	$(MAKE) -f src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.provides.build
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.provides

src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.provides.build: src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o


src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o: src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make
src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o: ../src/sutil/Scene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o -c /home/frank/15618_project/src/sutil/Scene.cpp

src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sutil_7_sdk.dir/Scene.cpp.i"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frank/15618_project/src/sutil/Scene.cpp > CMakeFiles/sutil_7_sdk.dir/Scene.cpp.i

src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sutil_7_sdk.dir/Scene.cpp.s"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frank/15618_project/src/sutil/Scene.cpp -o CMakeFiles/sutil_7_sdk.dir/Scene.cpp.s

src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.requires:

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.provides: src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.requires
	$(MAKE) -f src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.provides.build
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.provides

src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.provides.build: src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o


src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o: src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make
src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o: ../src/sutil/sutil.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o -c /home/frank/15618_project/src/sutil/sutil.cpp

src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sutil_7_sdk.dir/sutil.cpp.i"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frank/15618_project/src/sutil/sutil.cpp > CMakeFiles/sutil_7_sdk.dir/sutil.cpp.i

src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sutil_7_sdk.dir/sutil.cpp.s"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frank/15618_project/src/sutil/sutil.cpp -o CMakeFiles/sutil_7_sdk.dir/sutil.cpp.s

src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.requires:

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.provides: src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.requires
	$(MAKE) -f src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.provides.build
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.provides

src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.provides.build: src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o


src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o: src/sutil/CMakeFiles/sutil_7_sdk.dir/flags.make
src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o: ../src/sutil/Trackball.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o -c /home/frank/15618_project/src/sutil/Trackball.cpp

src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.i"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/frank/15618_project/src/sutil/Trackball.cpp > CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.i

src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.s"
	cd /home/frank/15618_project/build/src/sutil && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/frank/15618_project/src/sutil/Trackball.cpp -o CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.s

src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.requires:

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.provides: src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.requires
	$(MAKE) -f src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.provides.build
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.provides

src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.provides.build: src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o


# Object files for target sutil_7_sdk
sutil_7_sdk_OBJECTS = \
"CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o" \
"CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o" \
"CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o" \
"CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o" \
"CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o" \
"CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o"

# External object files for target sutil_7_sdk
sutil_7_sdk_EXTERNAL_OBJECTS =

lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/build.make
lib/libsutil_7_sdk.a: src/sutil/CMakeFiles/sutil_7_sdk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/frank/15618_project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library ../../lib/libsutil_7_sdk.a"
	cd /home/frank/15618_project/build/src/sutil && $(CMAKE_COMMAND) -P CMakeFiles/sutil_7_sdk.dir/cmake_clean_target.cmake
	cd /home/frank/15618_project/build/src/sutil && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sutil_7_sdk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/sutil/CMakeFiles/sutil_7_sdk.dir/build: lib/libsutil_7_sdk.a

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/build

src/sutil/CMakeFiles/sutil_7_sdk.dir/requires: src/sutil/CMakeFiles/sutil_7_sdk.dir/Camera.cpp.o.requires
src/sutil/CMakeFiles/sutil_7_sdk.dir/requires: src/sutil/CMakeFiles/sutil_7_sdk.dir/GLDisplay.cpp.o.requires
src/sutil/CMakeFiles/sutil_7_sdk.dir/requires: src/sutil/CMakeFiles/sutil_7_sdk.dir/PPMLoader.cpp.o.requires
src/sutil/CMakeFiles/sutil_7_sdk.dir/requires: src/sutil/CMakeFiles/sutil_7_sdk.dir/Scene.cpp.o.requires
src/sutil/CMakeFiles/sutil_7_sdk.dir/requires: src/sutil/CMakeFiles/sutil_7_sdk.dir/sutil.cpp.o.requires
src/sutil/CMakeFiles/sutil_7_sdk.dir/requires: src/sutil/CMakeFiles/sutil_7_sdk.dir/Trackball.cpp.o.requires

.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/requires

src/sutil/CMakeFiles/sutil_7_sdk.dir/clean:
	cd /home/frank/15618_project/build/src/sutil && $(CMAKE_COMMAND) -P CMakeFiles/sutil_7_sdk.dir/cmake_clean.cmake
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/clean

src/sutil/CMakeFiles/sutil_7_sdk.dir/depend: lib/ptx/cuda_compile_ptx_generated_whitted.cu.ptx
	cd /home/frank/15618_project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/frank/15618_project /home/frank/15618_project/src/sutil /home/frank/15618_project/build /home/frank/15618_project/build/src/sutil /home/frank/15618_project/build/src/sutil/CMakeFiles/sutil_7_sdk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/sutil/CMakeFiles/sutil_7_sdk.dir/depend

