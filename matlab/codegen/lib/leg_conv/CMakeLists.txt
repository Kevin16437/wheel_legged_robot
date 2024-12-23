###########################################################################
# CMakeLists.txt generated for component leg_conv
# Product type: STATIC library
###########################################################################
cmake_minimum_required(VERSION 3.12)
project(leg_conv)

# Propagate the CMAKE_EXPORT_COMPILE_COMMANDS variable from the 
# environment if it is defined as an environment variable, but not as a 
# CMake variable. This is to work around a bug in CMake 3.19 when the 
# "NMake Makefiles" generator is selected.
if(DEFINED ENV{CMAKE_EXPORT_COMPILE_COMMANDS} AND NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
    set(CMAKE_EXPORT_COMPILE_COMMANDS $ENV{CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# Define common variables that are used within the whole project.
set(SYSLIB_PREFIX $<IF:$<AND:$<BOOL:${WIN32}>,$<NOT:$<BOOL:${MINGW}>>>,lib,>)

###########################################################################
## Path variables
###########################################################################
# Derive an absolute path to the code generation anchor folder.
get_filename_component(START_DIR ../../.. ABSOLUTE)

# Special directories defined by using CACHE variables can be overridden 
# by setting the variable from the command line, e.g.,
# 
# 	cmake . -DMATLAB_ROOT=/path/to/another/matlab/root
set(MATLAB_ROOT D:/Matlab2023b CACHE PATH "")

# Additional variables that are defined conditionally.
if("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BINARY_START_DIR "${START_DIR}")
else()
    set(BINARY_START_DIR "${CMAKE_BINARY_DIR}")
endif()

###########################################################################
## System Libraries
###########################################################################
find_library(FOUND_LIBM m NO_SYSTEM_ENVIRONMENT_PATH PATHS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES} ${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES})

###########################################################################
## Target definition and commands
###########################################################################

# Definition of target "leg_conv".
add_library(leg_conv STATIC ${START_DIR}/codegen/lib/leg_conv/rt_nonfinite.c
    ${START_DIR}/codegen/lib/leg_conv/rtGetNaN.c
    ${START_DIR}/codegen/lib/leg_conv/rtGetInf.c
    ${START_DIR}/codegen/lib/leg_conv/leg_conv_initialize.c
    ${START_DIR}/codegen/lib/leg_conv/leg_conv_terminate.c
    ${START_DIR}/codegen/lib/leg_conv/leg_conv.c
    ${START_DIR}/codegen/lib/leg_conv/leg_pos.c
    ${START_DIR}/codegen/lib/leg_conv/leg_spd.c
    ${START_DIR}/codegen/lib/leg_conv/lqr_k.c)

# Set properties for target "leg_conv".
set_target_properties(leg_conv PROPERTIES PREFIX ""
    POSITION_INDEPENDENT_CODE ON
    RUNTIME_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/leg_conv/$<0:>"
    LIBRARY_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/leg_conv/$<0:>"
    ARCHIVE_OUTPUT_DIRECTORY "${BINARY_START_DIR}/codegen/lib/leg_conv/$<0:>")

# Specify language features required for target "leg_conv".
target_compile_features(leg_conv PUBLIC c_std_99)

# Specify compiler preprocessor definitions for target "leg_conv".
target_compile_definitions(leg_conv PRIVATE -DMODEL=leg_conv)

# Specify include directories for target "leg_conv".
target_include_directories(leg_conv PUBLIC $<BUILD_INTERFACE:${START_DIR}/codegen/lib/leg_conv>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/codegen/lib/leg_conv>
    $<BUILD_INTERFACE:${START_DIR}>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>>
    $<BUILD_INTERFACE:${MATLAB_ROOT}/extern/include>)

# Specify library link dependencies for target "leg_conv".  CMake 
# generator expressions are used to create a CMakeLists.txt file that 
# supports multiple platforms with differently named system library 
# dependencies.
target_link_libraries(leg_conv INTERFACE $<$<BOOL:${FOUND_LIBM}>:m>)


###########################################################################
## Build success message
###########################################################################
add_custom_command(TARGET leg_conv POST_BUILD
     COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --cyan "\\#\\#\\# Created STATIC library: $<TARGET_FILE:leg_conv>")

###########################################################################
## Call toolchain hook function if defined
###########################################################################
if(COMMAND toolchain_target_hook)
    toolchain_target_hook(leg_conv)
endif()
