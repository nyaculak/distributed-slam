#-----------------------------------------------------------------------------------------------
#  Run with "ccmake ." at the root directory, or use it as a template for
#   starting your own programs
#-----------------------------------------------------------------------------------------------
SET(sampleName DistributedSlam)
SET(PRJ_NAME "EXAMPLE_${sampleName}")

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
PROJECT(${PRJ_NAME})

# These commands are needed by modern versions of CMake:
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# ---------------------------------------------------------------------------
# Set the output directory of each example to its corresponding subdirectory
#  in the binary tree:
# ---------------------------------------------------------------------------
SET(EXECUTABLE_OUTPUT_PATH ".")

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED slam;vision;gui;topography;hwdrivers;obs)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
SET(SRCS
		main.cpp
		SerialPort.cpp SerialPort.h
		KinectGrabber.h
		RobotContainer.h
		)
# Define the executable target:
ADD_EXECUTABLE(${sampleName} ${SRCS})

SET_TARGET_PROPERTIES(
	${sampleName}
	PROPERTIES
	PROJECT_LABEL "(EXAMPLE) ${sampleName}")

# Add special defines needed by this example, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${sampleName}
	${MRPT_LIBS}  # This is filled by FIND_PACKAGE(MRPT ...)
	"-lboost_system"
	"-lboost_thread"
	"-l pthread"
	"-g -Wall -Werror"
	)

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
