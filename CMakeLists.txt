cmake_minimum_required(VERSION 3.0 FATAL_ERROR) # required by cmake eigen finder to find locally installed versions (e.g., CUDA 9.2 support was added in eigen 3.3.5)

include(${CMAKE_SOURCE_DIR}/cmake/Utils.cmake)

project( 2dFighter )

### FLAGS
if (WIN32)
  # TODO -- Makeshift solution -- This prevents rerunning build again
  # https://gitlab.kitware.com/cmake/cmake/issues/16783
  set(CMAKE_SUPPRESS_REGENERATION ON)

  string (REPLACE "/D_WINDOWS" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string (REPLACE "/DWIN32" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

  # /GL option causes the code to crash -- fix this
  # sdl flags causes error -- error : unknown attribute \"guard\"

  # https://stackoverflow.com/questions/43697154/detect-current-cmake-version-using-cmake
  message(STATUS "CMAKE_VERSION = ${CMAKE_VERSION}")
  # Remove "minSizeRel" and "relWithDebInfo"
  if (${CMAKE_VERSION} VERSION_LESS "3.14.0")
    set(CMAKE_CONFIGURATION_TYPES Release Debug CACHE TYPE INTERNAL FORCE)
  else (${CMAKE_VERSION} VERSION_LESS "3.14.0")
    # But the above line gives warning on CMake >= 3.14, alternative
    set(CMAKE_CONFIGURATION_TYPES Release Debug CACHE STRING "" FORCE)
  endif (${CMAKE_VERSION} VERSION_LESS "3.14.0")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
  string(REPLACE "/W3" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "/GR" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

  string(REPLACE "/O2" "/Ox" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
  string(REPLACE "/Ob2" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ot /Oi /Gy /Z7")

  set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG:incremental /OPT:REF /OPT:ICF")

  string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  string(REPLACE "/Zi" "/Z7" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  string(REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
elseif (UNIX)
  # Turn on C++11
  add_definitions(-std=c++11)
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
elseif (APPLE)
  # Turn on C++11
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif (WIN32)

# C++ additional flags
if (CMAKE_COMPILER_IS_GNUCXX)
  message(STATUS "GCC detected, adding compile flags")
  # set(OP_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wpedantic -Wall -Wextra -Wfatal-errors")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
  set(OP_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wall -Wextra -Wfatal-errors")
  # set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Wpedantic -Wall -Wextra -Wfatal-errors")
endif (CMAKE_COMPILER_IS_GNUCXX)


### DOWNLOAD MODELS
# Download the models if flag is set
message(STATUS "Download the models.")

# Download the models
option(DOWNLOAD_BODY_25_MODEL "Download body 25-keypoint (body COCO and 6-keypoint foot) model." ON)
option(DOWNLOAD_BODY_COCO_MODEL "Download body 18-keypoint COCO model." OFF)
option(DOWNLOAD_BODY_MPI_MODEL "Download body 15-keypoint MPI model." OFF)
option(DOWNLOAD_FACE_MODEL "Download face model." ON)
option(DOWNLOAD_HAND_MODEL "Download hand model." ON)

# URL to the models
set(OPENPOSE_URL "http://posefs1.perception.cs.cmu.edu/OpenPose/models/")

download_model("BODY_25" ${DOWNLOAD_BODY_25_MODEL} pose/body_25/pose_iter_584000.caffemodel
  78287B57CF85FA89C03F1393D368E5B7) # Body (BODY_25)
download_model("body (COCO)" ${DOWNLOAD_BODY_COCO_MODEL} pose/coco/pose_iter_440000.caffemodel
  5156d31f670511fce9b4e28b403f2939) # Body (COCO)
download_model("body (MPI)" ${DOWNLOAD_BODY_MPI_MODEL} pose/mpi/pose_iter_160000.caffemodel
  2ca0990c7562bd7ae03f3f54afa96e00) # Body (MPI)
download_model("face" ${DOWNLOAD_FACE_MODEL} face/pose_iter_116000.caffemodel
  e747180d728fa4e4418c465828384333) # Face
download_model("hand" ${DOWNLOAD_HAND_MODEL} hand/pose_iter_102000.caffemodel
  a82cfc3fea7c62f159e11bd3674c1531) # Hand

message(STATUS "Models Downloaded.")

### FIND REQUIRED PACKAGES
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

# Third party libraries
set(FIND_LIB_PREFIX ${CMAKE_SOURCE_DIR}/3rdparty/windows)

# Download Windows 3rd party
message(STATUS "Downloading windows dependencies...")
set(OP_WIN_URL "http://posefs1.perception.cs.cmu.edu/OpenPose/3rdparty/windows")
set(OP_WIN_DIR "${CMAKE_SOURCE_DIR}/3rdparty/windows")

# Download required zip files
  download_zip("opencv_411_v14_15_2019_09_24.zip" ${OP_WIN_URL} ${FIND_LIB_PREFIX} cf080121a8189ecb7f53ac9b0fef36c6)
message(STATUS "Windows dependencies downloaded.")

# Finding opencv
find_library(OpenCV_LIBS opencv_world411 HINTS ${FIND_LIB_PREFIX}/opencv/x64/vc15/lib)
find_library(GFLAGS_LIBRARY_RELEASE gflags HINTS ${FIND_LIB_PREFIX}/caffe3rdparty/lib)
find_library(GFLAGS_LIBRARY_DEBUG gflagsd HINTS ${FIND_LIB_PREFIX}/caffe3rdparty/lib)
find_library(OPENPOSE_LIBS openposed HINTS ${FIND_LIB_PREFIX}/openpose/lib)

set(OpenCV_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/windows/opencv/include")
set(Openpose_INCLUDE_DIRS "${FIND_LIB_PREFIX}/openpose/include")
set(GFLAGS_INCLUDE_DIR "${FIND_LIB_PREFIX}/caffe3rdparty/include")

include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${Openpose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS})

# Finding gflags for openpose
find_package(GFlags REQUIRED)

### COLLECT ALL 3RD-PARTY LIBRARIES TO BE LINKED AGAINST
set(OpenPose_3rdparty_libraries ${OpenCV_LIBS} ${GLOG_LIBRARY} ${OPENPOSE_LIBS})
set(OpenPose_3rdparty_libraries ${OpenPose_3rdparty_libraries} ${GFLAGS_LIBRARY})

set(EXE_NAME 2dfighter)
set(FILES main.cpp keyboard_simulator.cpp keyboard_simulator.hpp)

message(STATUS "Adding source code ${EXE_NAME}")

add_executable(${EXE_NAME} ${FILES})
target_link_libraries(${EXE_NAME}  ${OpenPose_3rdparty_libraries})

if (WIN32)
  set_property(TARGET ${EXE_NAME} PROPERTY FOLDER "Examples/Tutorial/C++ API")
  configure_file(${CMAKE_SOURCE_DIR}/cmake/OpenPose${VCXPROJ_FILE_GPU_MODE}.vcxproj.user
      ${CMAKE_CURRENT_BINARY_DIR}/${EXE_NAME}.vcxproj.user @ONLY)
  # Properties->General->Output Directory
  set_property(TARGET ${EXE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/$(Platform)/$(Configuration))
  set_property(TARGET ${EXE_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/$(Platform)/$(Configuration))
endif (WIN32)