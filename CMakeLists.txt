#
# Copyright (C) 2021, Xilinx Inc - All rights reserved
# Xilinx Decoder/Encoder XMA Plugin
#
# Licensed under the Apache License, Version 2.0 (the "License"). You may
# not use this file except in compliance with the License. A copy of the
# License is located at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#

cmake_minimum_required(VERSION 2.8.11)
include (CMakeDependentOption)
set(XMPSOC_PROJ "xilinx-u30-xmpsoccodecs")
set(XMPSOC_PROJ_VERSION "1.1.0")

find_program(UNAME_EXEC uname)
find_program(LSB_RELEASE_EXEC lsb_release)

# Grab Arch information like x86-64
execute_process(COMMAND ${UNAME_EXEC} -p
    OUTPUT_VARIABLE UNAME_ARCHTYPE
    OUTPUT_STRIP_TRAILING_WHITESPACE)
# Grab OS release short name
execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
	OUTPUT_VARIABLE LSB_RELEASE_REL_SHORT
	OUTPUT_STRIP_TRAILING_WHITESPACE)

set(XMPSOC_PROJ_NAME "${XMPSOC_PROJ}_${XMPSOC_PROJ_VERSION}_${LSB_RELEASE_REL_SHORT}-${UNAME_ARCHTYPE}")

project(${XMPSOC_PROJ})
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/lib/pkgconfig")

# Linking variables
find_package(PkgConfig REQUIRED)
pkg_check_modules(XRT REQUIRED	xrt)
pkg_check_modules(XMA REQUIRED libxma2api)
pkg_check_modules(XVBM REQUIRED xvbm)

set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XVBM_LDFLAGS} ${XRT_LDFLAGS} --stats -Bsymbolic")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS	"${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -DDSA64 -fpermissive")

# Bring the headers into the project
include_directories(include)

# Decoder Plugin
add_library(vcu-xma-dec-plg SHARED
	dec/vcu-xma-dec-plg.cpp
)

target_compile_options(vcu-xma-dec-plg
	PUBLIC	${XRT_CFLAGS}
	PUBLIC	${XMA_CFLAGS}
	PUBLIC	${XVBM_CFLAGS}
)

target_include_directories(vcu-xma-dec-plg
	PUBLIC ${XRT_INCLUDE_DIRS}/xma2
	PUBLIC ${XRT_INCLUDE_DIRS}
	PUBLIC ${XVBM_INCLUDE_DIRS}
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vcu-xma-dec-plg ${XVBM_LDFLAGS})

# Encoder Plugin
add_library(vcu-xma-enc-plg SHARED
	enc/vcu-xma-enc-plg.cpp
)

target_compile_options(vcu-xma-enc-plg
	PUBLIC	${XRT_CFLAGS}
	PUBLIC	${XMA_CFLAGS}
	PUBLIC	${XVBM_CFLAGS}
)

target_include_directories(vcu-xma-enc-plg
	PUBLIC ${XRT_INCLUDE_DIRS}/xma2
	PUBLIC ${XRT_INCLUDE_DIRS}
	PUBLIC ${XVBM_INCLUDE_DIRS}
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vcu-xma-enc-plg ${XVBM_LDFLAGS})

# Set the location for library installation
install(TARGETS vcu-xma-dec-plg DESTINATION ${CMAKE_INSTALL_PREFIX}/xma_plugins)
install(TARGETS vcu-xma-enc-plg DESTINATION ${CMAKE_INSTALL_PREFIX}/xma_plugins)

# Packaging section
set(CPACK_PACKAGE_NAME "${XMPSOC_PROJ}")
set(CPACK_PACKAGE_FILE_NAME "${XMPSOC_PROJ_NAME}")
set(CPACK_PACKAGE_VERSION "${XMPSOC_PROJ_VERSION}")
set(CPACK_PACKAGE_CONTACT "Rohit Athavale <rohit.athavale@xilinx.com>")
set(CPACK_PACKAGE_VENDOR "Xilinx")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Xilinx MPSoC H264/HEVC Encoder and Decoder Plugins")
set(CPACK_PACKAGE_DESCRIPTION "The Xilinx MPSoC based encoder and decoder plugins that control
							   the accelerated FPGA video codec kernels used to perform H264/HEVC
							   encoding and decoding respecitvely")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "xrt (>= 2.11), xilinx-xvbm (>= 1.0.2) ")
set(CPACK_RPM_PACKAGE_DEPENDS "xrt (>= 2.11), xilinx-xvbm (>= 1.0.2) ")
set(CPACK_RPM_EXCLUDE_FROM_AUTO_FILELIST_ADDITION "/opt" "/opt/xilinx" "/usr/lib" "/usr/lib/pkgconfig")
include(CPack)
