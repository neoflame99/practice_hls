cmake_minimum_required(VERSION 3.21)
project(jbfifo_verification LANGUAGES CXX)

include(CTest)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SYSTEMC_HOME "$ENV{SYSTEMC_HOME}" CACHE PATH "SystemC installation prefix")
set(CRAVE_HOME "$ENV{CRAVE_HOME}" CACHE PATH "CRAVE installation prefix")
if(CRAVE_HOME)
    list(PREPEND CMAKE_PREFIX_PATH "${CRAVE_HOME}")
else()
    list(APPEND CMAKE_PREFIX_PATH /opt/crave)
endif()
set(UVM_SYSTEMC_HOME "$ENV{UVM_SYSTEMC_HOME}" CACHE PATH
    "UVM-SystemC installation prefix")
set(CONNECTIONS_HOME "$ENV{CONNECTIONS_HOME}" CACHE PATH
    "MatchLib Connections installation prefix")
set(AC_TYPES_HOME "$ENV{AC_TYPES_HOME}" CACHE PATH
    "AC Types installation prefix")
set(MATCHLIB_HOME "$ENV{MATCHLIB_HOME}" CACHE PATH
    "MatchLib installation prefix")
set(HLSLIB_HOME "${CMAKE_CURRENT_LIST_DIR}/../../../hlslib" CACHE PATH
    "HLSLib installation prefix")

option(FIFO_CRAVE_VERBOSE "Show CRAVE solver logs" OFF)

if(FIFO_CRAVE_VERBOSE)
    target_compile_definitions(jbfifo_testbench PRIVATE FIFO_CRAVE_VERBOSE)
endif()

find_path(SYSTEMC_INCLUDE_DIR
    NAMES systemc.h
    HINTS "${SYSTEMC_HOME}"
    PATHS /opt/systemc
    PATH_SUFFIXES include
)
find_library(SYSTEMC_LIBRARY
    NAMES systemc
    HINTS "${SYSTEMC_HOME}"
    PATHS /opt/systemc
    PATH_SUFFIXES lib lib64 lib-macos lib-macosarm64
)
find_path(UVM_SYSTEMC_INCLUDE_DIR
    NAMES uvm
    HINTS "${UVM_SYSTEMC_HOME}"
    PATHS /opt/uvm-systemc
    PATH_SUFFIXES include
)
find_library(UVM_SYSTEMC_LIBRARY
    NAMES uvm-systemc uvm
    HINTS "${UVM_SYSTEMC_HOME}"
    PATHS /opt/uvm-systemc
    PATH_SUFFIXES lib lib64 lib-macos lib-macosarm64 lib-linux lib-linux64
)
find_path(CONNECTIONS_INCLUDE_DIR
    NAMES connections/connections.h
    HINTS "${CONNECTIONS_HOME}" "${MATCHLIB_HOME}"
    PATHS "${CMAKE_CURRENT_LIST_DIR}/../../../Mgc_inc"
    PATH_SUFFIXES include
)
find_path(AC_TYPES_INCLUDE_DIR
    NAMES ac_int.h
    HINTS "${AC_TYPES_HOME}" "${HLSLIB_HOME}"
    PATHS "${CMAKE_CURRENT_LIST_DIR}/../../../hlslib"
    PATH_SUFFIXES include ac_types/include
)

if(NOT SYSTEMC_INCLUDE_DIR OR NOT SYSTEMC_LIBRARY OR
   NOT CONNECTIONS_INCLUDE_DIR OR NOT AC_TYPES_INCLUDE_DIR)
    message(FATAL_ERROR
        "Required dependencies were not found. Set SYSTEMC_HOME, "
        "CONNECTIONS_HOME (or MATCHLIB_HOME), and AC_TYPES_HOME "
        "(or HLSLIB_HOME), or pass the corresponding *_INCLUDE_DIR "
        "and SYSTEMC_LIBRARY CMake cache variables."
    )
endif()

if(NOT UVM_SYSTEMC_INCLUDE_DIR OR NOT UVM_SYSTEMC_LIBRARY)
    message(FATAL_ERROR
        "UVM-SystemC was not found. Set UVM_SYSTEMC_HOME or pass "
        "UVM_SYSTEMC_INCLUDE_DIR and UVM_SYSTEMC_LIBRARY."
    )
endif()

# Reuse the Boost installation produced by the local CRAVE build when present.
find_package(Boost CONFIG QUIET COMPONENTS filesystem HINTS
    "${CMAKE_CURRENT_LIST_DIR}/../../../../ToolBuild/pack_sysc/crave/build/Default/_deps/install"
)
get_filename_component(SystemCLanguage_ROOT "${SYSTEMC_INCLUDE_DIR}" DIRECTORY)
find_package(crave CONFIG REQUIRED)
find_path(FC4SC_INCLUDE_DIR NAMES fc4sc.hpp
    HINTS "$ENV{FC4SC_HOME}"
    PATHS /opt/fc4sc
    PATH_SUFFIXES includes include
    REQUIRED
)

add_executable(jbfifo_testbench jbfifo_testbench.cpp)
target_compile_features(jbfifo_testbench PRIVATE cxx_std_17)
target_include_directories(jbfifo_testbench PRIVATE
    "${SYSTEMC_INCLUDE_DIR}"
    "${UVM_SYSTEMC_INCLUDE_DIR}"
    "${CONNECTIONS_INCLUDE_DIR}"
    "${AC_TYPES_INCLUDE_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../src"
)
target_include_directories(jbfifo_testbench SYSTEM PRIVATE "${FC4SC_INCLUDE_DIR}")
target_link_libraries(jbfifo_testbench PRIVATE crave::crave "${UVM_SYSTEMC_LIBRARY}" "${SYSTEMC_LIBRARY}")
target_compile_definitions(jbfifo_testbench PRIVATE
    ENABLE_AC_VCD_TRACE=0
    SC_INCLUDE_DYNAMIC_PROCESSES
    CONNECTIONS_FAST_SIM
)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(jbfifo_testbench PRIVATE
        "$<$<CONFIG:Debug>:-O0>"
        "$<$<CONFIG:Debug>:-g3>"
    )
endif()

if(BUILD_TESTING)
    add_test(NAME jbfifo_testbench COMMAND jbfifo_testbench FifoOrderTest
        "${CMAKE_CURRENT_BINARY_DIR}/jbfifo_order"
        "${CMAKE_CURRENT_BINARY_DIR}/jbfifo_order.ucis.xml")
    add_test(NAME jbfifo_full COMMAND jbfifo_testbench FifoFullTest
        "${CMAKE_CURRENT_BINARY_DIR}/jbfifo_full"
        "${CMAKE_CURRENT_BINARY_DIR}/jbfifo_full.ucis.xml")
    add_test(NAME jbfifo_randomize COMMAND jbfifo_testbench FifoRandomizeTest)
    set_tests_properties(jbfifo_testbench jbfifo_full jbfifo_randomize PROPERTIES TIMEOUT 15)
endif()
