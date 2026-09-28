cmake_minimum_required(VERSION 3.16)
project(ethercat_slave_core_test LANGUAGES CXX)

add_executable(ethercat_slave_core_test
  main.cpp
  ../../../src/protocols/ethercat/ethercat_process_data.cpp
  ../../../src/chassis_controller/communication/ethercat_slave_core.cpp
  ../../../src/chassis_controller/communication/ethercat_ssc_bridge.cpp
  ../../../src/protocols/ethercat/ethercat_sii_store.cpp
)
target_include_directories(ethercat_slave_core_test PRIVATE ../../../src)
target_compile_features(ethercat_slave_core_test PRIVATE cxx_std_17)
target_compile_options(ethercat_slave_core_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME ethercat_slave_core_test COMMAND ethercat_slave_core_test)
