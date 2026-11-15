# CMake generated Testfile for 
# Source directory: C:/Users/Korisnik/Desktop/crowd-navigation
# Build directory: C:/Users/Korisnik/Desktop/crowd-navigation/build-diag
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(grid_vs_bruteforce "C:/Users/Korisnik/Desktop/crowd-navigation/build-diag/test_grid.exe")
set_tests_properties(grid_vs_bruteforce PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;46;add_test;C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;0;")
add_test(sim_neighbors "C:/Users/Korisnik/Desktop/crowd-navigation/build-diag/test_sim.exe")
set_tests_properties(sim_neighbors PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;50;add_test;C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;0;")
add_test(orca_velocity "C:/Users/Korisnik/Desktop/crowd-navigation/build-diag/test_orca.exe")
set_tests_properties(orca_velocity PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;54;add_test;C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;0;")
add_test(orca_validation "C:/Users/Korisnik/Desktop/crowd-navigation/build-diag/test_validation.exe")
set_tests_properties(orca_validation PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;58;add_test;C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;0;")
add_test(sim_validation "C:/Users/Korisnik/Desktop/crowd-navigation/build-diag/test_sim_validation.exe")
set_tests_properties(sim_validation PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;62;add_test;C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;0;")
add_test(map_parsing "C:/Users/Korisnik/Desktop/crowd-navigation/build-diag/test_map.exe")
set_tests_properties(map_parsing PROPERTIES  _BACKTRACE_TRIPLES "C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;66;add_test;C:/Users/Korisnik/Desktop/crowd-navigation/CMakeLists.txt;0;")
subdirs("_deps/raylib-build")
