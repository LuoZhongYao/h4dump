project(btmitm)
cmake_minimum_required(VERSION 3.8)

add_executable(btmitm main.c)
target_link_libraries(btmitm bluetooth)
