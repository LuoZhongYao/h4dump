cmake_minimum_required(VERSION 3.8)

project(btuart)
set(CMAKE_C_FLAGS_DEBUG "-g -fsanitize=address")

find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB2 REQUIRED IMPORTED_TARGET glib-2.0)

include_directories(.)
add_executable(btuart
   main.c
   baudrate.c
   )

target_link_libraries(btuart PRIVATE PkgConfig::GLIB2)
target_link_libraries(btuart PRIVATE mon)

add_library(mon STATIC
   lib/hci.c
   lib/uuid.c
   lib/bluetooth.c
   src/log.c
   src/textfile.c
   src/settings.c
   src/shared/log.c
   src/shared/util.c
   src/shared/queue.c
   src/shared/crypto.c
   src/shared/gatt-db.c
   src/shared/bap-debug.c
   monitor/a2dp.c
   # monitor/analyze.c
   monitor/att.c
   monitor/avctp.c
   monitor/avdtp.c
   monitor/bnep.c
   monitor/broadcom.c
   # monitor/control.c
   monitor/display.c
   monitor/crc.c
   monitor/hwdb.c
   monitor/intel.c
   monitor/keys.c
   monitor/l2cap.c
   monitor/ll.c
   monitor/lmp.c
   monitor/msft.c
   monitor/packet.c
   monitor/rfcomm.c
   monitor/sdp.c
   monitor/vendor.c
)
target_include_directories(mon PRIVATE ${GLIB2_INCLUDE_DIRS})
target_compile_definitions(mon PRIVATE STORAGEDIR="/tmp" VERSION="0.0.1")
