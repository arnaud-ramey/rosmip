add_executable(rosmip.exe
  rosmip.cpp
  ../include/rosmip/rosmip.h
  ../third_parties/libmip/src/bluetooth_mac2device.h
  ../third_parties/libmip/src/gattmip.h
)
target_link_libraries(rosmip.exe  ${catkin_LIBRARIES}  libgatt  ${GLIB_LIBRARIES})

add_executable(mip_teleop_joy.exe          mip_teleop_joy.cpp)
target_link_libraries(mip_teleop_joy.exe   ${catkin_LIBRARIES})

add_executable(sound_player.exe            sound_player.cpp)
target_link_libraries(sound_player.exe     ${catkin_LIBRARIES})
