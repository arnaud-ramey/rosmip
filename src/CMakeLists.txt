add_executable(rosmip.exe
  rosmip.cpp
  ../include/rosmip/rosmip.h
  ../third_parties/libmip/src/bluetooth_mac2device.h
  ../third_parties/libmip/src/gattmip.h
)
target_link_libraries(rosmip.exe  ${catkin_LIBRARIES}  libgatt  ${GLIB_LIBRARIES})
