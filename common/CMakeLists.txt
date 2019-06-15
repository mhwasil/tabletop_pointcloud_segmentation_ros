#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH lib)

include_directories(include)
set(srcs src/common.cc)
add_library (common ${srcs})
target_link_libraries(common ${PCL_LIBRARIES})

#set(srcs src/plane_segmentation.cpp)
#add_library (plane_segmentation ${srcs})
#target_link_libraries(plane_segmentation ${PCL_LIBRARIES})

#set(srcs src/bounding_box.cpp)
#add_library (bounding_box ${srcs})
#target_link_libraries(bounding_box ${PCL_LIBRARIES})
