# hmp_test.cpp 
include_directories(${OSCL_HMP_INCLUDE})
include_directories(${OSCL_ENGINE_INCLUDE})
add_executable(hmp_test src/hmp_test.cpp)
target_link_libraries(hmp_test ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})

#hmp_rgbdeval.cpp
add_executable(hmp_rgbdeval src/hmp_rgbdeval.cpp)
target_link_libraries(hmp_rgbdeval ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})