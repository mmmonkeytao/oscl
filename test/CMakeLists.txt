 
include_directories(${OSCL_HMP_INCLUDE})
include_directories(${OSCL_ENGINE_INCLUDE})
add_executable(hmp_test src/hmp_test.cpp)
target_link_libraries(hmp_test ${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})
