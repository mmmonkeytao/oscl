set(HEADER_SUBDIR "include")

set(PUBLIC_HEADERS ${HEADER_SUBDIR}/pcloud.h)

set(SOURCES src/pcloud.cpp 
            src/pcloud_RangeImage.cpp
            src/pcloud_segment.cpp) 
#           src/segmentation.cpp 
#           src/cloud_normal.cpp 
#	   src/cluster_extract.cpp)

set(OSCL_PCLOUD_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OSCL_PCLOUD_LIBRARY oscl_pcloud)

include_directories(${OSCL_PCLOUD_INCLUDE})

cache_internal_init_unique(OSCL_PCLOUD_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
cache_internal_init_unique(OSCL_PCLOUD_LIBRARY oscl_pcloud)

add_library(${OSCL_PCLOUD_LIBRARY} ${SOURCES} ${PUBLIC_HEADERS})

target_link_libraries(${OSCL_PCLOUD_LIBRARY} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

cache_internal_append_unique(OSCL_LIBRARIES ${OSCL_PCLOUD_LIBRARY})
