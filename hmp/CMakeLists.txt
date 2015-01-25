set(SOURCES src/hmp.cpp
            src/omp.cpp
	    src/helper.cpp
	    src/depthtonormal.cpp
	    src/loadDicts.cpp
	    src/im2colstep.cpp
	    src/maxpooling.cpp
	    src/computeHMP.cpp
	    ) 

set(HEADER_SUBDIR "include")

set(PUBLIC_HEADERS include/hmp.h)

set(OSCL_HMP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OSCL_HMP_LIBRARY oscl_hmp)

include_directories(${OSCL_ENGINE_INCLUDE})
include_directories(${OSCL_HMP_INCLUDE})

cache_internal_init_unique(OSCL_HMP_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
cache_internal_init_unique(OSCL_HMP_LIBRARY oscl_hmp)

add_library(${OSCL_HMP_LIBRARY} ${SOURCES} ${PUBLIC_HEADERS})
target_link_libraries(${OSCL_HMP_LIBRARY} ${OSCL_ENGINE_LIBRARY})

cache_internal_append_unique(OSCL_LIBRARIES ${OSCL_HMP_LIBRARY})

