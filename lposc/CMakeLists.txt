set(SOURCES src/LPOSC.cpp
            src/save_eval_files.cpp
	    src/insert.cpp
	    src/label_prop.cpp
	    src/insert1.cpp
	    src/insert2.cpp
            )

set(HEADER_SUBDIR "include")

set(PUBLIC_HEADERS include/LPOSC.h
                   #include/Fmeasure.h
                   )

set(OSCL_LPOSC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OSCL_LPOSC_LIBRARY oscl_lposc)

include_directories(${OSCL_LPOSC_INCLUDE})
include_directories(${OSCL_CLUST_INCLUDE})

cache_internal_init_unique(OSCL_LPOSC_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
cache_internal_init_unique(OSCL_LPOSC_LIBRARY oscl_lposc)

add_library(${OSCL_LPOSC_LIBRARY} ${SOURCES} ${PUBLIC_HEADERS})
target_link_libraries(${OSCL_LPOSC_LIBRARY} ${OSCL_CLUST_LIBRARY})

cache_internal_append_unique(OSCL_LIBRARIES ${OSCL_LPOSC_LIBRARY})