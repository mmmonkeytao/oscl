set(SOURCES src/Galaxy.cpp
            src/Planet.cpp
	    src/GalaxyIO.cpp
	    src/galaxy_Vmeasure.cpp
	    src/computeSimilarity.cpp
	    src/LoadFeaPool.cpp
	    src/unloaded_insert.cpp
	    src/loaded_insert_noSim.cpp
	    src/Hquery_accuracy.cpp
	    src/oscInsert.cpp
	    src/helper.cpp
	    src/oscUpdate.cpp
	    src/guided_osc_insert.cpp
	    src/normal_osc_insert.cpp
	    src/nearest_neighbors_insert.cpp
	    )

set(HEADER_SUBDIR "include")

set(PUBLIC_HEADERS include/Planet.h
                   include/Galaxy.h
		   		   )

set(OSCL_GALAXY_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OSCL_GALAXY_LIBRARY oscl_galaxy)

include_directories(${OSCL_GALAXY_INCLUDE})

cache_internal_init_unique(OSCL_GALAXY_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
cache_internal_init_unique(OSCL_GALAXY_LIBRARY oscl_galaxy)

add_library(${OSCL_GALAXY_LIBRARY} ${SOURCES} ${PUBLIC_HEADERS})
target_link_libraries(${OSCL_GALAXY_LIBRARY} ${STXXL_LIBRARIES})

cache_internal_append_unique(OSCL_LIBRARIES ${OSCL_GALAXY_LIBRARY})