set(SOURCES src/Vertex.cpp  
            #src/Cluster.cpp  
	    src/OnlineStarClustering.cpp  
	    #src/ClusterList.cpp  
	    #src/Mesh.cpp
	    src/fastInsert.cpp
	    src/fastUpdate.cpp
	    src/Vmeasure.cpp
	    src/fastDelete.cpp
	    src/export.cpp
	    src/helper.cpp
	    src/loadata.cpp
	    src/insert.cpp
	    src/similarity.cpp
	    )

set(HEADER_SUBDIR "include")

set(PUBLIC_HEADERS include/Vertex.h
                   #include/KernelOSC.h  
		   #include/Mesh.h  
		   #include/Cluster.h  
		   include/OnlineStarClustering.h  
		   #include/ClusterList.h
		   )


set(OSCL_CLUST_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OSCL_CLUST_LIBRARY oscl_clustering)

include_directories(${OSCL_CLUST_INCLUDE})

cache_internal_init_unique(OSCL_CLUST_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
cache_internal_init_unique(OSCL_CLUST_LIBRARY oscl_clustering)

add_library(${OSCL_CLUST_LIBRARY} ${SOURCES} ${PUBLIC_HEADERS})
target_link_libraries(${OSCL_CLUST_LIBRARY} ${GSL_LIBRARIES})

cache_internal_append_unique(OSCL_LIBRARIES ${OSCL_CLUST_LIBRARY})