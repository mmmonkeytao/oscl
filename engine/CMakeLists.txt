set(SOURCES src/io.cpp
            src/proc.cpp
  )

set(HEADER_SUBDIR "include")

set(PUBLIC_HEADERS ${HEADER_SUBDIR}/io.h
                   ${HEADER_SUBDIR}/proc.h )

# append include files
#list(APPEND GLOBAL_INC_PATH ${PUBLIC_HEADERS})

set(OSCL_ENGINE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(OSCL_ENGINE_LIBRARY oscl_engine)

include_directories(${OSCL_ENGINE_INCLUDE})

cache_internal_init_unique(OSCL_ENGINE_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
cache_internal_init_unique(OSCL_ENGINE_LIBRARY oscl_engine)

add_library(${OSCL_ENGINE_LIBRARY} ${SOURCES} ${PUBLIC_HEADERS})
target_link_libraries(${OSCL_ENGINE_LIBRARY} ${OpenCV_LIBS} -L/usr/local/lib/ -ltbb)

cache_internal_append_unique(OSCL_LIBRARIES ${OSCL_ENGINE_LIBRARY})
