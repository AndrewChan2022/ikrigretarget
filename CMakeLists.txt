cmake_minimum_required(VERSION 3.1)

#add_definitions(-DEXCLUDE_LOG_MODULE)
#add_compile_definitions(MG_ENABLE_OPENSSL=1)

# cpp version (be different for other platform, TODO).
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(LIB_NAME ikrigretarget)
project(${LIB_NAME})

################## lib

# add lib then set property 
# or add lib with property
add_library(${LIB_NAME} STATIC)

################## inc
include_directories(./lib)

################## src

file(GLOB APP_HEADER_FILES
    ./lib/*.h
)
file(GLOB APP_SOURCE_FILES
    ./lib/*.cpp
)
# SET(APP_SOURCE_FILES
#     ./lib/*.cpp
# )
SET (ALL_SRCS
    ${APP_HEADER_FILES}
    ${APP_SOURCE_FILES}
)
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${ALL_SRCS})

target_sources(${LIB_NAME} PRIVATE
    ${ALL_SRCS}
)

#file(GLOB mysrc "./lib/*.cpp")


################## lib
#add_library(${LIB_NAME}  STATIC  # SHARED
#    ${mysrc}
#)


################## link

if(APPLE)
else()
    #find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL GLX)
    #message(STATUS "OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
    #target_link_libraries(${LIB_NAME} PRIVATE OpenGL::OpenGL OpenGL::EGL OpenGL::GLX)
endif()

########### other prj
add_subdirectory(test)
