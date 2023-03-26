cmake_minimum_required(VERSION 3.1)

#add_definitions(-DEXCLUDE_LOG_MODULE)
#add_compile_definitions(MG_ENABLE_OPENSSL=1)

# cpp version (be different for other platform, TODO).
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(testikrigretarget)

################## src
file(GLOB mysrc "./src/*.cpp")

# add include directory
include_directories(./src)
include_directories(SYSTEM ./thirdparty/include)

################## include
# add all source files
add_executable(testikrigretarget  
    ${mysrc}
)

################## link

message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

if(APPLE)

    target_link_libraries(testikrigretarget PRIVATE ikrigretarget)


else()

    #find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL GLX)
    #message(STATUS "OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
    # include
    #include_directories( ${OPENGL_INCLUDE_DIR} )
    # lib
    #CMAKE_CURRENT_SOURCE_DIR 
    set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp)
    message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
    find_library(ASSIMP_LIB NAMES assimp-vc143-mt.lib PATHS ${LIBS_DIR} NO_DEFAULT_PATH)
    message(STATUS "ASSIMP_LIB: ${ASSIMP_LIB}")
    target_link_libraries(testikrigretarget PRIVATE  ikrigretarget ${ASSIMP_LIB})

endif()