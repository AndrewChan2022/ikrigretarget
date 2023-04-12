cmake_minimum_required(VERSION 3.1)

#add_definitions(-DEXCLUDE_LOG_MODULE)
#add_compile_definitions(MG_ENABLE_OPENSSL=1)

# cpp version (be different for other platform, TODO).
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(testikrigretarget)

################## src
file(GLOB mysrc "./src/*.cpp"  "./src/*.h")

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

    set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/macos/assimp)
    message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
    find_library(ASSIMP_LIB NAMES libassimp.dylib PATHS ${LIBS_DIR} NO_DEFAULT_PATH)
    message(STATUS "ASSIMP_LIB: ${ASSIMP_LIB}")
    target_link_libraries(testikrigretarget PRIVATE  ikrigretarget ${ASSIMP_LIB})

    message(STATUS "TARGET_RUNTIME_DLLS: $<TARGET_RUNTIME_DLLS:testikrigretarget>")
    message(STATUS "TARGET_FILE_DIR: $<TARGET_RUNTIME_DLLS:testikrigretarget>")

    # add_custom_command(TARGET testikrigretarget POST_BUILD
    #     COMMAND ${CMAKE_COMMAND} -E copy_if_different 
    #     ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp/assimp-vc143-mtd.dll
    #     $<TARGET_FILE_DIR:testikrigretarget> 
    # )

else()

    #find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL GLX)
    #message(STATUS "OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
    # include
    #include_directories( ${OPENGL_INCLUDE_DIR} )
    # lib
    #CMAKE_CURRENT_SOURCE_DIR 

    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(ASSIMP_LIB_NAME "assimp-vc143-mtd.lib")
        set(ASSIMP_DLL_NAME "assimp-vc143-mtd.dll")
    else()
        set(ASSIMP_LIB_NAME "assimp-vc143-mt.lib")
        set(ASSIMP_DLL_NAME "assimp-vc143-mt.dll")
    endif()
    

    set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp)
    message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
    find_library(ASSIMP_LIB NAMES ${ASSIMP_LIB_NAME} PATHS ${LIBS_DIR} NO_DEFAULT_PATH)
    #find_library(ASSIMP_DLL NAMES assimp-vc143-mt.dll PATHS ${LIBS_DIR} NO_DEFAULT_PATH)
    #set(ASSIMP_DLL D:/data/light4/ikrigretarget/test/thirdparty/lib/win/assimp/assimp-vc143-mt.dll)
    message(STATUS "ASSIMP_LIB: ${ASSIMP_LIB}")
    #message(STATUS "ASSIMP_DLL: ${ASSIMP_DLL}")
    target_link_libraries(testikrigretarget PRIVATE  ikrigretarget ${ASSIMP_LIB})

    message(STATUS "TARGET_RUNTIME_DLLS: $<TARGET_RUNTIME_DLLS:testikrigretarget>")
    message(STATUS "TARGET_FILE_DIR: $<TARGET_RUNTIME_DLLS:testikrigretarget>")

    add_custom_command(TARGET testikrigretarget POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different 
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp/${ASSIMP_DLL_NAME}
        $<TARGET_FILE_DIR:testikrigretarget> 
    )

endif()