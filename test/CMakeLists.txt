cmake_minimum_required(VERSION 3.1)

#add_definitions(-DEXCLUDE_LOG_MODULE)
#add_compile_definitions(MG_ENABLE_OPENSSL=1)

# cpp version (be different for other platform, TODO).
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(testikrigretarget)

################## src
file(GLOB mysrc "./main.cpp" "./src/*.cpp"  "./src/*.h")

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

# set(EMBED_ASSIMP ON)
if(UNIX AND NOT APPLE)
    ### force embed
    if(EMBED_ASSIMP)
        target_link_libraries(testikrigretarget PRIVATE  libikrigretarget assimp)
    endif()
elseif(APPLE)

    if(EMBED_ASSIMP)
        target_link_libraries(testikrigretarget PRIVATE  libikrigretarget assimp)
    else()
        set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/macos/assimp)
        find_library(ASSIMP_LIB NAMES libassimp.dylib PATHS ${LIBS_DIR} NO_DEFAULT_PATH)
        target_link_libraries(testikrigretarget PRIVATE  libikrigretarget ${ASSIMP_LIB})
    endif()
else()

    if(EMBED_ASSIMP)
        target_link_libraries(testikrigretarget PUBLIC assimp libikrigretarget)
        add_custom_command(
            TARGET testikrigretarget POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory 
            $<TARGET_FILE_DIR:assimp>
            $<TARGET_FILE_DIR:testikrigretarget> 
        )
    else()
        set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp)

        # decide lib during cmake
        # find_library(ASSIMP_LIB NAMES ${ASSIMP_LIB_NAME} PATHS ${LIBS_DIR} NO_DEFAULT_PATH)

        # decide lib during vs build
        add_library(ASSIMP_LIB STATIC IMPORTED)
        set_target_properties(ASSIMP_LIB PROPERTIES 
            IMPORTED_LOCATION_DEBUG ${LIBS_DIR}/Debug/assimp-vc143-mtd.lib
            IMPORTED_LOCATION_RELEASE ${LIBS_DIR}/Release/assimp-vc143-mt.lib
            IMPORTED_LOCATION_MINSIZEREL ${LIBS_DIR}/Release/assimp-vc143-mt.lib
            IMPORTED_LOCATION_RELWITHDEBINFO ${LIBS_DIR}/Release/assimp-vc143-mt.lib
        )
        target_link_libraries(testikrigretarget PRIVATE libikrigretarget ASSIMP_LIB)

        # decide dll to copy during vs build
        add_custom_command(TARGET testikrigretarget POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different 
            $<$<CONFIG:Debug>:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp/Debug/assimp-vc143-mtd.dll>
            $<$<CONFIG:Release>:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp/Release/assimp-vc143-mt.dll>
            $<$<CONFIG:MinSizeRel>:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp/Release/assimp-vc143-mt.dll>
            $<$<CONFIG:RelWithDebInfo>:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/lib/win/assimp/Release/assimp-vc143-mt.dll>
            $<TARGET_FILE_DIR:testikrigretarget> 
        )
    endif()

endif()