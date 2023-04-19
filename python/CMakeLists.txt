cmake_minimum_required(VERSION 3.1)

#add_definitions(-DEXCLUDE_LOG_MODULE)
#add_compile_definitions(MG_ENABLE_OPENSSL=1)

# cpp version (be different for other platform, TODO).
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(PYTHON_MODULE_NAME ikrigretarget)
project(${PYTHON_MODULE_NAME})


################## pybind11
add_subdirectory(pybind11)


################## src
file(GLOB mysrc "./src/*.cpp"  "./src/*.h" "./src/*.hpp" "../test/src/*.cpp" "../test/src/*.h" "../test/src/*.hpp")

################## include
include_directories(./src ../test/src ../test/thirdparty/include)


################## app
pybind11_add_module(${PYTHON_MODULE_NAME} ${mysrc})
# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(${PYTHON_MODULE_NAME} PRIVATE VERSION_INFO=${MOUDLE_VERSION_INFO})

################## link

message(STATUS "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")

if(APPLE)
    if(EMBED_ASSIMP)
        target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE  libikrigretarget assimp)
    else()
        set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../test/thirdparty/lib/macos/assimp)
        find_library(ASSIMP_LIB NAMES libassimp.dylib PATHS ${LIBS_DIR} NO_DEFAULT_PATH)
        target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE  libikrigretarget ${ASSIMP_LIB})
    endif()
else()
    if(EMBED_ASSIMP)
        target_link_libraries(${PYTHON_MODULE_NAME} PUBLIC assimp libikrigretarget)
        if(BUILD_SHARED_LIBS)
            add_custom_command(
                TARGET ${PYTHON_MODULE_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory 
                $<TARGET_FILE_DIR:assimp>
                $<TARGET_FILE_DIR:${PYTHON_MODULE_NAME}> 
            )
        endif()
    else()
        set(LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../test/thirdparty/lib/win/assimp)

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
        target_link_libraries(${PYTHON_MODULE_NAME} PRIVATE libikrigretarget ASSIMP_LIB)

        # decide dll to copy during vs build
        add_custom_command(TARGET ${PYTHON_MODULE_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different 
            $<$<CONFIG:Debug>:${LIBS_DIR}/Debug/assimp-vc143-mtd.dll>
            $<$<CONFIG:Release>:${LIBS_DIR}/Release/assimp-vc143-mt.dll>
            $<$<CONFIG:MinSizeRel>:${LIBS_DIR}/Release/assimp-vc143-mt.dll>
            $<$<CONFIG:RelWithDebInfo>:${LIBS_DIR}/Release/assimp-vc143-mt.dll>
            $<TARGET_FILE_DIR:testikrigretarget> 
        )
    endif()
endif()
