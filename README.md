
# platform

    1. mac 
    2. linux
    3. windows

# windows build and test c++

    //./build.sh 

    rm -rf build
    mkdir build && cd build

    cmake ..
    
    open ikrigretarget.sln with visual studio
    set testikrigretarget as start project
    
    or
    
    cmake --build .
    test/Debug/testikrigretarget.exe

# release files

    release/libikrigretarget.a

# build release
    
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make VERBOSE=1

# prerequisite

    