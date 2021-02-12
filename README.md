# Bachelor

Tools
Visual Studio
    CMake Tools - sollte automatisch von Visual Studio installiert werden
CMake
    sudo apt-get install cmake
    sudo apt-get install cmake.qt-gui
GCC
    sudo apt-get install gcc
PCL
    Abhänigkeiten
        sudo apt-get install libboost-all-dev
        sudo apt-get install libeigen3-dev
        sudo apt-get install libflann-dev
        sudo apt-get install libopenni-dev 
        sudo apt-get install libqhull-dev
        sudo apt-get install libvtk6-dev
        sudo apt-get install libvtk6-qt-dev
    PCL selbst
        Download link https://github.com/PointCloudLibrary/pcl/releases (Verwendet wurde 1.10.1)
        cd pcl-pcl-1..1 # version anpassen
        mkdir build && cd build
        cmake ..
        Optinal: bei error (The CXX compiler identification is unknown), cmake -DCMAKE_CXX_COMPILER=/usr/bin/c++ an aktueller stelle ausführen
        ccmake ..


QHULL