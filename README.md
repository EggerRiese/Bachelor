# Bachelor

Tools
##Visual Studio
    CMake Tools - sollte automatisch von Visual Studio installiert werden
##CMake

    ```bash
    sudo apt-get install cmake
    ```
    ```bash
    sudo apt-get install cmake.qt-gui ?
    ```
    ```bash
    sudo apt-get install cmake-curses-gui
    ```
##GCC
    ```bash
    sudo apt-get install gcc ?
    ```
    ```bash
    sudo apt-get install g++
    ```
##PCL
    ###Abhänigkeiten
        ```bash
        sudo apt-get install libboost-all-dev
        ```
        ```bash
        sudo apt-get install libeigen3-dev
        ```
        ```bash
        sudo apt-get install libflann-dev
        ```
        ```bash
        sudo apt-get install libopenni-dev 
        ```
        ```bash
        sudo apt-get install libqhull-dev
        ```
        ```bash
        sudo apt-get install libvtk6-dev
        ```bash
        sudo apt-get install libvtk6-qt-dev
        ```
    ###PCL selbst
        Download link https://github.com/PointCloudLibrary/pcl/releases (Verwendet wurde 1.10.1)
        ```bash
        cd pcl-pcl-1..1 # version anpassen
        ```
        ```bash
        mkdir build && cd build
        ```
        ```bash
        cmake ..
        ```
        Optinal: bei error (The CXX compiler identification is unknown)
        ```bash
        cmake -DCMAKE_CXX_COMPILER=/usr/bin/c++ 
        ```
        an aktueller stelle ausführen
        ```bash
        ccmake ..
        ```
        ```bash
        make -j(Anzahl gweünschter Prozessoren)
        ```
        ```bash
        sudo make install
        ```
##QHULL
    https://github.com/qhull/qhull
    ```bash
    cd qhull/build
    ```
    ```bash
    cmake ..
    ```
    ```bash
    ccmake .. -> CMAKE_BUILD_TYPE = Debug;Release
    ```
    ```bash
    cmake ..
    ```
    ```bash
    make -j10 # 10 jobs, kann angepasst werden
    ```
    ```bash
    sudo make install   
    ```
##Librealsense
    ```bash
    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
    ```
    Ubuntu 20: 
    ```bash
    sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo focal main" -u
    ```
    Installieren
    ```bash
    sudo apt-get install librealsense2-dkms
    ```
    ```bash
    sudo apt-get install librealsense2-utils
    ```
    Developer Tools
    ```bash
    sudo apt-get install librealsense2-dev
    ```
    ```bash
    sudo apt-get install librealsense2-dbg
    ```
    How to run
    ```bash
    realsense-viewer
    ```