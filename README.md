# Installation
## Visual Studio
CMake Tools, sollte automatisch von Visual Studio installiert werden
## CMake
```bash
sudo apt-get install cmake
sudo apt-get install cmake.qt-gui
sudo apt-get install cmake-curses-gui
```
## GCC
```bash
sudo apt-get install gcc
sudo apt-get install g++
```
## PCL
### Abhänigkeiten
https://packages.ubuntu.com/xenial/libpcl-dev
```bash
sudo apt-get install libboost-all-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libflann-dev
sudo apt-get install libopenni-dev 
sudo apt-get install libqhull-dev
sudo apt-get install libvtk6-dev
sudo apt-get install libvtk6-qt-dev
```
### PCL selbst
- Download link https://github.com/PointCloudLibrary/pcl/releases (Verwendet wurde 1.10.1)
```
cd pcl-pcl-1.10.1 # version anpassen
```
```
mkdir build && cd build
```
```
cmake ..
```
Optinal: bei error: The CXX compiler identification is unknown.
Befehl im pcl Verzeichnis und nicht im build Ordner ausführen.
```bash
cmake -DCMAKE_CXX_COMPILER=/usr/bin/c++ 
```
Diese Befehle im build Ornder ausführen.
```
ccmake ..
```
WITH_QHULL -> deaktiveren, da es sonst zu einem Konflikt mit einer anderen Library von QHULL kommt. Mit die Taste 'c' wird die erneute Konfiguration gestartet. Anschließend beendet man ccmake mit 'q'.
```
make -j(Anzahl gewünschter Prozessoren)
```
```
sudo make install
```
## QHULL
https://github.com/qhull/qhull
```
cd qhull/build
```
```
cmake ..
```
```
ccmake ..
```
Einstellung setzen -> CMAKE_BUILD_TYPE = Debug;Release
```
cmake ..
```
```
make -j10 # 10 jobs, kann angepasst werden
```
```
sudo make install   
```
## Librealsense
Anleitung https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
Ubuntu 20: 
```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo focal main" -u
```
Installieren
```
sudo apt-get install librealsense2-dkms
```
```
sudo apt-get install librealsense2-utils
```
Developer Tools
```
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
Starten
```
realsense-viewer
```

## std-imager
ermöglicht das Lesen und Schreiben von Bildern
Download https://github.com/nothings/stb und Pfade anpassen

## pugi-xml
ermöglicht das Lesen von XML-Dateien
https://github.com/zeux/pugixml

## CImg
Für Morphones Snakes
Downlaod https://github.com/dtschump/CImg.git und Pfade anpassen