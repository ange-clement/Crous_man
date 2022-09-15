# Crous_man
Crous_man game : the movie : the game

## Build and Launch : 
You need to copy the files from `copyToBuild` to the build folder.
### Linux :
#### build
```
mkdir build
cd build
cmake ..
make -j
```
#### launch
```
cd build
./launch-Crous_man.sh
```
### Windows :

Install `visual studio 2022` (or 2019) with C++ extention (for cmake)  
Clone the repos using the IDE  
When loading, it will create a out/build/x64-Debug folder (if not, got into the CMakeLists.txt and save : ctrl+s)  
Copy the files from `copyToBuild` to that folder  
Build the project (ctrl+shift+b)  
Go to the build directory and launch `launch-Crous_man.cmd`  
