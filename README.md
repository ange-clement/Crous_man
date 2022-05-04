# Crous_man
Crous_man game : the movie : the game

## Build and Launch : 
Il faut copier les fichiers de 'copyToBuild' dans le dossier build.
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

Install visual studio 2022 (or 2019) with C++ extention (for cmake)  
Clone the repos using the IDE  
When loading, it will create a out/build/x64-Debug folder  
Copy the files from 'copyToBuild' to that folder  
Build the project (ctrl+shift+b)  
Go to the build directory and launch 'launch-Crous_man.cmd'  
