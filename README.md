# Crous_man
#### Crous_man game : the movie : the game
As part of our studies in Master Imagine in Montpellier, we had to set up a game engine in C++.
Afterwards, we developed an interactive application to present the scope of our engine: .
For this, we were inspired by the famous roundabout in Montpellier.

![roundabout](./presentations/famous_roundabout.jpg)

#### Speech of the game
You play as a giant monster in an urban environment that is more realistic than ever, and your only goal is to destroy it. It's a bit like if Godzilla was taking place in Montpellier...

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

## Documentations :

- Work repport :
[<img align="center" src="./presentations/french.png" width="50" height="50"/>](./presentations/GameEngine-ReindersErwanAngeClement.pdf)

- Work presentation :
[<img align="center" src="./presentations/french.png" width="50" height="50"/>](./presentations/GameEngine-Presentation-ReindersErwanAngeClement.pdf)
