#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <irrKlang.h>

#include "ECS/EntityManager.hpp"
#include "Components/Camera.hpp"

#include "SoundManager.hpp"

SoundManager* SoundManager::instance = NULL;

SoundManager::SoundManager() {
    if (SoundManager::instance == NULL) {
        SoundManager::instance = this;

        soundEngine = irrklang::createIrrKlangDevice();
    }
    else {
        std::cerr << "Error : cannot instanciate two InputManager" << std::endl;
        exit(1);
    }
}

SoundManager::~SoundManager() {
    soundEngine->drop();
}

void SoundManager::play(std::string soundFile) {
    soundEngine->play2D(soundFile.c_str(), false);
}