#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include <irrKlang.h>

#include "Transform.hpp"
#include "ECS/EntityManager.hpp"
#include "Components/Camera.hpp"
#include "InputManager.hpp"

#include "SoundManager.hpp"
#include "ECS/Entity.hpp"


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

void SoundManager::update() {
    if (audioListener != NULL) {
        soundEngine->setListenerPosition(entityPosToIrrklangVec3(audioListener), entityForwardToIrrklangVec3(audioListener), getAudioListenerVelocity());
        this->previousAudioListenerPos = entityPosToIrrklangVec3(audioListener);
    }
    else {
        std::cout << "Warning : no audio listener has been set!" << std::endl;
    }
    if (DEBUG_AUDIO && audios.size() > 0.0f) {
        std::cout << audios.size() << ": ";
    }
    auto it = audios.begin();
    while (it != audios.end()) {
        if (it->sound3D == NULL) {
            if (DEBUG_AUDIO)
                std::cout << "| ";
        }
        else if (!it->sound3D->isLooped() && it->sound3D->isFinished()) {
            if (DEBUG_AUDIO)
                std::cout << "X ";
            it->sound3D->drop();
            it->sound3D = NULL;
        }
        else {
            if (DEBUG_AUDIO)
                std::cout << (it->attachedEntity != NULL) << " ";
            if (it->attachedEntity != NULL) {
                it->sound3D->setPosition(entityPosToIrrklangVec3(it->attachedEntity));
            }
        }
        if ((it+1) >= audios.end() && it->sound3D == NULL) {
            it = audios.erase(it);
        }
        else {
            it++;
        }
    }

    if (DEBUG_AUDIO && audios.size() > 0.0f) {
        std::cout << "fin" << std::endl;
    }
}

void SoundManager::play(std::string soundFile) {
    soundEngine->play2D(soundFile.c_str(), false);
}
void SoundManager::playLoop(std::string soundFile) {
    soundEngine->play2D(soundFile.c_str(), true);
}

unsigned int SoundManager::playAt(std::string soundFile, glm::vec3 pos) {
    return playAt(soundFile, pos, defaultMinDistance);
}

unsigned int SoundManager::playAt(std::string soundFile, glm::vec3 pos, float minDistance) {
    irrklang::ISound* sound3D = soundEngine->play3D(soundFile.c_str(), glmVec3ToIrrklangVec3(pos), false, false, true);

    if (sound3D) {
        sound3D->setMinDistance(minDistance);
    }
    
    unsigned int i = getLowerNonOccupied() ;
    audios[i].sound3D = sound3D;
    audios[i].attachedEntity = NULL;

    return i;
}

unsigned int SoundManager::playOver(std::string soundFile, Entity* entity) {
   return playOver(soundFile, entity, defaultMinDistance);
}

unsigned int SoundManager::playOver(std::string soundFile, Entity* entity, float minDistance) {
    irrklang::ISound* sound3D = soundEngine->play3D(soundFile.c_str(), glmVec3ToIrrklangVec3(entity->worldTransform->translation), false, false, true);

    if (sound3D) {
        sound3D->setMinDistance(minDistance);
    }

    unsigned int i = getLowerNonOccupied() ;
    audios[i].sound3D = sound3D;
    audios[i].attachedEntity = entity;

    return i;
}

void SoundManager::setAudioListener(Entity* audioListener) {
    this->audioListener = audioListener;
    this->previousAudioListenerPos = entityPosToIrrklangVec3(audioListener);
}

irrklang::vec3df SoundManager::getAudioListenerVelocity() {
    //(1.0 / InputManager::instance->deltaTime) * 
    return (entityPosToIrrklangVec3(audioListener) - previousAudioListenerPos);
}

unsigned int SoundManager::getLowerNonOccupied() {
    for (unsigned int i = 0, size = audios.size(); i < size; i++) {
        if (audios[i].sound3D == NULL) {
            return i;
        }
    }
    audios.push_back(PlayingAudio());
    return audios.size()-1;
}



irrklang::vec3df glmVec3ToIrrklangVec3(glm::vec3 vec) {
    return irrklang::vec3df(vec[0], vec[1], vec[2]);
}

irrklang::vec3df entityPosToIrrklangVec3(Entity* e) {
    return glmVec3ToIrrklangVec3(e->worldTransform->translation);
}

irrklang::vec3df entityForwardToIrrklangVec3(Entity* e) {
    return glmVec3ToIrrklangVec3(-e->worldTransform->getForward());
}
