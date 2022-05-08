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


PlayingAudio::PlayingAudio(irrklang::ISound* sound3D, Entity* attachedEntity) : sound3D(sound3D), attachedEntity(attachedEntity) {

}



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
    auto it = audios.begin();
    while (it != audios.end()) {
        if (!it->sound3D->isLooped() && it->sound3D->isFinished()) {
            it->sound3D->drop();
            it = audios.erase(it);
        }
        else {
            if (it->attachedEntity != NULL) {
                it->sound3D->setPosition(entityPosToIrrklangVec3(it->attachedEntity));
            }
            it++;
        }
    }
}

void SoundManager::play(std::string soundFile) {
    soundEngine->play2D(soundFile.c_str(), false);
}

void SoundManager::playAt(std::string soundFile, glm::vec3 pos) {
    playAt(soundFile, pos, defaultMinDistance);
}

void SoundManager::playAt(std::string soundFile, glm::vec3 pos, float minDistance) {
    irrklang::ISound* sound3D = soundEngine->play3D(soundFile.c_str(), glmVec3ToIrrklangVec3(pos), false, false, true);

    if (sound3D) {
        sound3D->setMinDistance(5.0f);
    }

    audios.push_back(PlayingAudio(sound3D, NULL));
}

void SoundManager::playOver(std::string soundFile, Entity* entity) {
    playOver(soundFile, entity, defaultMinDistance);
}

void SoundManager::playOver(std::string soundFile, Entity* entity, float minDistance) {
    irrklang::ISound* sound3D = soundEngine->play3D(soundFile.c_str(), glmVec3ToIrrklangVec3(entity->worldTransform->translation), false, false, true);

    if (sound3D) {
        sound3D->setMinDistance(5.0f);
    }

    audios.push_back(PlayingAudio(sound3D, entity));
}

void SoundManager::setAudioListener(Entity* audioListener) {
    this->audioListener = audioListener;
    this->previousAudioListenerPos = entityPosToIrrklangVec3(audioListener);
}

irrklang::vec3df SoundManager::getAudioListenerVelocity() {
    //(1.0 / InputManager::instance->deltaTime) * 
    return (entityPosToIrrklangVec3(audioListener) - previousAudioListenerPos);
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
