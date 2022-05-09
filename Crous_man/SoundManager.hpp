#ifndef SOUND_MANAGER_HPP
#define SOUND_MANAGER_HPP

#define DEBUG_AUDIO false

#include <irrKlang.h>

class Entity;

struct PlayingAudio {
    irrklang::ISound* sound3D;
    Entity* attachedEntity;
};

class SoundManager {
public:
    static SoundManager* instance;

    irrklang::ISoundEngine* soundEngine;

    std::vector<PlayingAudio> audios;

    Entity* audioListener = NULL;
    irrklang::vec3df previousAudioListenerPos;

    float defaultMinDistance = 50.0f;

public:
    SoundManager();
    ~SoundManager();

    void update();

    void play(std::string soundFile);
    void playLoop(std::string soundFile);
    unsigned int playAt(std::string soundFile, glm::vec3 pos);
    unsigned int playAt(std::string soundFile, glm::vec3 pos, float minDistance);
    unsigned int playOver(std::string soundFile, Entity* entity);
    unsigned int playOver(std::string soundFile, Entity* entity, float minDistance);

    void setAudioListener(Entity* audioListener);

    irrklang::vec3df getAudioListenerVelocity();

    unsigned int getLowerNonOccupied();
};

irrklang::vec3df glmVec3ToIrrklangVec3(glm::vec3 vec);

irrklang::vec3df entityPosToIrrklangVec3(Entity* e);
irrklang::vec3df entityForwardToIrrklangVec3(Entity* e);

#endif