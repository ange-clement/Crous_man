#ifndef SOUND_MANAGER_HPP
#define SOUND_MANAGER_HPP

#include <irrKlang.h>

class SoundManager {
public:
    static SoundManager* instance;

    irrklang::ISoundEngine* soundEngine;

public:
    SoundManager();
    ~SoundManager();

    void play(std::string soundFile);
};

#endif