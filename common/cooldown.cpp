#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>
#include <Crous_man/InputManager.hpp>

#include "cooldown.hpp"

Cooldown::Cooldown(float cooldownTime) {
	this->cooldownTime = cooldownTime;
	this->startTime = 0.0f;
	this->endTime = 0.0f;
}

void Cooldown::start() {
	this->startTime = InputManager::instance->lastFrame;
	this->endTime = this->startTime + this->cooldownTime;
}

bool Cooldown::inCooldown() {
	return InputManager::instance->lastFrame < this->endTime;
}