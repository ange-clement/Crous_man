#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <initializer_list>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Bitmap.hpp"

#include "EntityManager.hpp"


Bitmap::Bitmap() : Bitmap(0) {

}
Bitmap::~Bitmap() {

}

Bitmap::Bitmap(unsigned int bitmap) {
    this->bitmap = bitmap;
}

Bitmap::Bitmap(std::initializer_list<SystemIDs> systems) {
    loadFromSystemIDS(systems);
}

void Bitmap::loadFromSystemIDS(std::initializer_list<SystemIDs> systems) {
    this->bitmap = 0;
    for (SystemIDs system : systems) {
        this->addId(system);
    }
}

void Bitmap::addId(SystemIDs other) {
    this->bitmap |= 1 << other;
}
void Bitmap::removeId(SystemIDs other) {
    this->bitmap ^= 1 << other;
}
void Bitmap::addBitmap(const Bitmap* other) {
    this->bitmap |= other->bitmap;
}
void Bitmap::removeBitmap(const Bitmap* other) {
    this->bitmap ^= other->bitmap;
}

Bitmap* Bitmap::combine(const Bitmap* other) {
    return new Bitmap(this->bitmap & other->bitmap);
}


bool Bitmap::equals(const Bitmap* other) {
    return this->bitmap == other->bitmap;
}