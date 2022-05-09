#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Util.hpp"

#include "Transform.hpp"


Rotation::Rotation() {
    rotationMatrix = glm::mat3(1.0);
}
Rotation::Rotation(glm::mat3 otherRotationMatrix) {
    rotationMatrix = glm::mat3(otherRotationMatrix);
}

glm::vec3 Rotation::rotate(glm::vec3 p) {
    return rotationMatrix*p;
}


void Rotation::setRotation(float angle, glm::vec3 axis) {
    glm::mat4 ID = glm::mat4(1.0);
    glm::mat4 result = glm::rotate(ID, angle, axis);
    rotationMatrix = getFromMat4(result);
}

void Rotation::combineRotation(float angle, glm::vec3 axis) {
    glm::mat4 rotMat4 = glm::mat4(rotationMatrix);
    glm::mat4 result = glm::rotate(rotMat4, angle, axis);
    rotationMatrix = getFromMat4(result);
}

Rotation Rotation::combineRotation(Rotation r) {
    Rotation newRotation = Rotation();
    newRotation.rotationMatrix = rotationMatrix * r.rotationMatrix;
    return newRotation;
}

Rotation Rotation::inverse() {
    Rotation newRotation = Rotation();
    newRotation.rotationMatrix = glm::transpose(rotationMatrix);
    return newRotation;
}

Rotation Rotation::mixWith(Rotation r, float k) {
    Rotation newRotation = Rotation();
    newRotation.rotationMatrix = ((1-k)*rotationMatrix) + (k*r.rotationMatrix);
    return newRotation;
}

glm::mat4 Rotation::toMat4() {
    glm::mat4 mat = glm::mat4(rotationMatrix);
    return mat;
}

void Rotation::lookAt(glm::vec3 position, glm::vec3 target, glm::vec3 up) {
    glm::mat4 lookAtMatrice = glm::lookAt(
        position,
        target,
        up
    );
    rotationMatrix = getFromMat4Transposed(lookAtMatrice);
}


Transform::Transform() {
    translation = glm::vec3(0.0, 0.0, 0.0);
    scaling = glm::vec3(1.0, 1.0, 1.0);
    rotation = Rotation();
}

Transform::Transform(Transform* other) {
    this->copy(other);
}

void Transform::copy(Transform* other) {
    translation = glm::vec3(other->translation);
    scaling = glm::vec3(other->scaling);
    rotation = Rotation(other->rotation.rotationMatrix);
}

glm::vec3 Transform::applyToPoint(glm::vec3 p) {
    return rotation.rotate(this->scaling * glm::vec3(p)) + translation;
}

glm::vec3 Transform::applyToVector(glm::vec3 v) {
    return rotation.rotate(this->scaling * glm::vec3(v));
}
glm::vec3 Transform::applyToVersor(glm::vec3 n) {
    return rotation.rotate(glm::vec3(n));
}

Transform* Transform::combineWith(const Transform* t) {
    Transform* newTransform = new Transform();
    newTransform->translation = applyToPoint(t->translation);
    newTransform->scaling = scaling * t->scaling;
    newTransform->rotation = rotation.combineRotation(t->rotation);
    return newTransform;
}

Transform* Transform::inverse() {
    Transform* newTransform = new Transform();
    newTransform->translation = -translation;
    newTransform->scaling = glm::vec3(1.0/scaling[0], 1.0/scaling[1], 1.0/scaling[2]);
    newTransform->rotation = rotation.inverse();
    return newTransform;
}

Transform* Transform::mixWith(const Transform* t, float k) {
    Transform* newTransform = new Transform();
    newTransform->translation = translation + (t->translation - translation) * k;
    newTransform->scaling = scaling + (t->scaling - scaling) * k;
    newTransform->rotation = rotation.mixWith(t->rotation, k);
    return newTransform;
}

glm::mat4 Transform::toMat4() {
    glm::mat4 mat = rotation.toMat4();
    mat[3][0] = translation[0];
    mat[3][1] = translation[1];
    mat[3][2] = translation[2];

    mat[0] *= scaling[0];
    mat[1] *= scaling[1];
    mat[2] *= scaling[2];
    return mat;
}

glm::mat4 Transform::toMat4NoScaling() {
    glm::mat4 mat = rotation.toMat4();
    mat[3][0] = translation[0];
    mat[3][1] = translation[1];
    mat[3][2] = translation[2];
    return mat;
}

glm::mat4 Transform::toMat4NoScalingNoRotation() {
    glm::mat4 mat;
    mat[3][0] = translation[0];
    mat[3][1] = translation[1];
    mat[3][2] = translation[2];
    return mat;
}

glm::mat4 Transform::toNormal() {
    return glm::transpose(this->inverse()->toMat4());
    //return glm::transpose(rotation.toMat4());
}

glm::vec3 Transform::getRight() {
    return applyToVersor(glm::vec3(1.0, 0.0, 0.0));
}

glm::vec3 Transform::getUp() {
    return applyToVersor(glm::vec3(0.0, 1.0, 0.0));
}

glm::vec3 Transform::getForward() {
    return applyToVersor(glm::vec3(0.0, 0.0, 1.0));
}

glm::vec3 Transform::worldToLocal(glm::vec3 point) {
    return this->inverse()->applyToPoint(point);
}



void Transform::setLocalPosition(glm::vec3 position) {
    this->translation = position;
}

void Transform::translate(glm::vec3 amount) {
    this->translation += amount;
}

void Transform::lookAt(const Transform* other) {
    glm::vec3 up = glm::cross(getRight(), other->translation - translation);
    rotation.lookAt(translation, other->translation, up);
}

void Transform::lookAt(glm::vec3 target) {
    glm::vec3 up = glm::cross(getRight(), target - translation);
    rotation.lookAt(translation, target, up);
}

void Transform::lookAtDirection(glm::vec3 forwardVector) {
    glm::vec3 up = glm::cross(getRight(), forwardVector);
    glm::vec3 target = translation + forwardVector;
    rotation.lookAt(translation, target, up);
}