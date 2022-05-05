#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Util.hpp"

glm::mat3 getFromMat4(glm::mat4 mat) {
    //return glm::mat3(mat[0][0], mat[1][0], mat[2][0], mat[0][1], mat[1][1], mat[2][1], mat[0][2], mat[1][2], mat[2][2]);
    return glm::mat3(mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]);
}

void print(float v) {
    std::cout << v << std::endl;
}

void print(glm::vec2 v) {
    std::cout << v[0] << " "<< v[1] << std::endl;
}

void print(glm::vec3 v) {
    std::cout << v[0] << " "<< v[1] << " " << v[2] << std::endl;
}

void print(glm::vec4 v) {
    std::cout << v[0] << " "<< v[1] << " " << v[2] << " " << v[3] << std::endl;
}

void print(glm::mat4 m) {
    print(m[0]);
    print(m[1]);
    print(m[2]);
    print(m[3]);
}

void print(glm::mat3 m) {
    print(m[0]);
    print(m[1]);
    print(m[2]);
}

void minVec3(glm::vec3 v1, glm::vec3 v2, glm::vec3& res) {
    res = glm::vec3(
        ((v1.x < v2.x) ? v1.x : v2.x),
        ((v1.y < v2.y) ? v1.y : v2.y),
        ((v1.z < v2.z) ? v1.z : v2.z)
    );
}

void maxVec3(glm::vec3 v1, glm::vec3 v2, glm::vec3& res) {
    res = glm::vec3(
        ((v1.x > v2.x) ? v1.x : v2.x),
        ((v1.y > v2.y) ? v1.y : v2.y),
        ((v1.z > v2.z) ? v1.z : v2.z)
    );
}

bool compareWithEpsilon(float f1, float f2) {
    return (std::abs(f1 - f2) <= FLT_EPSILON * std::max(1.0f, std::max(std::abs(f1), std::abs(f2))));
}

glm::vec2 Project(const glm::vec2& length, const glm::vec2& direction) {
    float dot = glm::dot(length, direction);
    float magSq = glm::dot(direction,direction);
    return direction * (dot / magSq);
}

glm::vec3 Project(const glm::vec3& length, const glm::vec3& direction) {
    float dot = glm::dot(length, direction);
    float magSq = glm::dot(direction,direction);
    return direction * (dot / magSq);
}


glm::vec3 multiplyVector(const glm::vec3& vec, const glm::mat4& mat) {
    glm::vec3 result;
    result.x = vec.x * mat[0].x + vec.y * mat[0].y + vec.z * mat[0].z;
    result.y = vec.x * mat[1].x + vec.y * mat[1].y + vec.z * mat[1].z;
    result.z = vec.x * mat[2].x + vec.y * mat[2].y + vec.z * mat[2].z;
    return result;
}

glm::vec3 projectV3OnM3(const glm::vec3& vec, const glm::mat3& mat) {
    glm::vec3 result;
    result.x = glm::dot(vec, mat[0]);
    result.y = glm::dot(vec, mat[1]);
    result.z = glm::dot(vec, mat[2]);
    return result;
}