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