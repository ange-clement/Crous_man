#ifndef UTIL_HPP
#define UTIL_HPP

glm::mat3 getFromMat4(glm::mat4 mat);

void print(float v);

void print(glm::vec2 v);

void print(glm::vec3 v);

void print(glm::vec4 v);

void print(glm::mat4 m);

void print(glm::mat3 m);

//Return min between v1 and v2 
void minVec3(glm::vec3 v1, glm::vec3 v2, glm::vec3& res);

//Return max between v1 and v2 
void maxVec3(glm::vec3 v1, glm::vec3 v2, glm::vec3& res);

//Compare two float with epsilon bias
bool compareWithEpsilon(float f1, float f2);


glm::vec2 Project(const glm::vec2& length, const glm::vec2& direction);

glm::vec3 Project(const glm::vec3& length, const glm::vec3& direction);

glm::vec3 multiplyVector(const glm::vec3& vec, const glm::mat4& mat);

glm::vec3 projectV3OnM3(const glm::vec3& vec, const glm::mat3& mat);
#endif