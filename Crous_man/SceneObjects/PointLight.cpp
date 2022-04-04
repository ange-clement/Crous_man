#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "PointLight.hpp"

#include "../Transform.hpp"
#include "../Mesh.hpp"

PointLight::PointLight() : SceneObject() {
        
}