#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Moon.hpp"

#include "Planet.hpp"


Moon::Moon() : Planet("../ressources/moon.ppm", "shaders/planet_vertex_shader.glsl", "shaders/planet_fragment_shader.glsl") {

}