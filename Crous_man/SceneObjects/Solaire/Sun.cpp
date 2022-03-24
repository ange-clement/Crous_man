#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <glm/gtc/matrix_transform.hpp>

#include "Sun.hpp"

#include "Planet.hpp"


Sun::Sun() : Planet("../ressources/sun.ppm", "shaders/sun_vertex_shader.glsl", "shaders/sun_fragment_shader.glsl") {

}