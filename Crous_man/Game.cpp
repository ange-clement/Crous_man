// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Include image loading
#include "../external/image_ppm.h"
//#define STB_IMAGE_IMPLEMENTATION
//#include "../external/stb_image.h"  

// Include my stuff
#include "Util.hpp"

#include "Mesh.hpp"
#include "Scene.hpp"
#include "InputManager.hpp"
#include "Scenes/ECS_test.hpp"

#include "Shaders/Shader.hpp"
#include "ECS/EntityManager.hpp"


#include <common/shader.hpp>
#include <common/objloader.hpp>
#include <common/vboindexer.hpp>

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

// settings
unsigned int SCR_WIDTH = 1024;
unsigned int SCR_HEIGHT = 768;

/*******************************************************************************/

int main( void )
{
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( SCR_WIDTH, SCR_HEIGHT, "Crous man 2 ! With 3D !", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    // Hide the mouse and enable unlimited mouvement
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    //glfwSetCursorPosCallback(window, mouse_callback);  
    glfwSetScrollCallback(window, scroll_callback);

    glfwSetWindowSizeCallback(window, framebuffer_size_callback);

    // Set the mouse at the center of the screen
    glfwPollEvents();
    glfwSetCursorPos(window, SCR_WIDTH/2, SCR_WIDTH/2);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);
    
    new EntityManager();

    createSceneECS();

    EntityManager::instance->initializeAllSystems();

    do{
        InputManager::instance->update(window);

        EntityManager::instance->updateAllSystems();
        EntityManager::instance->updateTransforms();
        //TODO DRAW AFTER UPDATE TRANSFORM !

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
        //glfwSetWindowShouldClose(window, true);

    }
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    delete EntityManager::instance;

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}



void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    InputManager::instance->scroll_callback(window, xoffset, yoffset);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    InputManager::instance->framebuffer_size_callback(window, width, height);
}