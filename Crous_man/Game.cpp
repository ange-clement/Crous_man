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
#include "SceneObjects/Camera.hpp"
#include "SceneObjects/PointLight.hpp"
#include "Scenes/Object_control.hpp"

#include "ECS/EntityManager.hpp"

#include "SceneObjects/Terrain.hpp"
#include "SceneObjects/LODObject.hpp"
#include "Transform.hpp"


#include <common/shader.hpp>
#include <common/objloader.hpp>
#include <common/vboindexer.hpp>

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);

// settings
unsigned int SCR_WIDTH = 1024;
unsigned int SCR_HEIGHT = 768;

// model
SceneObject* scene;
Camera* camera;

std::vector<PointLight*> pointLights;
InputManager* inputManager;

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
    glClearColor(0.8f, 0.8f, 0.8f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    //glEnable(GL_CULL_FACE);
    
    new EntityManager();


    scene = createSceneObject(camera, pointLights);

    Terrain* terrain = (Terrain*) scene->childrens[0];
    SceneObject* player = scene->childrens[1];
    SceneObject* cameraVerticalContainer = scene->childrens[2];
    LODObject* lod = (LODObject*) scene->childrens[3];
    float l;
    inputManager = new InputManager();

    // For speed computation
    double lastTime = glfwGetTime();
    float deltaTime;
    float lastFrame;
    int nbFrames = 0;

    EntityManager::instance->initializeAllSystems();

    do{
        // Measure speed
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Inputs
        inputManager->processInput(window);
        scene->processInput(window, inputManager, deltaTime);

        EntityManager::instance->updateAllSystems();
        EntityManager::instance->updateTransforms();

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        scene->updateTransforms();
        
        scene->updateLights(pointLights);
        camera->updateMVP();
        scene->draw(camera->view, camera->projection);


        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();
        //glfwSetWindowShouldClose(window, true);

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    delete scene;
    delete camera;
    delete EntityManager::instance;

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}



void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    inputManager->scroll_callback(window, xoffset, yoffset);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    inputManager->framebuffer_size_callback(window, width, height);
}