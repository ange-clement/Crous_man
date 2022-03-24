#ifndef INPUT_MANAGER_HPP
#define INPUT_MANAGER_HPP

class InputManager {
public:

    float interpolation = 0.0;

    unsigned int SCR_WIDTH = 1024;
    unsigned int SCR_HEIGHT = 768;

    float scroll_distance;

    bool firstMouseMouve;
    float lastMouseX, lastMouseY;
    float mouseOffsetX, mouseOffsetY;

public:
    InputManager();
    
    void processInput(GLFWwindow *window);


    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    void framebuffer_size_callback(GLFWwindow* window, int width, int height);
};

#endif