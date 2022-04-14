#ifndef TEXTURE_HPP
#define TEXTURE_HPP

GLuint loadTextureFromColor(glm::vec3 color);
GLuint loadTextureFromFloat(float value);
GLuint loadTextureFromPGM(const char* textureName);
GLuint loadTextureFromPPM(const char* textureName);


// Load a .BMP file using our custom loader
GLuint loadTextureFromBMP_custom(const char * imagepath);

//// Since GLFW 3, glfwLoadTexture2D() has been removed. You have to use another texture loading library, 
//// or do it yourself (just like loadBMP_custom and loadDDS)
//// Load a .TGA file using GLFW's own loader
//GLuint loadTGA_glfw(const char * imagepath);

// Load a .DDS file using GLFW's own loader
GLuint loadTextureFromDDS(const char * imagepath);


#endif