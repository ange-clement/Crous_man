#ifndef SHADER_OBJECT_HPP
#define SHADER_OBJECT_HPP

class Shader {
public:
    GLuint programID;

public:
    Shader(std::string vs, std::string fs);
    ~Shader();

    void use();
};

#endif