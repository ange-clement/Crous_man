#ifndef BASIC_SHAPE_RENDER_HPP
#define BASIC_SHAPE_RENDER_HPP

class BasicShapeRender {
public:
    static BasicShapeRender* instance;

    unsigned int quadVAO = 0;
    unsigned int quadVBO;
public:
    BasicShapeRender();
    void renderQuad();
};

#endif