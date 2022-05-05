#ifndef FRAMEBUFFER_BUFFER_HPP
#define FRAMEBUFFER_BUFFER_HPP


class FrameBuffer {
public:
    GLuint  frameBufferObject;
    GLuint* buffers;
    unsigned int numberOfBuffer;
    unsigned int rboDepth;

    unsigned int height, width;

    GLint interpolationType;
public:
    FrameBuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, unsigned int numberOfBuffer);
    FrameBuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, unsigned int numberOfBuffer, GLint interpolationType);
    ~FrameBuffer();

    void update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, GLint interpolationType);

    void use();
};

#endif