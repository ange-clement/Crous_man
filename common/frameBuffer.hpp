#ifndef FRAMEBUFFER_BUFFER_HPP
#define FRAMEBUFFER_BUFFER_HPP


class FrameBuffer {
public:
    unsigned int frameBufferObject;
    unsigned int* buffers;
    unsigned int numberOfBuffer;
    unsigned int rboDepth;

    unsigned int height, width;
public:
    FrameBuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, unsigned int numberOfBuffer);
    ~FrameBuffer();

    void update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void use();
};

#endif