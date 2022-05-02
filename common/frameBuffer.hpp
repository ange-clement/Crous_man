#ifndef FRAMEBUFFER_BUFFER_HPP
#define FRAMEBUFFER_BUFFER_HPP


struct FrameBuffer {
    unsigned int frameBufferObject;
    unsigned int* buffers;
    unsigned int numberOfBuffer;
    unsigned int rboDepth;

    FrameBuffer();
    FrameBuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT, unsigned int numberOfBuffer);

    void update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);
};

#endif