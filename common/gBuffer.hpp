#ifndef G_BUFFER_HPP
#define G_BUFFER_HPP


struct GBuffer {
    unsigned int gBuffer;
    unsigned int gPosition, gNormal, gAlbedoSpec;
    unsigned int rboDepth;

    GBuffer();
    GBuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);
};

#endif