#ifndef TEXTURE_FRAMEBUFFER_HPP
#define G_BUFFER_HPP


struct TextureFramebuffer {
    unsigned int framebuffer;
    unsigned int textureColorBuffer;
    unsigned int rboDepth;

    TextureFramebuffer();
    TextureFramebuffer(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void update(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void init(unsigned int SCR_WIDTH, unsigned int SCR_HEIGHT);

    void use();
};

#endif