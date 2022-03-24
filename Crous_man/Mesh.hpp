#ifndef MODEL_HPP
#define MODEL_HPP

class PointLight;
struct Transform;

class Texture {
public:
    unsigned int texture;
    
public:
    Texture();

    unsigned int loadTextureGrey(const char* textureName);
    
    unsigned int loadTexture(const char* textureName);
};

class Mesh {
public:
    GLuint programID;

    GLuint VertexArrayID;

    GLuint vertexbuffer;
    GLuint normalsbuffer;
    GLuint elementbuffer;

    GLuint uModelMatrixID;
    GLuint uViewMatrixID;
    GLuint uProjectionMatrixID;
    GLuint uNormalMatrixID;

    GLuint numLightsID;
    GLuint lightID;

    std::vector<unsigned short> indices;
    std::vector<std::vector<unsigned short>> triangles;
    std::vector<glm::vec3> indexed_vertices;

    std::vector<glm::vec3> triangle_normals;
    std::vector<glm::vec3> normals;
    
    bool shaderLoaded;

public:
    Mesh();

    ~Mesh();

    void loadShaders(std::string vertex_shader, std::string fragment_shader);

    void loadShaders();

    void loadMesh(std::string filename);
    void loadMesh(std::string filename, bool fileHasNormals);

    void loadBuffers();

    void computeTrianglesNormals();

    void computeSmoothVertexNormal();

    void computeNormals();

    void useProgram();

    void updateLights(const std::vector<PointLight*> & pointLights);

    void draw(Transform* transform, glm::mat4 view, glm::mat4 projection);
};


#endif