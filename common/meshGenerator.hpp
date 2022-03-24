#ifndef MESH_GENERATOR_HPP
#define MESH_GENERATOR_HPP

void uvSphere(  std::vector<glm::vec3> & vertices,
                std::vector<glm::vec3> & normals,
                std::vector<glm::vec2> & texCoords,
                std::vector<unsigned short> & indices,
                std::vector<std::vector<unsigned short>> & triangles,
                float radius = 0.5, unsigned int slices = 32, unsigned int stacks = 16
                );
#endif