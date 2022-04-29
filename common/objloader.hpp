#ifndef OBJLOADER_H
#define OBJLOADER_H
#include <string>

bool loadOBJ(
	const char * path, 
	std::vector<glm::vec3> & out_vertices, 
	std::vector<glm::vec2> & out_uvs, 
	std::vector<glm::vec3> & out_normals
);

bool loadPLY(
    std::string const& filename,
    std::vector< glm::vec3 >& o_vertices,
    std::vector< glm::vec3 >& o_normals,
    std::vector<glm::vec2>& o_UV,
    std::vector< unsigned short >& indices,
    std::vector< std::vector<unsigned short> >& o_triangles,
    bool invertTriangles
);



bool loadAssImp(
	const char * path, 
	std::vector<unsigned short> & indices,
	std::vector<glm::vec3> & vertices,
	std::vector<glm::vec2> & uvs,
	std::vector<glm::vec3> & normals
);


bool loadOFF( const std::string & filename ,
              std::vector< glm::vec3 > & vertices ,
              std::vector< unsigned short > & faces) ;


bool loadOFF( const std::string & filename ,
              std::vector< glm::vec3 > & vertices ,
              std::vector< unsigned short > & indices,
              std::vector< std::vector<unsigned short > > & triangles) ;

void openOFF(   std::string const & filename,
                std::vector< glm::vec3 > & o_vertices,
                std::vector< glm::vec3 > & o_normals,
                std::vector< unsigned short > & indices,
                std::vector< std::vector<unsigned short> > & o_triangles,
                bool load_normals = true );
#endif
