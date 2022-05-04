# CMake entry point
cmake_minimum_required (VERSION 3.0)
project (Game)

find_package(OpenGL REQUIRED)


if( CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR )
    message( FATAL_ERROR "Please select another Build Directory ! (and give it a clever name, like bin_Visual2012_64bits/)" )
endif()
if( CMAKE_SOURCE_DIR MATCHES " " )
	message( "Your Source Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()
if( CMAKE_BINARY_DIR MATCHES " " )
	message( "Your Build Directory contains spaces. If you experience problems when compiling, this can be the cause." )
endif()



# Compile external dependencies 
add_subdirectory (external)

# On Visual 2005 and above, this module can set the debug working directory
cmake_policy(SET CMP0026 OLD)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-fe2273")
include(CreateLaunchers)
include(MSVCMultipleProcessCompile) # /MP

if(INCLUDE_DISTRIB)
	add_subdirectory(distrib)
endif(INCLUDE_DISTRIB)



include_directories(
	external/glfw-3.1.2/include/
	external/glm-0.9.7.1/
	external/glew-1.13.0/include/
	external/irrKlang-64bit-1.6.0/include/
	.
)

set(ALL_LIBS
	${OPENGL_LIBRARY}
	glfw
	GLEW_1130
	irrKlang
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

# Crous_man
add_executable(
	Crous_man
    Crous_man/Game.cpp

	common/shader.cpp
	common/shader.hpp
	common/controls.cpp
	common/controls.hpp
	common/texture.cpp
	common/texture.hpp
	common/objloader.cpp
	common/objloader.hpp
	common/vboindexer.cpp
	common/vboindexer.hpp
	common/meshGenerator.cpp
	common/meshGenerator.hpp
	common/basicShapeRender.cpp
	common/basicShapeRender.hpp
	common/frameBuffer.cpp
	common/frameBuffer.hpp
	common/gBuffer.cpp
	common/gBuffer.hpp
	common/textureFramebuffer.cpp
	common/textureFramebuffer.hpp

	external/image_ppm.cpp
	external/image_ppm.h

	Crous_man/Util.cpp
	Crous_man/Util.hpp

	Crous_man/Transform.cpp
	Crous_man/Transform.hpp

	Crous_man/Shaders/Shader.cpp
	Crous_man/Shaders/Shader.hpp
	Crous_man/Shaders/MeshShader.cpp
	Crous_man/Shaders/MeshShader.hpp
	Crous_man/Shaders/QuadShader.cpp
	Crous_man/Shaders/QuadShader.hpp
	Crous_man/Shaders/GShader.cpp
	Crous_man/Shaders/GShader.hpp
	Crous_man/Shaders/MeshEShader.cpp
	Crous_man/Shaders/MeshEShader.hpp
	Crous_man/Shaders/QuadEShader.cpp
	Crous_man/Shaders/QuadEShader.hpp
	Crous_man/Shaders/LShader.cpp
	Crous_man/Shaders/LShader.hpp
	Crous_man/Shaders/PEShader.cpp
	Crous_man/Shaders/PEShader.hpp

	Crous_man/Shaders/GShaders/BasicGShader.cpp
	Crous_man/Shaders/GShaders/BasicGShader.hpp
	Crous_man/Shaders/GShaders/TextureGShader.cpp
	Crous_man/Shaders/GShaders/TextureGShader.hpp

	Crous_man/Shaders/EShaders/DepthMeshEShader.cpp
	Crous_man/Shaders/EShaders/DepthMeshEShader.hpp

	Crous_man/Shaders/LShaders/BlinnPhongLShader.cpp
	Crous_man/Shaders/LShaders/BlinnPhongLShader.hpp
	Crous_man/Shaders/LShaders/BlinnPhongShadowLShader.cpp
	Crous_man/Shaders/LShaders/BlinnPhongShadowLShader.hpp

	Crous_man/Shaders/PEShaders/SingleTextureQuadShader.cpp
	Crous_man/Shaders/PEShaders/SingleTextureQuadShader.hpp

	Crous_man/Shaders/ColliderShader.cpp
	Crous_man/Shaders/ColliderShader.hpp

	Crous_man/ECS/Bitmap.cpp
	Crous_man/ECS/Bitmap.hpp
	Crous_man/ECS/Entity.cpp
	Crous_man/ECS/Entity.hpp
	Crous_man/ECS/ComponentSystem.cpp
	Crous_man/ECS/ComponentSystem.hpp
	Crous_man/ECS/EntityManager.cpp
	Crous_man/ECS/EntityManager.hpp
	Crous_man/ECS/EntityBuilder.cpp
	Crous_man/ECS/EntityBuilder.hpp

	Crous_man/Components/Mesh.cpp
	Crous_man/Components/Mesh.hpp
	Crous_man/Components/Spin.cpp
	Crous_man/Components/Spin.hpp
	Crous_man/Components/Renderer.cpp
	Crous_man/Components/Renderer.hpp
	Crous_man/Components/Camera.cpp
	Crous_man/Components/Camera.hpp
	Crous_man/Components/PointLight.cpp
	Crous_man/Components/PointLight.hpp
	Crous_man/Components/Destructible.cpp
	Crous_man/Components/Destructible.hpp

	
	Crous_man/Components/Collider.cpp
	Crous_man/Components/Collider.hpp

	Crous_man/Components/Controllers/FlyingController.cpp
	Crous_man/Components/Controllers/FlyingController.hpp

	Crous_man/InputManager.cpp
	Crous_man/InputManager.hpp
	Crous_man/SoundManager.cpp
	Crous_man/SoundManager.hpp

	Crous_man/Scenes/ECS_test.cpp
	Crous_man/Scenes/ECS_test.hpp)
target_link_libraries(Crous_man
	${ALL_LIBS}
)
# Xcode and Visual working directories
set_target_properties(Crous_man PROPERTIES XCODE_ATTRIBUTE_CONFIGURATION_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Crous_man/")
create_target_launcher(Crous_man WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/Crous_man/")




SOURCE_GROUP(common REGULAR_EXPRESSION ".*/common/.*" )
SOURCE_GROUP(shaders REGULAR_EXPRESSION ".*/.*shader$" )


if (NOT ${CMAKE_GENERATOR} MATCHES "Xcode" )
add_custom_command(
   TARGET Crous_man POST_BUILD
   COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/Crous_man${CMAKE_EXECUTABLE_SUFFIX}" "${CMAKE_CURRENT_SOURCE_DIR}/Crous_man/"
)
elseif (${CMAKE_GENERATOR} MATCHES "Xcode" )

endif (NOT ${CMAKE_GENERATOR} MATCHES "Xcode" )
