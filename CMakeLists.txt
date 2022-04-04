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
	.
)

set(ALL_LIBS
	${OPENGL_LIBRARY}
	glfw
	GLEW_1130
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
	common/gBuffer.cpp
	common/gBuffer.hpp

	external/image_ppm.cpp
	external/image_ppm.h

	Crous_man/Util.cpp
	Crous_man/Util.hpp

	Crous_man/Mesh.cpp
	Crous_man/Mesh.hpp
	Crous_man/Scene.cpp
	Crous_man/Scene.hpp
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
	Crous_man/Shaders/LShader.cpp
	Crous_man/Shaders/LShader.hpp
	Crous_man/Shaders/BasicGShader.cpp
	Crous_man/Shaders/BasicGShader.hpp
	Crous_man/Shaders/BlinnPhongLShader.cpp
	Crous_man/Shaders/BlinnPhongLShader.hpp

	Crous_man/ECS/Bitmap.cpp
	Crous_man/ECS/Bitmap.hpp
	Crous_man/ECS/Entity.cpp
	Crous_man/ECS/Entity.hpp
	Crous_man/ECS/ComponentSystem.cpp
	Crous_man/ECS/ComponentSystem.hpp
	Crous_man/ECS/EntityManager.cpp
	Crous_man/ECS/EntityManager.hpp

	Crous_man/Components/Mesh.cpp
	Crous_man/Components/Mesh.hpp
	Crous_man/Components/Spin.cpp
	Crous_man/Components/Spin.hpp
	Crous_man/Components/Renderer.cpp
	Crous_man/Components/Renderer.hpp
	Crous_man/Components/Camera.cpp
	Crous_man/Components/Camera.hpp
	Crous_man/Components/Controllers/FlyingController.cpp
	Crous_man/Components/Controllers/FlyingController.hpp
	

	Crous_man/SceneObjects/Camera.cpp
	Crous_man/SceneObjects/Camera.hpp
	Crous_man/SceneObjects/Terrain.cpp
	Crous_man/SceneObjects/Terrain.hpp
	Crous_man/SceneObjects/LODObject.cpp
	Crous_man/SceneObjects/LODObject.hpp
	Crous_man/SceneObjects/PointLight.cpp
	Crous_man/SceneObjects/PointLight.hpp

	Crous_man/SceneObjects/Solaire/Planet.cpp
	Crous_man/SceneObjects/Solaire/Planet.hpp
	Crous_man/SceneObjects/Solaire/Sun.cpp
	Crous_man/SceneObjects/Solaire/Sun.hpp
	Crous_man/SceneObjects/Solaire/Earth.cpp
	Crous_man/SceneObjects/Solaire/Earth.hpp
	Crous_man/SceneObjects/Solaire/Moon.cpp
	Crous_man/SceneObjects/Solaire/Moon.hpp
	Crous_man/SceneObjects/Solaire/SpinningObject.cpp
	Crous_man/SceneObjects/Solaire/SpinningObject.hpp

	Crous_man/SceneObjects/UtilObjects/MouseControlObject.cpp
	Crous_man/SceneObjects/UtilObjects/MouseControlObject.hpp
	Crous_man/SceneObjects/UtilObjects/KeyboardControlObject.cpp
	Crous_man/SceneObjects/UtilObjects/KeyboardControlObject.hpp
	Crous_man/SceneObjects/UtilObjects/CharacterControlObject.cpp
	Crous_man/SceneObjects/UtilObjects/CharacterControlObject.hpp
	Crous_man/SceneObjects/UtilObjects/MouseVerticalControlObject.cpp
	Crous_man/SceneObjects/UtilObjects/MouseVerticalControlObject.hpp


	Crous_man/InputManager.cpp
	Crous_man/InputManager.hpp

	Crous_man/Scenes/SampleScene.cpp
	Crous_man/Scenes/SampleScene.hpp
	Crous_man/Scenes/Systeme_solaire.cpp
	Crous_man/Scenes/Systeme_solaire.hpp
	Crous_man/Scenes/Object_control.cpp
	Crous_man/Scenes/Object_control.hpp
	Crous_man/Scenes/ECS_test.cpp
	Crous_man/Scenes/ECS_test.hpp
)
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

