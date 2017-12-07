#pragma once
// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// Std. Includes
#include <string>
#include <time.h>

// GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"


// Other Libs
#include "SOIL2/SOIL2.h"

// project includes
#include "Application.h"
#include "Shader.h"
#include "Mesh.h"
#include "Body.h"
#include "Force.h"
#include "Particle.h"
#include "RigidBody.h"

// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 20.0f));
	// create ground plane
	Mesh plane = Mesh::Mesh();
	Mesh plane2 = Mesh::Mesh();

	//floor
	plane.scale(glm::vec3(10.0f, 10.0f, 10.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));


	//back wall
	plane2.translate(glm::vec3(0.0f, 5.0f, -5.0f));
	plane2.rotate(1.57f, glm::vec3(1.0f, 0.0f, 0.0f));
	plane2.scale(glm::vec3(10.0f, 10.0f, 10.0f));
	plane2.setShader(Shader("resources/shaders/core.vert", "resources/shaders/back.frag"));
	
	Gravity g = Gravity::Gravity(glm::vec3(0.0f, -9.8f, 0.0f));
	RigidBody cb = RigidBody();
	Mesh cube = Mesh::Mesh(Mesh::CUBE);

	cube.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	cb.setMesh(cube);	
	cb.setMass(2.0f);
	cb.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	//cb.rotate(M_PI_2, glm::vec3(0.0f, 0.0f, 1.0f));
	cb.setAngVel(glm::vec3(1.5f, 0.0f, 0.5f));
	cb.translate(glm::vec3(0.0f, 14.0f, 0.0f));
	cb.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	cb.addForce(&g);
	// Game loop
	std::cout << glm::to_string(cb.getInvInertia());
	cb.setEl(0.7f);
	// time  (time step solution / task 2)
	GLdouble deltaTime = 0.001;
	GLfloat currentTime = (GLfloat)glfwGetTime();
	GLdouble accumulator = 0.0;
	glm::vec3 J = glm::vec3(0, 13.0f, 0);
	////////////task 1//////////////
	while (!glfwWindowShouldClose(app.getWindow()))
	{

		// Set frame time
		GLfloat newTime = (GLfloat)glfwGetTime();
		// the animation can be sped up or slowed down by multiplying currentFrame by a factor.
		GLfloat frameTime = newTime - currentTime;
		currentTime = newTime ;		
		accumulator += frameTime;
		/*
		**	INTERACTION
		*/
		// Manage interaction
		app.doMovement(frameTime);
		
		////////////////////////////TIME STEP SOLUTION loop
		while (accumulator >= deltaTime)
		{
			glm::vec3 temp = cb.checkCollision(plane);
			if (temp != glm::vec3(0))
			{
				cb.Collide(temp);
			}
			
			cb.rotateRB(deltaTime);
			cb.move(deltaTime);
			
			accumulator -= deltaTime;
		}

		/*
		**	RENDER 
		*/		
		// clear buffer
		app.clear();
		// draw groud and back planes
		app.draw(plane);		
	//	app.draw(plane2);
		// draw cube
		app.draw(cb.getMesh());
	
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

