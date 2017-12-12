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
	RigidBody cb2 = RigidBody();
	RigidBody cb3 = RigidBody();

	Particle p = Particle();
	p.setPos(glm::vec3(0.0f, 10.0f, 0.0f));

	Mesh cube = Mesh::Mesh(Mesh::CUBE);
	Mesh cube2 = Mesh::Mesh(Mesh::CUBE);

	
	cube.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	cube2.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
	


	cb.setMesh(cube);	
	cb.setMass(1.0f);
	cb.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	cb.setAngVel(glm::vec3(0.0f, 0.0f, 0.000001f));
	cb.translate(glm::vec3(0.0f, 3.0f, 0.0f));
	cb.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	cb.setEl(0.6f);

	cb2.setMesh(cube2);
	cb2.setMass(1.0f);
	cb2.scale(glm::vec3(1.0f, 3.0f, 1.0f));
	cb2.setAngVel(glm::vec3(0.0f, 0.0f, -1.0001f));
	cb2.translate(glm::vec3(-3.0f, 4.0f, 0.0f));
	cb2.setVel(glm::vec3(0.0f, 0.0f, 0.0f));
	cb2.setEl(0.6f);





	cb.addForce(&g);
	cb2.addForce(&g);


	
	// Game loop
	std::cout << glm::to_string(cb.getInvInertia());

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
			
				
			glm::vec3 collisionNormal = cb2.CheckBodyCollision(cb);
			if (glm::length(collisionNormal) > 0.0f)
			{
				cb.HandleCollision(cb2, glm::normalize(collisionNormal));
			}


			
				cb.MonitorPlaneCollisions(plane);
				cb.rotateRB(deltaTime);
				cb.move(deltaTime);
			
			
				cb2.MonitorPlaneCollisions(plane);
				cb2.rotateRB(deltaTime);
				cb2.move(deltaTime);
			
			

			
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
		app.draw(cb2.getMesh());
		
	
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

