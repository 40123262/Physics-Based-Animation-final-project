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
#include "Particle.h"

// main function
int main()
{
	// create application
	Application app = Application::Application();
	app.initRender();
	Application::camera.setCameraPosition(glm::vec3(0.0f, 5.0f, 10.0f));
			
	// create ground plane
	Mesh plane = Mesh::Mesh();
	Mesh plane2 = Mesh::Mesh();
	Mesh cone_0 = Mesh::Mesh();
	Mesh cone_1 = Mesh::Mesh();
	Mesh cone_2 = Mesh::Mesh();
	Mesh cone_3 = Mesh::Mesh();



	//floor
	plane.scale(glm::vec3(10.0f, 10.0f, 10.0f));
	plane.setShader(Shader("resources/shaders/core.vert", "resources/shaders/core.frag"));

	//air cone bottom
	cone_0.scale(glm::vec3(1.0f, 1.0f, 1.0f));
	cone_0.translate(glm::vec3(0.0f, 0.1f, 0.0f));
	cone_0.setShader(Shader("resources/shaders/core.vert", "resources/shaders/cone.frag"));
	//air cone bottom
	cone_1.scale(glm::vec3(2.0f, 2.0f, 2.0f));
	cone_1.translate(glm::vec3(0.0f, 1.0f, 0.0f));
	cone_1.setShader(Shader("resources/shaders/core.vert", "resources/shaders/cone.frag"));
	//air cone bottom
	cone_2.scale(glm::vec3(3.0f, 3.0f, 3.0f));
	cone_2.translate(glm::vec3(0.0f, 2.0f, 0.0f));
	cone_2.setShader(Shader("resources/shaders/core.vert", "resources/shaders/cone.frag"));
	//air cone bottom
	cone_3.scale(glm::vec3(4.0f, 4.0f, 4.0f));
	cone_3.translate(glm::vec3(0.0f, 3.0f, 0.0f));
	cone_3.setShader(Shader("resources/shaders/core.vert", "resources/shaders/cone.frag"));

	//back wall
	plane2.translate(glm::vec3(0.0f, 5.0f, -5.0f));
	plane2.rotate(1.57f, glm::vec3(1.0f, 0.0f, 0.0f));
	plane2.scale(glm::vec3(10.0f, 10.0f, 10.0f));
	plane2.setShader(Shader("resources/shaders/core.vert", "resources/shaders/back.frag"));

	// create particles
	Particle balls[81];
	for (int i = 0; i < sizeof(balls) / sizeof(balls[0]); i++)
	{
		balls[i] = Particle::Particle();
		balls[i].setVel(glm::vec3(0.0f, 0.0f, 0.0f));
		balls[i].getMesh().setShader(Shader("resources/shaders/core.vert", "resources/shaders/core_blue.frag"));
		balls[i].addGravity();
	}

		int x = 0;
		for (int i = 0; i < sqrt(sizeof(balls) / sizeof(balls[0])); i++)
			for (int j = 0; j < sqrt(sizeof(balls) / sizeof(balls[0])); j++)
			{
				balls[x].setPos(glm::vec3(-1.8f + 0.5f*j, 9.5f, -1.8f + 0.5*i));
				x++;
			}

	//balls[0].setPos(glm::vec3(0.0f, 0.1f, 0.0f));
	
	//wind force (2N ->)
	glm::vec3 wind = glm::vec3(2.0f, 0.0f, 0.0f);
	//change mass of one of the balls to see that physics are in check
	// balls[1].setMass(5.0f);
	// Game loop

	// time  (time step solution / task 2)
	GLdouble deltaTime = 0.01;
	GLfloat currentTime = (GLfloat)glfwGetTime();
	GLdouble accumulator = 0.0;

	//adding gravity  (implementation in BODY.CPP)


//	GLfloat airDragForce;

	GLfloat HorizontalForceSignifier;
	GLfloat VerticalForceSignifier;
	while (!glfwWindowShouldClose(app.getWindow()))
	{

		// Set frame time
		GLfloat newTime = (GLfloat)glfwGetTime();
		// the animation can be sped up or slowed down by multiplying currentFrame by a factor.
		GLfloat frameTime = newTime - currentTime;
		currentTime = newTime;		
		accumulator += frameTime;
		/*
		**	INTERACTION
		*/
		// Manage interaction
		app.doMovement(deltaTime);
		////////////////////////////TIME STEP SOLUTION loop
		while (accumulator >= deltaTime)
		{

			/*
			**	SIMULATION  
			*/
			//forces applied, calculations done inside Particle class
			//balls[1].applyForce(wind);
			//balls[2].applyForce(wind);


//////////////////////BLOW DRYER CONE/////////////////////////
			for (int i = 0; i < sizeof(balls) / sizeof(balls[0]); i++)
			{
				if (pow((balls[i].getPos().x), 2) + pow((balls[i].getPos().z), 2) < pow((balls[i].getPos().y + 1), 2) && balls[i].getPos().y < 3)
				{
					//horizontal between 0 and 1. (a part of radius) 1 - particle is far , 0 - particle is in the middle
					HorizontalForceSignifier = (sqrt(pow((balls[i].getPos().x), 2) + pow((balls[i].getPos().z), 2)) / (balls[i].getPos().y + 1));
					//vertical can be between 0 and 3. 3 - at the top of cone, 0, at the bottom
					VerticalForceSignifier = (balls[i].getPos().y);

					GLfloat CompleteForceSignifier =  (3.0f/VerticalForceSignifier)*(1.0f/HorizontalForceSignifier);

					glm::vec3 direction = glm::vec3(balls[i].getPos()) +
			glm::vec3(balls[i].getPos().x, 0.0f, balls[i].getPos().z)/(balls[i].getPos().y + 1);
					glm::vec3 unitV = direction / direction.length();
					balls[i].applyForce(unitV*CompleteForceSignifier*10.0f  + 3.5f*glm::vec3(balls[i].getPos().z/HorizontalForceSignifier, 0.0f, -balls[i].getPos().x/HorizontalForceSignifier));
				}
				else balls[i].applyForce(glm::vec3(0));

				//cube    (implementation in BODY.CPP)
				balls[i].bounceBetween(glm::vec3(-5.0f, 0.05f, -5.0f), glm::vec3(5.0f, 10.0f, 5.0f));
				//		balls[2].bounceBetween(glm::vec3(-5.0f, 0.05f, -5.0f), glm::vec3(5.0f, 10.0f, 5.0f));
						//middle ball - semi implicit euler , right ball forward euler


//////////////////////////////TASK 3//////////////////////////////// (implementation in BODY.CPP)
				balls[i].moveSemiImplicitEuler(deltaTime);
			//	balls[i].moveForwardEuler(deltaTime);
			}
			accumulator -= deltaTime;
		}

		/*
		**	RENDER 
		*/		
		// clear buffer
		app.clear();
		// draw groud and back planes
		app.draw(plane);
		
		app.draw(plane2);
		// draw particles
		for (int i = 0; i < sizeof(balls) / sizeof(balls[0]); i++)
		app.draw(balls[i].getMesh());
		app.draw(cone_0);
		app.draw(cone_1);
		app.draw(cone_2);
		app.draw(cone_3);
		app.display();
	}

	app.terminate();

	return EXIT_SUCCESS;
}

