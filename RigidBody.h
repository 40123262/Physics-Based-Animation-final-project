#pragma once
#include "Body.h"

class RigidBody :
	public Body
	{
	public:
		RigidBody();
		~RigidBody();
		
			// set and get methods
		void setAngVel(const glm::vec3 & omega) { m_angVel = omega; }
		void setAngAccl(const glm::vec3 & alpha) { m_angAcc = alpha; }
		void setInvInertia(const glm::mat3 & invInertia) { m_invInertia = invInertia; }
		glm::vec3 getAngVel() { return m_angVel; }
		glm::vec3 getAngAcc() { return m_angAcc; }
		glm::mat3 getInvInertia();
		void Collide(glm::vec3 point);
		void applyImpulse(glm::vec3 &J, glm::vec3 point);
		void rotateRB(GLfloat deltaTime);
		glm::vec3 checkCollision(Mesh otherBody);
		//void scale(glm::vec3 vect);
		
			private:
				glm::mat3 m_invInertia; // Inverse Inertia
				glm::vec3 m_angVel; // angular velocity
				glm::vec3 m_angAcc ; // angular acceleration
				};
