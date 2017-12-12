#pragma once
#include "Body.h"
struct OBB
{
	glm::vec3 center;
	glm::vec3 u[3];
	glm::vec3 e;
};

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
		OBB getOBB();
		glm::vec3 pointClosestOBB(glm::vec3 p, OBB b);
		void HandleCollision(RigidBody &other, glm::vec3 normal);
		glm::vec3 getAxis(int number, OBB a, OBB b);
		void Collide(glm::vec3 point, glm::vec3 n);
		GLfloat distanceToOBB(glm::vec3 p, OBB b);
		glm::vec3 CheckBodyCollision(RigidBody &other);
		void applyImpulse(glm::vec3 &J, glm::vec3 point);
		void rotateRB(GLfloat deltaTime);
		void MonitorPlaneCollisions(Mesh other);
		glm::vec3 checkCollision(Mesh otherBody);
		//void scale(glm::vec3 vect);
		
			private:
				OBB obb;
				glm::mat3 m_invInertia; // Inverse Inertia
				glm::vec3 m_angVel; // angular velocity
				glm::vec3 m_angAcc ; // angular acceleration
				};
