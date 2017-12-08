#include "RigidBody.h"
#include <vector>
#include <numeric>
#include <string>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtx/matrix_cross_product.hpp>
RigidBody::RigidBody()
{
	
}

RigidBody::~RigidBody()
{
}
glm::mat3 RigidBody::getInvInertia()
{
	float x = getScale()[0][0] * 2.0f;
	float y = getScale()[1][1] * 2.0f;
	float z = getScale()[2][2] * 2.0f;
	float m = getMass();
	float Ix = 1.0f / 12.0f * m * (pow(y, 2) + pow(z, 2));
	float Iy = 1.0f / 12.0f * m * (pow(x, 2) + pow(z, 2));
	float Iz = 1.0f / 12.0f * m * (pow(x, 2) + pow(y, 2));
	glm::mat3 I{Ix, 0, 0,
				0,	Iy, 0,
				0,	0,	Iz};
	return glm::inverse(I);
} //

void RigidBody::applyImpulse(glm::vec3 &J, glm::vec3 point)
{
		
	if (glm::length(getVel()) + glm::length(getAngVel()) < 0.1f)
	{
		setVel(glm::vec3(0.0f));
		setAngVel(glm::vec3(0.0f));
		setAcc(glm::vec3(0.0f));
	}
	else
	{
		setVel(getVel() + J / getMass());
		setAngVel(getAngVel() + getInvInertia()*glm::cross(point-getPos(), J));
	}
		
}
void RigidBody::MonitorPlaneCollisions(Mesh other)
{
	glm::vec3 collisionPoint = checkCollision(other);
	if (collisionPoint != glm::vec3(0))
	{
		Collide(collisionPoint);
	}
}
void RigidBody::Collide(glm::vec3 point)
{
	glm::vec3 applicationP = point - getPos();
	glm::vec3 n = glm::normalize(glm::vec3(0.0f, 1.0f, 0.0f));
	glm::vec3 vr = getVel() + glm::cross(getAngVel(), applicationP);	
	glm::vec3 VerticalImpulse = (-(1 + getEl()) * vr * n) / (pow(getMass(), -1) + n * glm::cross((getInvInertia() * (glm::cross(applicationP, n))), applicationP));
	
	applyImpulse(VerticalImpulse, point);
	
	if (glm::length(VerticalImpulse) > 0.0f)
	{
		glm::vec3 vt = vr - glm::dot(vr, n) * n;
		GLfloat u = 0.1f;
		glm::vec3 Friction = -u * glm::length(VerticalImpulse) * glm::normalize(vt);

		applyImpulse(Friction, point);
	}
}
glm::vec3 RigidBody::checkCollision(Mesh otherBody) 
{
	glm::vec3 sum = glm::vec3(0);
	GLfloat count = 0.0f;	
	std::vector<Vertex> points = getMesh().getVertices();
	glm::vec3 lowestPoint = points[0].getCoord();

	for (Vertex vert : points)
	{
		glm::vec3 collisionP = glm::mat3(getMesh().getModel()) * vert.getCoord() + getPos();
		if (collisionP.y <= otherBody.getPos().y)
		{
			sum += collisionP;
			count += 1.0f;
			if (collisionP.y < lowestPoint.y)
				lowestPoint = collisionP;
		}				
	}
	if (count > 0.0f)
	{
	glm::vec3 displacement = glm::vec3(0.0f);
	displacement.y = glm::abs(lowestPoint.y);
	translate(0.001f*displacement);
		return sum / count;
	}
	else
		return sum;
}
void RigidBody::rotateRB(GLfloat deltaTime)
{
	
	setAngVel(getAngVel() + deltaTime * getAngAcc());
	glm::mat3 angVelSkew = glm::matrixCross3(getAngVel());
	glm::mat3 R = getRotate();

	R += deltaTime*angVelSkew*R;
	R = glm::orthonormalize(R);
	
	setRotate(R);
	
}