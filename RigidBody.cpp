#include "RigidBody.h"
#include <vector>
#include <numeric>
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
	
	if (J.y > 1.5f)
		setVel( J / getMass() + 1.5f*glm::vec3(-(getRotate()*point).x*J.length()*getEl(), 0.0f, -(getRotate()*point).z*J.length()*getEl()));
	else
		setVel( J / getMass() + J.y*glm::vec3(-(getRotate()*point).x*J.length()*getEl(), 0.0f, -(getRotate()*point).z*J.length()*getEl()));
	
	setAngVel(getAngVel()*0.5f + getRotate()*getInvInertia()*glm::transpose(getRotate())*glm::cross(getRotate()*point, J));
	J *= getEl();
}
glm::vec3 RigidBody::checkCollision(Mesh otherBody) 
{
	glm::vec3 sum = glm::vec3(0);
	GLfloat count = 0.0f;	
	std::vector<Vertex> points = getMesh().getVertices();
	Vertex lowestPoint = points[0];
	for (Vertex vert : points)
	{
		if ((getMesh().getModel() *glm::vec4(vert.getCoord(), 1.0f)).y <= otherBody.getPos().y)
		{
			sum+= getScale()*vert.getCoord();
			count += 1.0f;
			if (vert.getCoord().y < lowestPoint.getCoord().y)
				lowestPoint = vert;
		}				
	}
	if (count > 0.0f)
	{
		glm::vec3 displacement = glm::vec3(0.0f);
		displacement.y =  glm::abs(lowestPoint.getCoord().y);
		translate(0.001f*displacement);
		return sum/count;	
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