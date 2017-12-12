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
		Collide(collisionPoint, glm::vec3(0.0f, 1.0f, 0.0f));
	}
}
void RigidBody::Collide(glm::vec3 point, glm::vec3 n)
{
	glm::vec3 applicationP = point - getPos();
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
OBB RigidBody::getOBB()
{
	OBB temp;
	temp.u[0] = glm::normalize(glm::vec3(getMesh().getRotate()[0]));
	temp.u[1] = glm::normalize(glm::vec3(getMesh().getRotate()[1]));
	temp.u[2] = glm::normalize(glm::vec3(getMesh().getRotate()[2]));
	temp.center = getPos();
	temp.e[0] = getScale()[0][0];
	temp.e[1] = getScale()[1][1];
	temp.e[2] = getScale()[2][2];
	obb = temp;
	return temp;
}
glm::vec3 RigidBody::CheckBodyCollision(RigidBody &other)
{

	int AxisNumber=0;
	float lowestPenetration = 100.0f;
	OBB a = getOBB();
	OBB b = other.getOBB();
	GLfloat depth;

	float ra, rb;
	glm::mat3 R, AbsR;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(a.u[i], b.u[j]);

	glm::vec3 t = b.center - a.center;

	t = glm::vec3(glm::dot(t, a.u[0]), glm::dot(t, a.u[1]), glm::dot(t, a.u[2]));

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			AbsR[i][j] = abs(R[i][j]) + 0.001f;

	for (int i = 0; i < 3; i++)
	{

		ra = a.e[i];
		rb = b.e[0] * AbsR[i][0] + b.e[1] * AbsR[i][1] + b.e[2] * AbsR[i][2];
		if (abs(t[i]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[i]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 1 + i;
			}
		}
	}
	for (int i = 0; i < 3; i++)
	{
		ra = a.e[0] * AbsR[0][i] + a.e[1] * AbsR[1][i] + a.e[2] * AbsR[2][i];
		rb = b.e[i];
		if (abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2]*R[2][i]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 4 + i;
			}
		}
	}
	// Test axis L = A0 x B0
	glm::vec3 m = glm::cross(a.u[0], b.u[0]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[1] * AbsR[2][0] + a.e[2] * AbsR[1][0];
		rb = b.e[1] * AbsR[0][2] + b.e[2] * AbsR[0][1];
		if (abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[2] * R[1][0] - t[1] * R[2][0]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 7;
			}
		}
	}
	// Test axis L = A0 x B1
	m = glm::cross(a.u[0], b.u[1]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[1] * AbsR[2][1] + a.e[2] * AbsR[1][1];
		rb = b.e[0] * AbsR[0][2] + b.e[2] * AbsR[0][0];
		if (abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[2] * R[1][1] - t[1] * R[2][1]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 8;
			}
		}
	}
	// Test axis L = A0 x B2
	m = glm::cross(a.u[0], b.u[2]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[1] * AbsR[2][2] + a.e[2] * AbsR[1][2];
		rb = b.e[0] * AbsR[0][1] + b.e[1] * AbsR[0][0];
		if (abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[2] * R[1][2] - t[1] * R[2][2]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 9;
			}
		}
	}
	// Test axis L = A1 x B0
	m = glm::cross(a.u[1], b.u[0]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[0] * AbsR[2][0] + a.e[2] * AbsR[0][0];
		rb = b.e[1] * AbsR[1][2] + b.e[2] * AbsR[1][1];
		if (abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[0] * R[2][0] - t[2] * R[0][0]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 10;
			}
		}
	}
	// Test axis L = A1 x B1
	m = glm::cross(a.u[1], b.u[1]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[0] * AbsR[2][1] + a.e[2] * AbsR[0][1];
		rb = b.e[0] * AbsR[1][2] + b.e[2] * AbsR[1][0];
		if (abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[0] * R[2][1] - t[2] * R[0][1]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 11;
			}
		}
	}
	// Test axis L = A1 x B2
	m = glm::cross(a.u[0], b.u[2]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[0] * AbsR[2][2] + a.e[2] * AbsR[0][2];
		rb = b.e[0] * AbsR[1][1] + b.e[1] * AbsR[1][0];
		if (abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[0] * R[2][2] - t[2] * R[0][2]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 12;
			}
		}
	}
	// Test axis L = A2 x B0
	m = glm::cross(a.u[2], b.u[0]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[0] * AbsR[1][0] + a.e[1] * AbsR[0][0];
		rb = b.e[1] * AbsR[2][2] + b.e[2] * AbsR[2][1];
		if (abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[1] * R[0][0] - t[0] * R[1][0]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 13;
			}
		}
	}
	// Test axis L = A2 x B1
	m = glm::cross(a.u[2], b.u[1]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
		ra = a.e[0] * AbsR[1][1] + a.e[1] * AbsR[0][1];
		rb = b.e[0] * AbsR[2][2] + b.e[2] * AbsR[2][0];
		if (abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) return glm::vec3(0);
		else
		{
			depth = (ra + rb) - abs(t[1] * R[0][1] - t[0] * R[1][1]);
			if (depth < lowestPenetration)
			{
				lowestPenetration = depth;
				AxisNumber = 14;
			}
		}
	}
	// Test axis L = A2 x B2
	m = glm::cross(a.u[2], b.u[2]);
	if (!(abs(glm::length(m)) <= FLT_EPSILON))
	{
	ra = a.e[0] * AbsR[1][2] + a.e[1] * AbsR[0][2];
	rb = b.e[0] * AbsR[2][1] + b.e[1] * AbsR[2][0];
	if (abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) return glm::vec3(0);
	else
	{
		depth = (ra + rb) - abs(t[1] * R[0][2] - t[0] * R[1][2]);
		if ( depth < lowestPenetration)
		{
			lowestPenetration = depth;
			AxisNumber = 15;
		}
	}
	}
//	std::cout << "COLLISION! "<< AxisNumber << std::endl;
	return getAxis(AxisNumber, a, b);
}

void RigidBody::HandleCollision(RigidBody &other, glm::vec3 normal)
{
	GLfloat minDist = 100.0f;
	glm::vec3 contacts = glm::vec3(0);
	GLfloat count = 0;
	for (Vertex vert : getMesh().getVertices())
	{
		GLfloat currentDist = distanceToOBB(glm::mat3(getMesh().getModel())*vert.getCoord(), other.obb);
		if (currentDist < minDist)
			minDist = currentDist;
	}

	for (Vertex vert : getMesh().getVertices())
	{
		GLfloat currentDist = distanceToOBB(glm::mat3(getMesh().getModel())*vert.getCoord(), other.obb);
		if (currentDist == minDist)
		{
			contacts += pointClosestOBB(glm::mat3(getMesh().getModel())*vert.getCoord(), obb);
			count ++;
		}
	}
	for (Vertex vert : other.getMesh().getVertices())
	{
		GLfloat currentDist = distanceToOBB(glm::mat3(other.getMesh().getModel())*vert.getCoord(), obb);
		if (currentDist < minDist)
		{
			minDist = currentDist;
			contacts = glm::vec3(0);
			count = 0.0f;
		}
	}
	for (Vertex vert : other.getMesh().getVertices())
	{
		GLfloat currentDist = distanceToOBB(glm::mat3(other.getMesh().getModel())*vert.getCoord(), obb);
		if (currentDist == minDist)
		{
			contacts += pointClosestOBB(glm::mat3(other.getMesh().getModel())*vert.getCoord(), other.obb);
			count++;
		}
	}

	glm::vec3 applicationPoint = contacts / count;

	
	glm::vec3 r1 = applicationPoint - obb.center;
	glm::vec3 r2 = applicationPoint - other.obb.center;
	glm::vec3 vr = other.getVel() + glm::cross(other.getAngVel(), r2) -(getVel() + glm::cross(getAngVel(), r1));

	GLfloat impulse = (-(1 + getEl())* glm::dot(vr , normal)) / (pow(getMass(), -1) + pow(other.getMass(), -1)
		+ glm::dot(normal , glm::cross(getInvInertia()*glm::cross(r1, normal), r1))
		+ glm::dot(normal , glm::cross(other.getInvInertia()*glm::cross(r2, normal), r2)));

	setVel(getVel() - impulse/getMass()* normal);
	setAngVel(getAngVel() - (impulse*getInvInertia()*(glm::cross(r1, normal))));

	other.setVel(other.getVel() + impulse /other.getMass()* normal );
	other.setAngVel(other.getAngVel() + (impulse*other.getInvInertia()*(glm::cross(r2, normal))));	
}
glm::vec3 RigidBody::pointClosestOBB(glm::vec3 p, OBB b)
{
	glm::vec3 d = p - b.center;
	glm::vec3 Q = b.center;

	for (int i = 0; i < 3; i++)
	{
		GLfloat dist = glm::dot(d, b.u[i]);
		if (dist > b.e[i]) dist = b.e[i];
		if (dist < -b.e[i]) dist = -b.e[i];
		Q += dist * b.u[i];
	}
	return Q;		
}
GLfloat RigidBody::distanceToOBB(glm::vec3 p, OBB b)
{
	glm::vec3 Closest = pointClosestOBB(p, b);
	GLfloat sqDist = glm::dot(Closest - p, Closest - p);
	return sqDist;
}
glm::vec3 RigidBody::getAxis(int number, OBB a, OBB b)
{
	switch (number)
	{
	case 1:
		return a.u[0];
	case 2:
		return a.u[1];
	case 3:
		return a.u[2];
	case 4:
		return b.u[0];
	case 5:
		return b.u[1];
	case 6:
		return b.u[2];
	case 7:
		return glm::cross(a.u[0], b.u[0]);
	case 8:
		return glm::cross(a.u[0], b.u[1]);
	case 9:
		return glm::cross(a.u[0], b.u[2]);
	case 10:
		return glm::cross(a.u[1], b.u[0]);
	case 11:
		return glm::cross(a.u[1], b.u[1]);
	case 12:
		return glm::cross(a.u[1], b.u[2]);
	case 13:
		return glm::cross(a.u[2], b.u[0]);
	case 14:
		return glm::cross(a.u[2], b.u[1]);
	case 15:
		return glm::cross(a.u[2], b.u[2]);
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