#include "Body.h"

Body::Body()
{
}

Body::~Body()
{
}

/* TRANSFORMATION METHODS*/
void Body::translate(const glm::vec3 &vect) {
	m_pos = m_pos + vect;
	m_mesh.translate(vect);
}

void Body::rotate(float angle, const glm::vec3 &vect) {
	m_mesh.rotate(angle, vect);
}

void Body::scale(const glm::vec3 &vect) {
	m_mesh.scale(vect);
}

glm::vec3 Body::applyForces(glm::vec3 x, glm::vec3 v, float dt) //returns acc from each force
{
	glm::vec3 fAccumulator = glm::vec3(0.0f);
	for (auto &f : m_forces)
	{
		fAccumulator += f->apply(m_mass, x, v);
	}
	return fAccumulator/getMass();
}
void Body::move(GLfloat deltaTime)
{
		setAcc(applyForces(m_pos, m_vel, deltaTime)); 

		setVel(m_vel + (m_acc*deltaTime));
		this->setPos(m_pos + m_vel*deltaTime);
}
void Body::moveForwardEuler(GLfloat deltaTime)
{
		this->setPos(m_pos + m_vel*deltaTime);
		m_vel += (m_acc*deltaTime);
}
void Body::restrict(const glm::vec3 &bottom, const glm::vec3 &top)
{
	if (m_pos.y < bottom.y)
	{
		m_pos.y = bottom.y;
		m_vel.y *= -0.9f;
	}
	else if (m_pos.y > top.y)
	{
		m_pos.y = top.y;
		m_vel.y *= -1.0f;
	}
	if (m_pos.x < bottom.x)
	{
		m_pos.x = bottom.x;
		m_vel.x *= -1.0f;
	}
	else if (m_pos.x > top.x)
	{
		m_pos.x = top.x;
		m_vel.x *= -1.0f;
	}
	if (m_pos.z < bottom.z)
	{
		m_pos.z = bottom.z;
		m_vel.z *= -1.0f;
	}
	else if (m_pos.z > top.z)
	{
		m_pos.z = top.z;
		m_vel.z *= -1.0f;
	}

}


