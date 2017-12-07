# include <iostream>
#include <cmath>
#include<vector>
#include"Force.h"
#include"Body.h"
#include"glm/ext.hpp "
glm::vec3 cross(glm::vec3 fst, glm::vec3 snd)
{
	return glm::vec3(fst.y*snd.z-fst.z*snd.y, fst.z*snd.x - fst.x*snd.z, fst.x*snd.y- fst.y*snd.x);
}

glm::vec3 Force::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel) 
{
	return force_vect;
}
void Force::set(glm::vec3 pos)
{
	force_vect = pos;
}
/*
** GRAVITY
*/
glm::vec3 Gravity::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel) 
{
	return glm::vec3(0.0f,-9.8f,0.0f)*mass;
}

/*
** DRAG
*/
glm::vec3 Drag::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel) 
{
	
	return -vel / vel.length()*(0.252f * vel.length() * vel.length());
}
/*
** HOOKE // SPRINGS
*/
glm::vec3 Hooke::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	float dist = sqrt(pow((m_b1->getPos().x - m_b2->getPos().x ),2)+ 
			          pow((m_b1->getPos().y - m_b2->getPos().y ),2)+ 
		              pow((m_b1->getPos().z - m_b2->getPos().z ),2));
	float displacement = m_rest - dist;

	glm::vec3 direction = -(m_b1->getPos()-m_b2->getPos()) / dist;
	return direction*m_ks*(m_rest - dist) - m_kd*(m_b2->getVel()-m_b1->getVel());
}
glm::vec3 Aero::apply(float mass, const glm::vec3 & pos, const glm::vec3 & vel)
{
	glm::vec3 normal = cross(m_b2->getPos() - m_b1->getPos(), m_b3->getPos()-m_b1->getPos());
	GLfloat  area = 0.5f*normal.length();
	normal = normal / normal.length();

	return -m_b1->getVel() / m_b1->getVel().length()*(0.652f * m_b1->getVel().length() * m_b1->getVel().length()) * area/3.0F;
}