#version 440 core
out vec4 color;
in vec3 positionx;
void main()
{
	if(pow(positionx.x,2)+pow(positionx.z,2) > 1)
	discard;
 color = vec4(1.0, 0.2, 0.2, 0.2);
}