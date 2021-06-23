#version 150

uniform mat4	ciModelViewProjection;
uniform mat4	ciModelMatrix;
uniform mat3	ciNormalMatrix;
uniform vec3 	uLightPos;

in vec4		ciPosition;
// in vec2		ciTexCoord0;
in vec4     ciColor;
in vec3		ciNormal;

out lowp vec4	Color;
out highp vec3	Normal;
out highp vec4 Light;
// out highp vec2	TexCoord0;

void main( void )
{
	gl_Position	= ciModelViewProjection * ciPosition;
	//TexCoord0	= ciTexCoord0;
	Light =  ciModelMatrix * vec4(uLightPos, 1);
	//Light = ciModelMatrix*vec4( 0, 0, -1, 1);
	Color = ciColor;
	Normal		= ciNormalMatrix * ciNormal;
}
