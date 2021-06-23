#version 150

//uniform sampler2D	uTex0;

in vec4		Color;
in vec3		Normal;
// in vec2		TexCoord0;
in vec4		Light;

out vec4 	oColor;

void main( void )
{
	vec3 normal = normalize( -Normal );
	// float diffuse = max( dot( normal, vec3( 0, 0, -1 ) ), 0 );
	float diffuse = max( dot( normal, normalize(Light.xyz) ), 0.1 );
	float camLight = max( dot( normal, vec3( 0, 0, -1 ) ), 0 );
	// vec3 color = texture( uTex0, TexCoord0 ).rgb * diffuse;
	vec3 color = Color.xyz * diffuse * 0.8 + Color.xyz * camLight * 0.2;
	oColor = vec4( color, 1.0 );
}