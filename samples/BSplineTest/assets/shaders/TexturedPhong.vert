#version 150

uniform mat4	ciModelViewProjection;
uniform mat4	ciModelView;
uniform mat3	ciNormalMatrix;

in vec4		ciPosition;
in vec3		ciNormal;
in vec3		ciTangent;
in vec3		ciBitangent;
in vec2		ciTexCoord0;

out VertexData {
	vec4 position;
	vec3 normal;
	vec3 tangent;
	vec3 bitangent;
	vec2 texCoord;
} vVertexOut;

void main( void )
{
	gl_Position	= ciModelViewProjection * ciPosition;
	
	vVertexOut.position = ciModelView * ciPosition;
	vVertexOut.normal = ciNormalMatrix * ciNormal;
	vVertexOut.tangent = ciNormalMatrix * ciTangent;
	vVertexOut.bitangent = ciNormalMatrix * ciBitangent;
	vVertexOut.texCoord = ciTexCoord0;
}
