#version 150

uniform mat4 ciModelViewProjection;
uniform mat4 ciModelView;
uniform mat3 ciNormalMatrix;

uniform vec4 eyePosition;
uniform vec4 lightPosition;

in vec4	ciPosition;
in vec3	ciNormal;

out vec3 lightVector;
out vec3 eyeVector;
out vec3 normalVector;

void main() {
	// store the vertex position in world space
	vec4 world_vert_position = ciModelView * ciPosition;

	// perform full MVP transformation
	gl_Position = ciModelViewProjection * ciPosition;

	// write vectors for fragment stage
	normalVector = normalize(ciNormalMatrix * ciNormal).xyz;
	lightVector = normalize(lightPosition - world_vert_position).xyz;
	eyeVector = normalize(eyePosition - world_vert_position).xyz;
}