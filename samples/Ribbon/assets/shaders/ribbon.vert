
uniform vec4 eyePosition, lightPosition;
varying vec3 lightVector, eyeVector, normalVector;

void main() {
	// store the vertex position in world space
	vec4 world_vert_position = gl_ModelViewMatrix * gl_Vertex;

	// perform full MVP transformation
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

	// write vectors for fragment stage
	normalVector = normalize(gl_NormalMatrix * gl_Normal).xyz;
	lightVector = normalize(lightPosition - world_vert_position).xyz;
	eyeVector = normalize(eyePosition - world_vert_position).xyz;
}