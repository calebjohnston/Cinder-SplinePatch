#version 150

uniform float alpha;
uniform vec3 ambientColor;
uniform vec3 specularColor;
uniform vec3 diffuseColor;

in vec3 normalVector;
in vec3 lightVector;
in vec3 eyeVector;

out vec4 FragColor;

void main() {
	vec3 light_color = diffuseColor;//ambientColor;

	float nDotL = dot(normalVector, lightVector);
	float magma = 1.0 - abs(dot(normalVector, eyeVector));

	vec3 halfVector = normalize(lightVector + eyeVector);
	float specular = pow( max( dot(normalVector, halfVector), 0.0), 24.0);
	light_color += (diffuseColor * nDotL) + (specularColor * specular);

	FragColor = vec4(light_color.rgb, magma);
}