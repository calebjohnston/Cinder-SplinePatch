#version 150

uniform sampler2D	uTex0;
uniform vec3		lightPosition;

in VertexData	{
	vec4 position;
	vec3 normal;
	vec3 tangent;
	vec3 bitangent;
	vec2 texCoord;
} vVertexIn;

out vec4 FragColor;

void main()
{
	// set diffuse and specular colors
	vec3 cDiffuse = texture( uTex0, vVertexIn.texCoord.st ).rgb;
	vec3 cSpecular = vec3( 0.3 );
	
	// lighting calculations
	vec3 vVertex = vVertexIn.position.xyz;
	vec3 vNormal = normalize( vVertexIn.normal );
	vec3 vToLight = normalize( lightPosition - vVertex );
	vec3 vToEye = normalize( -vVertex );
	vec3 vReflect = normalize( -reflect(vToLight, vNormal) );
	
	// diffuse coefficient
	vec3 diffuse = max( dot( vNormal, vToLight ), 0.0 ) * cDiffuse;
	
	// specular coefficient with energy conservation
	const float shininess = 96.0;
	const float coeff = (2.0 + shininess) / (2.0 * 3.14159265);
	vec3 specular = pow( max( dot( vReflect, vToEye ), 0.0 ), shininess ) * coeff * cSpecular;
	
	// final color
	FragColor = vec4( diffuse + abs(specular), 1.0 );
}
