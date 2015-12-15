
##GL_VERTEX_SHADER

#version 400
layout(location=0) in vec3 in_position;
layout(location=1) in vec3 in_normal;
layout(location=2) in vec2 in_texture;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform mat4 MV;
uniform mat4 MVP;

out vec3 normal;
out vec2 textureCoords;
out vec3 lightDir;

void main() {
	normal = (MV * vec4(in_normal, 0.0)).xyz;
	textureCoords = in_texture;
	lightDir = normalize((view*vec4(1,1,1,0)).xyz);
	gl_Position = MVP * vec4(in_position, 1.0);
}


##GL_FRAGMENT_SHADER

#version 400

in vec3 normal;
in vec2 textureCoords;
in vec3 lightDir;


layout(location=0) out vec4 out_color;

void main() {

    //complicated lighting model
    float intensity = max(0.1,dot(normal, lightDir));

    out_color = intensity* vec4(1.0, 0.0, 0.0, 1.0);
}

##end
