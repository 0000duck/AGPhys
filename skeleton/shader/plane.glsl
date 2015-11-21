
##GL_VERTEX_SHADER

#version 400
layout(location=0) in vec3 in_position;
layout(location=1) in vec3 in_normal;
layout(location=2) in vec3 in_d1;
layout(location=3) in vec3 in_d2;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform mat4 MV;
uniform mat4 MVP;

out vec3 normal;
out vec3 d1;
out vec3 d2;

void main() {
    normal = normalize(vec3(view*model * vec4( in_normal, 0 )));
    d1 = in_d1;
    d2 = in_d2;
    gl_Position = model* vec4(in_position,1);
}


##GL_GEOMETRY_SHADER
#version 400

layout(points) in;
in vec3[1] normal;
in vec3[1] d1;
in vec3[1] d2;
layout(triangle_strip, max_vertices=4) out;
uniform mat4 proj;
uniform mat4 view;

out vec3 normal2;
out vec3 lightDir;

void main() {

    vec3 L = normalize(vec3(view*vec4(1,1,1,0)));

  vec4 pos = gl_in[0].gl_Position;
  vec3 normal = normal[0];
  vec3 d1 = d1[0];
  vec3 d2 = d2[0];

  // vert 1
  lightDir = L;
  normal2 = normal;
  gl_Position = proj * view * (pos + vec4(d1 + d2, 0));
  EmitVertex();

  // vert 2
  lightDir = L;
  normal2 = normal;
  gl_Position = proj * view * (pos - vec4(d1, 0) + vec4(d2, 0));
  EmitVertex();

  // vert 3
  lightDir = L;
  normal2 = normal;
  gl_Position = proj * view * (pos + vec4(d1, 0) - vec4(d2, 0));
  EmitVertex();

  // vert 4
  lightDir = L;
  normal2 = normal;
  gl_Position = proj * view * (pos - vec4(d1, 0) - vec4(d2, 0));
  EmitVertex();


}


##GL_FRAGMENT_SHADER

#version 400



in vec3 normal2;
in vec3 lightDir;
layout(location=0) out vec4 out_color;

void main() {


    //complicated lighting model
    float intensity = max(0.1,dot(normal2, lightDir));

    out_color = intensity* vec4(1.0, 0.0, 0.0, 1.0);
}

##end
