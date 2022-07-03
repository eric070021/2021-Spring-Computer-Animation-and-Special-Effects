#version 330 core

layout(location = 0) in vec4 position_in;
layout(location = 1) in vec4 normal_in;

out VS_OUT {
  vec3 position;
  vec3 normal;
  flat vec3 viewPosition;
} vs_out;

layout (std140) uniform model {
  mat4 modelMatrix;
  mat4 normalMatrix;
};

layout (std140) uniform camera {
  mat4 viewProjectionMatrix;
  vec4 viewPosition;
};

void main() {
  gl_Position = viewProjectionMatrix * modelMatrix * position_in;
  vs_out.position = (modelMatrix * position_in).xyz;
  vs_out.normal = vec3(normalMatrix * normal_in);
  vs_out.viewPosition = viewPosition.xyz;
}
