#version 330 core
layout(location = 0) in vec4 position_in;
layout(location = 1) in vec4 normal_in;
layout(location = 2) in vec4 m0;
layout(location = 3) in vec4 m1;
layout(location = 4) in vec4 m2;
layout(location = 5) in vec4 m3;

out VS_OUT {
  vec3 position;
  vec3 normal;
} vs_out;

layout (std140) uniform camera {
  mat4 viewProjectionMatrix;
  vec4 viewPosition;
};

void main() {
  mat4 model = mat4(m0, m1, m2, m3);
  mat3 normalMatrix = transpose(inverse(mat3(model)));
  vs_out.position = vec3(model * position_in);
  vs_out.normal = normalMatrix * normal_in.xyz;
  gl_Position = viewProjectionMatrix * model * position_in;
}
