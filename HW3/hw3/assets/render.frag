#version 330 core
layout(location = 0) out vec4 FragColor;

in VS_OUT {
  vec3 position;
  vec3 normal;
} fs_in;

layout (std140) uniform camera {
  mat4 viewProjectionMatrix;
  vec4 viewPosition;
};

uniform vec4 inputColor;

void main() {
  vec3 lightDirection = normalize(vec3(25.0, 12.0, -10.0));
  vec3 normal = normalize(fs_in.normal);

  vec3 viewDirection = normalize(viewPosition.xyz - fs_in.position);

  vec3 halfwayDirection = normalize(lightDirection + viewDirection);
  float ambient = 0.1;
  float normalDotLight = dot(normal, lightDirection);
  float diffuse = max(normalDotLight, 0.0);
  float specular = 0.2 * pow(max(dot(normal, halfwayDirection), 0.0), 32.0);
  float lighting = ambient + diffuse + specular;
  FragColor = vec4(lighting * inputColor.rgb, inputColor.a);
}
