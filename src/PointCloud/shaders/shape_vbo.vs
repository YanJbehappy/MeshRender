#version 450

layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec3 aColor;

out vec3 Color;

uniform mat4 uWorldViewProj;
uniform int uSize;

// vec3 getColorFromV3(){
// 	vec3 v = vec3(
// 		(aColor >>  16) & 0xFF, // r
// 		(aColor >>   8) & 0xFF, // g
// 		(aColor >>   0) & 0xFF  // b
// 	);
// 	v = v / 255.0;

// 	return v;
// }

void main()
{
    gl_Position = uWorldViewProj * vec4(aPosition, 1.0);
    Color = aColor;
    gl_PointSize = uSize;
}