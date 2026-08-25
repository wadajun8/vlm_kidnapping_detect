#version 460
//-----------------------------------------------------------------------------
// Program Type: Vertex shader
// Language: glsl
// Created by Ogre RT Shader Generator. All rights reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//                         FORWARD DECLARATIONS
//-----------------------------------------------------------------------------
void FFP_Assign(in float, out float);
void FFP_Assign(in vec2, out vec2);
void FFP_Construct(in float, in float, in float, in float, out vec4);
void FFP_Transform(in mat4, in vec4, out vec4);
void SGX_TransformNormal(in mat4, in vec3, out vec3);

//-----------------------------------------------------------------------------
//                         GLOBAL PARAMETERS
//-----------------------------------------------------------------------------

uniform	mat4	worldviewproj_matrix;
uniform	mat4	inverse_transpose_worldview_matrix;
uniform	mat4	world_texture_view_proj0;
uniform	mat4	world_texture_view_proj1;
uniform	mat4	world_texture_view_proj2;

//-----------------------------------------------------------------------------
// Function Name: main
// Function Desc: Vertex Program Entry point
//-----------------------------------------------------------------------------
in	vec4	vertex;
in	vec3	normal;
in	vec4	uv0;
out	vec3	oTexcoord3_0;
out	vec2	oTexcoord2_1;
out	float	oTexcoord1_2;
out	vec4	oTexcoord4_3;
out	vec4	oTexcoord4_4;
out	vec4	oTexcoord4_5;
void main(void) {
	vec4	lLocalParam_0;
	vec4	lLocalParam_1;

	FFP_Transform(worldviewproj_matrix, vertex, gl_Position);

	FFP_Construct(1.0, 1.0, 1.0, 1.0, lLocalParam_0);

	FFP_Construct(0.0, 0.0, 0.0, 0.0, lLocalParam_1);

	SGX_TransformNormal(inverse_transpose_worldview_matrix, normal, oTexcoord3_0);

	FFP_Assign(uv0.xy, oTexcoord2_1);

	FFP_Assign(gl_Position.z, oTexcoord1_2);

	FFP_Transform(world_texture_view_proj0, vertex, oTexcoord4_3);

	FFP_Transform(world_texture_view_proj1, vertex, oTexcoord4_4);

	FFP_Transform(world_texture_view_proj2, vertex, oTexcoord4_5);

}

