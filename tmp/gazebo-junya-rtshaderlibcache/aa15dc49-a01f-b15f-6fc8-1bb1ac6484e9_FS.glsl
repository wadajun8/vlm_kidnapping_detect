#version 460
//-----------------------------------------------------------------------------
// Program Type: Fragment shader
// Language: glsl
// Created by Ogre RT Shader Generator. All rights reserved.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//                         FORWARD DECLARATIONS
//-----------------------------------------------------------------------------
void FFP_Add(in vec3, in vec3, out vec3);
void FFP_Assign(in vec4, out vec4);
void FFP_Construct(in float, in float, in float, in float, out vec4);
void FFP_Modulate(in vec4, in vec4, out vec4);
void FFP_SampleTexture(in sampler2D, in vec2, out vec4);
void SGX_ApplyShadowFactor_Diffuse(in vec4, in vec4, in float, out vec4);
void SGX_ComputeShadowFactor_PSSM3(in float, in vec4, in vec4, in vec4, in vec4, in sampler2DShadow, in sampler2DShadow, in sampler2DShadow, in vec4, in vec4, in vec4, out float);
void SGX_Light_Directional_Diffuse(in vec3, in vec3, in vec3, in vec3, out vec3);
void SGX_ModulateScalar(in float, in vec4, out vec4);

//-----------------------------------------------------------------------------
//                         GLOBAL PARAMETERS
//-----------------------------------------------------------------------------

uniform	vec4	derived_ambient_light_colour;
uniform	vec4	surface_diffuse_colour;
uniform	vec4	surface_specular_colour;
uniform	vec4	surface_emissive_colour;
uniform	vec4	derived_scene_colour;
uniform	float	surface_shininess;
uniform	vec4	light_direction_view_space0;
uniform	vec4	derived_light_diffuse1;
uniform	sampler2D	gTextureSampler0;
uniform	vec4	pssm_split_points2;
uniform	sampler2DShadow	shadow_map1;
uniform	vec4	inv_shadow_texture_size3;
uniform	sampler2DShadow	shadow_map2;
uniform	vec4	inv_shadow_texture_size4;
uniform	sampler2DShadow	shadow_map3;
uniform	vec4	inv_shadow_texture_size5;

//-----------------------------------------------------------------------------
// Function Name: main
// Function Desc: Pixel Program Entry point
//-----------------------------------------------------------------------------
in	vec3	oTexcoord3_0;
in	vec2	oTexcoord2_1;
in	float	oTexcoord1_2;
in	vec4	oTexcoord4_3;
in	vec4	oTexcoord4_4;
in	vec4	oTexcoord4_5;
out vec4 fragColour;
void main(void) {
	vec4	lLocalParam_0;
	vec4	lLocalParam_1;
	vec4	lPerPixelDiffuse;
	vec4	texel_0;
	vec4	source1;
	vec4	source2;
	float	lShadowFactor;

	FFP_Construct(1.0, 1.0, 1.0, 1.0, lLocalParam_0);

	FFP_Construct(0.0, 0.0, 0.0, 0.0, lLocalParam_1);

	FFP_Assign(lLocalParam_0, fragColour);

	FFP_Assign(derived_scene_colour, lPerPixelDiffuse);

	SGX_Light_Directional_Diffuse(oTexcoord3_0, light_direction_view_space0.xyz, derived_light_diffuse1.xyz, lPerPixelDiffuse.xyz, lPerPixelDiffuse.xyz);

	FFP_Assign(lPerPixelDiffuse, lLocalParam_0);

	FFP_Assign(lLocalParam_0, fragColour);

	SGX_ComputeShadowFactor_PSSM3(oTexcoord1_2, pssm_split_points2, oTexcoord4_3, oTexcoord4_4, oTexcoord4_5, shadow_map1, shadow_map2, shadow_map3, inv_shadow_texture_size3, inv_shadow_texture_size4, inv_shadow_texture_size5, lShadowFactor);

	SGX_ApplyShadowFactor_Diffuse(derived_scene_colour, lLocalParam_0, lShadowFactor, lLocalParam_0);

	SGX_ModulateScalar(lShadowFactor, lLocalParam_1, lLocalParam_1);

	FFP_Assign(lLocalParam_0, fragColour);

	FFP_SampleTexture(gTextureSampler0, oTexcoord2_1, texel_0);

	FFP_Assign(texel_0, source1);

	FFP_Assign(lLocalParam_0, source2);

	FFP_Modulate(source1, source2, fragColour);

	FFP_Add(fragColour.xyz, lLocalParam_1.xyz, fragColour.xyz);

}

