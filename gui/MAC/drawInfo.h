/*
 *  $Id: drawInfo.h
 *  hog2
 *
 *  Created by Geoff Stahl on 03/12/03.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 *	This file is a source include that contains a pile of capability printing code
 *
 *  Copyright (c) 2003 Apple Computer Inc. All rights reserved.

	Disclaimer:	IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
			("Apple") in consideration of your agreement to the following terms, and your
			use, installation, modification or redistribution of this Apple software
			constitutes acceptance of these terms.  If you do not agree with these terms,
			please do not use, install, modify or redistribute this Apple software.

			In consideration of your agreement to abide by the following terms, and subject
			to these terms, Apple grants you a personal, non-exclusive license, under AppleÕs
			copyrights in this original Apple software (the "Apple Software"), to use,
			reproduce, modify and redistribute the Apple Software, with or without
			modifications, in source and/or binary forms; provided that if you redistribute
			the Apple Software in its entirety and without modifications, you must retain
			this notice and the following text and disclaimers in all such redistributions of
			the Apple Software.  Neither the name, trademarks, service marks or logos of
			Apple Computer, Inc. may be used to endorse or promote products derived from the
			Apple Software without specific prior written permission from Apple.  Except as
			expressly stated in this notice, no other rights or licenses, express or implied,
			are granted by Apple herein, including but not limited to any patent rights that
			may be infringed by your derivative works or by other works in which the Apple
			Software may be incorporated.

			The Apple Software is provided by Apple on an "AS IS" basis.  APPLE MAKES NO
			WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION THE IMPLIED
			WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR
			PURPOSE, REGARDING THE APPLE SOFTWARE OR ITS USE AND OPERATION ALONE OR IN
			COMBINATION WITH YOUR PRODUCTS.

			IN NO EVENT SHALL APPLE BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR
			CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
			GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
			ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION
			OF THE APPLE SOFTWARE, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT
			(INCLUDING NEGLIGENCE), STRICT LIABILITY OR OTHERWISE, EVEN IF APPLE HAS BEEN
			ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

// draw caps for this renderer
// note: this bitmap technique is not the speediest and one should use textures for fonts in more proformance critical code
static void drawCaps (pRecContext )
{ // we are already in an orthographic per pixel projection
//	unsigned short i;
//	short line = 1, lineStep = 12, leftEdge = -410;
//	char cstr [256];
//	short index = -1;
//	// match renderer, it is possible to get the wrong info if there are PCI and AGP cards with the same renderer ID, unlikely but possible
//	GLint renderer = 0;
//	// find current renderer to get current capabilities
//	AGLPixelFormat pf = pContextInfo->aglPixFmt; // first PF
//	GLint pfVS = 0, currVS = aglGetVirtualScreen (pContextInfo->aglContext); // current VS
//	aglDescribePixelFormat (pf, AGL_VIRTUAL_SCREEN, &pfVS); // get VS for PF
//	while ((pfVS != currVS) && pf) { // while we have PF's and the VS's do not match
//		pf = aglNextPixelFormat (pf); // get next PF
//		aglDescribePixelFormat (pf, AGL_VIRTUAL_SCREEN, &pfVS); // get VS for PF
//	}
//	if (pf) // if we matched VS (we should)
//		aglDescribePixelFormat(pf, AGL_RENDERER_ID, &renderer); // for matched VS
//	for (i = 0; i < gNumDisplays; i++) {
//		if (renderer == (GLint)gDisplayCaps[i].rendererID)
//			index = i;
//	}
//	// draw info
//	if (index == -1) // could not find current display in list
//		return;
//	glColor3f (1.0, 1.0, 1.0);
//	sprintf (cstr, "OpenGL Capabilities:");
//	glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//	drawCStringGL (cstr, pContextInfo->boldFontList);
//
//	glColor3f (0.9, 0.9, 0.9);
//
//	sprintf (cstr, "Max VRAM- %ld MB (%ld MB free)", 
//			 gDisplayCaps[index].deviceVRAM / 1024 / 1024, gDisplayCaps[index].deviceTextureRAM / 1024 / 1024);
//	glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//	drawCStringGL (cstr, pContextInfo->regFontList);
//
//	sprintf (cstr, "Max Texture Size- 1D/2D: %ld, 3D: %ld, Cube: %ld, Rect: %ld (%ld texture units)", 
//			 gDisplayCaps[index].maxTextureSize, gDisplayCaps[index].max3DTextureSize, 
//			 gDisplayCaps[index].maxCubeMapTextureSize, gDisplayCaps[index].maxRectTextureSize, 
//			 gDisplayCaps[index].textureUnits);
//	glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//	drawCStringGL (cstr, pContextInfo->regFontList); 
//
//	sprintf (cstr, "Features:");
//	glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//	drawCStringGL (cstr, pContextInfo->boldFontList);
//	if (gDisplayCaps[index].fSpecularVector) {
//		sprintf (cstr, "Specular Vector (GL_APPLE_specular_vector)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTransformHint) {
//		sprintf (cstr, "Transform Hint (GL_APPLE_transform_hint)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPackedPixels) {
//		sprintf (cstr, "Packed Pixels (GL_APPLE_packed_pixels or OpenGL 1.2+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fClientStorage) {
//		sprintf (cstr, "Client Storage (GL_APPLE_client_storage)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fYCbCr) {
//		sprintf (cstr, "YCbCr Textures (GL_APPLE_ycbcr_422)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTextureRange) {
//		sprintf (cstr, "Texture Range (AGP Texturing) (GL_APPLE_texture_range)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fFence) {
//		sprintf (cstr, "Fence (GL_APPLE_fence)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fVAR) {
//		sprintf (cstr, "Vertex Array Range (GL_APPLE_vertex_array_range)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fVAO) {
//		sprintf (cstr, "Vertex Array Object (GL_APPLE_vertex_array_object)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fElementArray) {
//		sprintf (cstr, "Element Array (GL_APPLE_element_array)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fVPEvals) {
//		sprintf (cstr, "Vertex Program Evaluators (GL_APPLE_vertex_program_evaluators)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fFloatPixels) {
//		sprintf (cstr, "Floating Point Pixels (GL_APPLE_float_pixels)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fFlushRenderer) {
//		sprintf (cstr, "Flush Renderer (GL_APPLE_flush_render)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPixelBuffer) {
//		sprintf (cstr, "Pixel Buffers (GL_APPLE_pixel_buffer)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fImaging) {
//		sprintf (cstr, "Imaging Subset (GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTransposeMatrix) {
//		sprintf (cstr, "Transpose Matrix (GL_ARB_transpose_matrix or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fMultitexture) {
//		sprintf (cstr, "Multitexture (GL_ARB_multitexture or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexEnvAdd) {
//		sprintf (cstr, "Texture Env Add (GL_ARB_texture_env_add, GL_EXT_texture_env_add or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexEnvCombine) {
//		sprintf (cstr, "Texture Env Combine (GL_ARB_texture_env_combine or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexEnvDot3) {
//		sprintf (cstr, "Texture Env Dot3 (GL_ARB_texture_env_dot3 or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexEnvCrossbar) {
//		sprintf (cstr, "Texture Env Crossbar (GL_ARB_texture_env_crossbar or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexCubeMap) {
//		sprintf (cstr, "Texture Env Cube Map (GL_ARB_texture_cube_map or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexCompress) {
//		sprintf (cstr, "Texture Compression (GL_ARB_texture_compression or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fMultisample) {
//		sprintf (cstr, "Multisample (Anti-aliasing) (GL_ARB_multisample or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexBorderClamp) {
//		sprintf (cstr, "Texture Border Clamp (GL_ARB_texture_border_clamp or OpenGL 1.3+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPointParam) {
//		sprintf (cstr, "Point Parameters (GL_ARB_point_parameters or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPointParam) {
//		sprintf (cstr, "Point Parameters (GL_ARB_point_parameters or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fVertexProg) {
//		sprintf (cstr, "Vertex Program (GL_ARB_vertex_program)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fFragmentProg) {
//		sprintf (cstr, "Fragment Program (GL_ARB_fragment_program)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexMirrorRepeat) {
//		sprintf (cstr, "Texture Mirrored Repeat (GL_ARB_texture_mirrored_repeat or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fDepthTex) {
//		sprintf (cstr, "Depth Texture (GL_ARB_depth_texture or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fShadow) {
//		sprintf (cstr, "Shadow Support (GL_ARB_shadow or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fShadowAmbient) {
//		sprintf (cstr, "Shadow Ambient (GL_ARB_shadow_ambient)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fVertexBlend) {
//		sprintf (cstr, "Vertex Blend (GL_ARB_vertex_blend)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fWindowPos) {
//		sprintf (cstr, "Window Position (GL_ARB_window_pos or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTex3D) {
//		sprintf (cstr, "3D Texturing (GL_EXT_texture3D or OpenGL 1.2+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fClipVolHint) {
//		sprintf (cstr, "Clip Volume Hint (GL_EXT_clip_volume_hint)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fRescaleNorm) {
//		sprintf (cstr, "Rescale Normal (GL_EXT_rescale_normal or OpenGL 1.2+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendColor) {
//		sprintf (cstr, "Blend Color (GL_EXT_blend_color or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendMinMax) {
//		sprintf (cstr, "Blend Min/Max (GL_EXT_blend_minmax or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendSub) {
//		sprintf (cstr, "Blend Subtract (GL_EXT_blend_subtract or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fCVA) {
//		sprintf (cstr, "Compiled Vertex Array (GL_EXT_compiled_vertex_array)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexLODBias) {
//		sprintf (cstr, "Texture Level Of Detail Bias (GL_EXT_texture_lod_bias or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fABGR) {
//		sprintf (cstr, "ABGR Texture Support (GL_EXT_abgr)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBGRA) {
//		sprintf (cstr, "BGRA Texture Support (GL_EXT_bgra or OpenGL 1.2+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexFilterAniso) {
//		sprintf (cstr, "Anisotropic Texture Filtering (GL_EXT_texture_filter_anisotropic)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPaletteTex) {
//		sprintf (cstr, "Paletted Textures (GL_EXT_paletted_texture)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fShareTexPalette) {
//		sprintf (cstr, "Shared Texture Palette (GL_EXT_shared_texture_palette)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fSecColor) {
//		sprintf (cstr, "Secondary Color (GL_EXT_secondary_color or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexCompressS3TC) {
//		sprintf (cstr, "Texture Compression S3TC (GL_EXT_texture_compression_s3tc)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexRect) {
//		sprintf (cstr, "Texture Rectangle (GL_EXT_texture_rectangle)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fFogCoord) {
//		sprintf (cstr, "Fog Coordinate (GL_EXT_fog_coord)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fDrawRangeElements) {
//		sprintf (cstr, "Draw Range Elements (GL_EXT_draw_range_elements)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fStencilWrap) {
//		sprintf (cstr, "Stencil Wrap (GL_EXT_stencil_wrap or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendFuncSep) {
//		sprintf (cstr, "Separate Blend Function (GL_EXT_blend_func_separate or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fMultiDrawArrays) {
//		sprintf (cstr, "Multi-Draw Arrays (GL_EXT_multi_draw_arrays or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fShadowFunc) {
//		sprintf (cstr, "Shadow Function (GL_EXT_shadow_funcs)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fStencil2Side) {
//		sprintf (cstr, "2-Sided Stencil (GL_EXT_stencil_two_side)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fColorSubtable) {
//		sprintf (cstr, "Color Subtable ( GL_EXT_color_subtable or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fConvolution) {
//		sprintf (cstr, "Convolution ( GL_EXT_convolution or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fHistogram) {
//		sprintf (cstr, "Histogram ( GL_EXT_histogram or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fColorTable) {
//		sprintf (cstr, "Color Table ( GL_SGI_color_table or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fColorMatrix) {
//		sprintf (cstr, "Color Matrix ( GL_SGI_color_matrix or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexEdgeClamp) {
//		sprintf (cstr, "Texture Edge Clamp (GL_SGIS_texture_edge_clamp or OpenGL 1.2+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexLOD) {
//		sprintf (cstr, "Texture Level Of Detail (GL_SGIS_texture_lod or OpenGL 1.2+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPointCull) {
//		sprintf (cstr, "Point Culling (GL_ATI_point_cull_mode)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexMirrorOnce) {
//		sprintf (cstr, "Texture Mirror Once (GL_ATI_texture_mirror_once)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPNtriangles) {
//		sprintf (cstr, "PN Triangles (GL_ATI_pn_triangles or GL_ATIX_pn_triangles)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTextFragShader) {
//		sprintf (cstr, "Text Fragment Shader (GL_ATI_text_fragment_shader)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendEqSep) {
//		sprintf (cstr, "Separate Blend Equations (GL_ATI_blend_equation_separate)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendWeightMinMax) {
//		sprintf (cstr, "Blend Weighted Min/Max (GL_ATI_blend_weighted_minmax)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fCombine3) {
//		sprintf (cstr, "Texture Env Combine 3 (GL_ATI_texture_env_combine3)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fSepStencil) {
//		sprintf (cstr, "Separate Stencil (GL_ATI_separate_stencil)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fArrayRevComps4Byte) {
//		sprintf (cstr, "Reverse 4 Byte Array Components (GL_ATI_array_rev_comps_in_4_bytes)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fPointSprite) {
//		sprintf (cstr, "Point Sprites (GL_NV_point_sprite)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fRegCombiners) {
//		sprintf (cstr, "Register Combiners (GL_NV_register_combiners)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fRegCombiners2) {
//		sprintf (cstr, "Register Combiners 2 (GL_NV_register_combiners2)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexEnvCombine4) {
//		sprintf (cstr, "Texture Env Combine 4 (GL_NV_texture_env_combine4)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fBlendSquare) {
//		sprintf (cstr, "Blend Square (GL_NV_blend_square or OpenGL 1.4+)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fFogDist) {
//		sprintf (cstr, "Eye Radial Fog Distance (GL_NV_fog_distance)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fMultisampleFilterHint) {
//		sprintf (cstr, "Multi-Sample Filter Hint (GL_NV_multisample_filter_hint)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexGenReflect) {
//		sprintf (cstr, "TexGen Reflection (GL_NV_texgen_reflection)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexShader) {
//		sprintf (cstr, "Texture Shader (GL_NV_texture_shader)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexShader2) {
//		sprintf (cstr, "Texture Shader 2 (GL_NV_texture_shader2)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fTexShader3) {
//		sprintf (cstr, "Texture Shader 3 (GL_NV_texture_shader3)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fDepthClamp) {
//		sprintf (cstr, "Depth Clamp (GL_NV_depth_clamp)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fLightMaxExp) {
//		sprintf (cstr, "Light Max Exponent (GL_NV_light_max_exponent)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fConvBorderModes) {
//		sprintf (cstr, "Convolution Border Modes (GL_HP_convolution_border_modes or GL_ARB_imaging)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
//	if (gDisplayCaps[index].fRasterPosClip) {
//		sprintf (cstr, "Raster Position Clipping (GL_IBM_rasterpos_clip)");
//		glRasterPos3d (pContextInfo->camera.viewWidth + leftEdge, 0 + lineStep * line++ , 0); 
//		drawCStringGL (cstr, pContextInfo->regFontList);
//	}
}
