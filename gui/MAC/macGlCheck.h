/*
 *  $Id: macGlCheck.h
 *  hog2
 *
 *  Created by Geoff Stahl on 01/24/03.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
	Copyright:	Copyright 2003 Apple Computer, Inc., All Rights Reserved

	Disclaimer:	IMPORTANT:  This Apple software is supplied to you by Apple Computer, Inc.
			("Apple") in consideration of your agreement to the following terms, and your
			use, installation, modification or redistribution of this Apple software
			constitutes acceptance of these terms.  If you do not agree with these terms,
			please do not use, install, modify or redistribute this Apple software.

			In consideration of your agreement to abide by the following terms, and subject
			to these terms, Apple grants you a personal, non-exclusive license, under Apple’s
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
 
// glcheck allows developer to check the hardware capabilities of all GPU's
// returning an array of records reflecting the attached hardware. This
// list can be regenerated on Display Manager notifications to keep the
// client update to on capabilities and setup changes.  This is provided as
// sample to allow developers the freedom to check as few or as many
// conditions and capabilities as they would like or add their own checks

#include <Carbon/Carbon.h>
#include <ApplicationServices/ApplicationServices.h>
#include "GLUtil.h"

typedef struct {
  // developers can add capabilities as required
  
  CGDirectDisplayID cgDisplayID; // CG display ID (main identifier)
  unsigned long displayID; // QD display ID
  GDHandle hGDevice; // graphics device handle
  CGOpenGLDisplayMask cglDisplayMask; // CGL display mask
  
  // current (at time of look up) device geometry
  long deviceWidth; // pixel width
  long deviceHeight; // pixel width
  long deviceOriginX; // left location of device (relative to main device)
  long deviceOriginY; // upper location of device (relative to main device)
  short deviceDepth; // pixel depth in bits
  short deviceRefresh; // integer refresh rate in Hz
  
  // Renderer info
  GLint deviceVRAM; // video memory in bytes
  GLint deviceTextureRAM; // uses current mode (geometry, pixel depth, etc.)
  GLint rendererID; // renderer ID
  char strRendererName [256]; // name of hardware renderer
  char strRendererVendor [256]; // name of hardware renderer vendor
  char strRendererVersion [256]; // string rep of hardware renderer version
  bool fullScreenCapable; // does device support full screen
  // can add more device specs as you want
  
  // Renderer Caps
  GLint textureUnits; // standard gl path max number of texture units
  GLint maxTextureSize; // maximum 1D and 2D texture size supported
  GLint max3DTextureSize; // maximum 3D texture size supported
  GLint maxCubeMapTextureSize; // maximum cube map texture size supported
  GLint maxRectTextureSize; // maximum rectangular texture size supported
  
  // OpenGL version support
  unsigned short glVersion; // bcd gl version (ie. 1.4 is 0x0140)
  
  
  // Functionality
  bool fSpecularVector; // GL_APPLE_specular_vector
  bool fTransformHint; // GL_APPLE_transform_hint 
  bool fPackedPixels; // GL_APPLE_packed_pixels or 1.2+
  bool fClientStorage; // GL_APPLE_client_storage
  bool fYCbCr; // GL_APPLE_ycbcr_422 (YUV texturing)
  bool fTextureRange; // GL_APPLE_texture_range (AGP texturing)
  bool fFence; // GL_APPLE_fence
  bool fVAR; // GL_APPLE_vertex_array_range
  bool fVAO; // GL_APPLE_vertex_array_object
  bool fElementArray; // GL_APPLE_element_array
  bool fVPEvals; // GL_APPLE_vertex_program_evaluators
  bool fFloatPixels; // GL_APPLE_float_pixels
  bool fFlushRenderer; // GL_APPLE_flush_render
  bool fPixelBuffer; // GL_APPLE_pixel_buffer
  bool fImaging; // GL_ARB_imaging  (not required in 1.2+)
  bool fTransposeMatrix; // GL_ARB_transpose_matrix or 1.3+
  bool fMultitexture; // GL_ARB_multitexture or 1.3+
  bool fTexEnvAdd; // GL_ARB_texture_env_add, GL_EXT_texture_env_add or 1.3+
  bool fTexEnvCombine; // GL_ARB_texture_env_combine or 1.3+
  bool fTexEnvDot3; // GL_ARB_texture_env_dot3 or 1.3+
  bool fTexEnvCrossbar; // GL_ARB_texture_env_crossbar or 1.4+
  bool fTexCubeMap; // GL_ARB_texture_cube_map or 1.3+
  bool fTexCompress; // GL_ARB_texture_compression or 1.3+
  bool fMultisample; // GL_ARB_multisample or 1.3+ (Anti-aliasing)
  bool fTexBorderClamp; // GL_ARB_texture_border_clamp or 1.3+
  bool fPointParam; // GL_ARB_point_parameters or 1.4+
  bool fVertexProg; // GL_ARB_vertex_program
  bool fFragmentProg; // GL_ARB_fragment_program
  bool fTexMirrorRepeat; // GL_ARB_texture_mirrored_repeat or 1.4+
  bool fDepthTex; // GL_ARB_depth_texture or 1.4+
  bool fShadow; // GL_ARB_shadow or 1.4+
  bool fShadowAmbient; // GL_ARB_shadow_ambient
  bool fVertexBlend; // GL_ARB_vertex_blend
  bool fWindowPos; // GL_ARB_window_pos or 1.4+
  bool fTex3D; // GL_EXT_texture3D or 1.2+
  bool fClipVolHint; // GL_EXT_clip_volume_hint
  bool fRescaleNorm; // GL_EXT_rescale_normal or 1.2+
  bool fBlendColor; // GL_EXT_blend_color or GL_ARB_imaging
  bool fBlendMinMax; // GL_EXT_blend_minmax or GL_ARB_imaging
  bool fBlendSub; // GL_EXT_blend_subtract or GL_ARB_imaging
  bool fCVA; // GL_EXT_compiled_vertex_array
  bool fTexLODBias; // GL_EXT_texture_lod_bias or 1.4+
  bool fABGR; // GL_EXT_abgr
  bool fBGRA; // GL_EXT_bgra or 1.2+
  bool fTexFilterAniso; // GL_EXT_texture_filter_anisotropic
  bool fPaletteTex; // GL_EXT_paletted_texture
  bool fShareTexPalette; // GL_EXT_shared_texture_palette
  bool fSecColor; // GL_EXT_secondary_color or 1.4+
  bool fTexCompressS3TC; // GL_EXT_texture_compression_s3tc
  bool fTexRect; // GL_EXT_texture_rectangle
  bool fFogCoord; // GL_EXT_fog_coord
  bool fDrawRangeElements; // GL_EXT_draw_range_elements
  bool fStencilWrap; // GL_EXT_stencil_wrap or 1.4+
  bool fBlendFuncSep; // GL_EXT_blend_func_separate or 1.4+
  bool fMultiDrawArrays; // GL_EXT_multi_draw_arrays or 1.4+
  bool fShadowFunc; // GL_EXT_shadow_funcs
  bool fStencil2Side; // GL_EXT_stencil_two_side
  bool fColorSubtable; // GL_EXT_color_subtable or GL_ARB_imaging
  bool fConvolution; // GL_EXT_convolution or GL_ARB_imaging
  bool fHistogram; // GL_EXT_histogram or GL_ARB_imaging
  bool fColorTable; // GL_SGI_color_table or GL_ARB_imaging
  bool fColorMatrix; // GL_SGI_color_matrix
  bool fTexEdgeClamp; // GL_SGIS_texture_edge_clamp or 1.2+
  bool fGenMipmap; // GL_SGIS_generate_mipmap or 1.4+
  bool fTexLOD; // GL_SGIS_texture_lod or 1.2+
  bool fPointCull; // GL_ATI_point_cull_mode
  bool fTexMirrorOnce; // GL_ATI_texture_mirror_once
  bool fPNtriangles; // GL_ATI_pn_triangles or GL_ATIX_pn_triangles
  bool fTextFragShader; // GL_ATI_text_fragment_shader
  bool fBlendEqSep; // GL_ATI_blend_equation_separate
  bool fBlendWeightMinMax; // GL_ATI_blend_weighted_minmax
  bool fCombine3; // GL_ATI_texture_env_combine3
  bool fSepStencil; // GL_ATI_separate_stencil
  bool fArrayRevComps4Byte; // GL_ATI_array_rev_comps_in_4_bytes
  bool fPointSprite; // GL_NV_point_sprite
  bool fRegCombiners; // GL_NV_register_combiners
  bool fRegCombiners2; // GL_NV_register_combiners2
  bool fTexEnvCombine4; // GL_NV_texture_env_combine4
  bool fBlendSquare; // GL_NV_blend_square or 1.4+
  bool fFogDist; // GL_NV_fog_distance
  bool fMultisampleFilterHint; // GL_NV_multisample_filter_hint
  bool fTexGenReflect; // GL_NV_texgen_reflection
  bool fTexShader; // GL_NV_texture_shader
  bool fTexShader2; // GL_NV_texture_shader2
  bool fTexShader3; // GL_NV_texture_shader3
  bool fDepthClamp; // GL_NV_depth_clamp
  bool fLightMaxExp; // GL_NV_light_max_exponent
  bool fConvBorderModes; // GL_HP_convolution_border_modes or GL_ARB_imaging
  bool fRasterPosClip; // GL_IBM_rasterpos_clip
} GLCaps;

// this does a reasonable check to see if things have changed without being 
// too heavy weight; returns 1 if changed 0 if not
// checks num displays, displayID, displayMask, each display geometry and 
// renderer VRAM and ID
unsigned char HaveOpenGLCapsChanged (GLCaps aDisplayCaps[], 
                                     CGDisplayCount dspyCnt);


// This will walk all active displays and gather information about their
// hardware renderer 

// An array length (maxDisplays) and array of GLCaps are passed in. Up to
// maxDisplays of the array are filled in with the displays meeting the
// specified criteria.  The actual number of displays filled in is returned
// in dspyCnt.  Calling this function with maxDisplays of 0 will just
// return the number of displays in dspyCnt.

// Developers should note this is NOT an exhaustive list of all the
// capabilities one could query, nor a required set of capabilities,
// feel free to add or subtract queries as you find helpful for your 
// application/use.

// one note on mirrored displays... if the display configuration is 
// changed it is possible (and likely) that the current active display
// in a mirrored configuration (as identified by the OpenGL Display Mask)
// will change if the mirrored display is removed.  
// This is due to the preference of selection the external display as 
// the active display.  This may affect full screen apps which should 
// always detect display configuration changes and respond accordingly.

void CheckOpenGLCaps (CGDisplayCount maxDisplays, 
                      GLCaps * aDisplayCaps, 
                      CGDisplayCount * dspyCnt);

