/*
 *  $Id: macGlCheck.cpp
 *  hog2
 *
 *  Created by Geoff Stahl on 01/24/03.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
	Copyright:	Copyright � 2003 Apple Computer, Inc., All Rights Reserved

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
*/
 
 // see glcheck.h for more explanation on the use of CheckOpenGLCaps and it's associated data

 
#include "MacGlCheck.h"

#include <OpenGL/OpenGL.h>
#include <AGL/agl.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>

#include <string.h>

// -------------------------

// local CF dictionary routines

static long _getDictLong (CFDictionaryRef refDict, CFStringRef key)
{
  long int_value;
  CFNumberRef num_value = (CFNumberRef)CFDictionaryGetValue(refDict, key);
  if (!num_value) // if can't get a number for the dictionary
    return -1;  // fail
  // or if cant convert it
  if (!CFNumberGetValue(num_value, kCFNumberLongType, &int_value)) 
    return -1; // fail
  return int_value; // otherwise return the long value
}

static double _getDictDouble (CFDictionaryRef refDict, CFStringRef key)
{
  double double_value;
  CFNumberRef num_value = (CFNumberRef)CFDictionaryGetValue(refDict, key);
  if (!num_value) // if can't get a number for the dictionary
    return -1;  // fail
  // or if cant convert it
  if (!CFNumberGetValue(num_value, kCFNumberDoubleType, &double_value)) 
    return -1; // fail
  return double_value; // otherwise return the long value
}

// -------------------------

// this does a reasonable check to see if things have changed without being 
// too heavy weight; returns 1 if changed 0 if not
// checks num displays, displayID, displayMask, each display geometry and 
// renderer VRAM and ID

unsigned char HaveOpenGLCapsChanged (GLCaps aDisplayCaps[], 
                                     CGDisplayCount dspyCnt)
{
  CGDisplayCount maxDisplays = 32;
  CGDirectDisplayID activeDspys[32];
  CGDisplayErr error;
  unsigned short i;
  CGDisplayCount newDspyCnt = 0;
  
  if (NULL == aDisplayCaps) return 1;

  error = CGGetActiveDisplayList(maxDisplays, activeDspys, &newDspyCnt);
  // if error getting list mark as changed
  if (error) return 1; 
  // if number of displays not equal
  if (dspyCnt != newDspyCnt) return 1;
  
  for (i = 0; i < dspyCnt; i++) {
    // get device ids
    if (aDisplayCaps[i].cgDisplayID != activeDspys[i]) return 1;
    if (aDisplayCaps[i].cglDisplayMask !=  
        CGDisplayIDToOpenGLDisplayMask(activeDspys[i])) return 1;
 
    // get current geometry
    {
      CGRect displayRect = CGDisplayBounds (activeDspys[i]);
      // get mode dictionary
      CFDictionaryRef dispMode = CGDisplayCurrentMode (activeDspys[i]);
      // check for all geometry matches 
      if (aDisplayCaps[i].deviceWidth != (long) displayRect.size.width)
        return 1;   
      if (aDisplayCaps[i].deviceHeight != (long) displayRect.size.height) 
        return 1;   
      if (aDisplayCaps[i].deviceOriginX != (long) displayRect.origin.x) 
        return 1;   
      if (aDisplayCaps[i].deviceOriginY != (long) displayRect.origin.y) 
        return 1;   
      if (aDisplayCaps[i].deviceDepth != 
                 (short) _getDictLong (dispMode,  kCGDisplayBitsPerPixel)) 
        return 1;    
      if (aDisplayCaps[i].deviceRefresh != 
          (short)(_getDictDouble (dispMode, kCGDisplayRefreshRate) + 0.5)) 
        return 1; // round to GLint
    }

    // get renderer info based on gDevice
    {
		CGLRendererInfoObj info;
		GLint rv = 0;
		GLint j, numRenderers = 0;
		CGLError err = (CGLError)0;
		GLint deviceVRAM; // video memory in bytes
		GLint rendererID; // renderer ID
      
      err = CGLQueryRendererInfo (aDisplayCaps[i].cglDisplayMask, 
                                  &info, &numRenderers);
      if (0 == err) {
        CGLDescribeRenderer (info, 0, kCGLRPRendererCount, &numRenderers);
        for (j = 0; j < numRenderers; j++) {
          // find accelerated renderer (assume only one)
          CGLDescribeRenderer (info, j, kCGLRPAccelerated, &rv); 
          if (true == rv) { // if accelerated
            // what is the renderer ID
            CGLDescribeRenderer (info, j, kCGLRPRendererID, &rendererID); 
            if (rendererID != aDisplayCaps[i].rendererID) // check match
              return 1;
            // what is the VRAM
            CGLDescribeRenderer (info, j, kCGLRPVideoMemory, &deviceVRAM); 
            if (deviceVRAM != aDisplayCaps[i].deviceVRAM) // check match
              return 1;
            break; // done
          }
        }
      }
      CGLDestroyRendererInfo (info);
    }
  }
  return 0;
}

// -------------------------

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

void CheckOpenGLCaps (CGDisplayCount maxDspys, 
                      GLCaps dCaps[], 
                      CGDisplayCount * dCnt)
{
  CGLContextObj curr_ctx = 0;
  CGDirectDisplayID dspys[32];
  CGDisplayErr err;
  unsigned short i;
  short size = sizeof (GLCaps);
  
  // no devices
  *dCnt = 0;
  
  if (maxDspys == 0) { // find number of displays
    *dCnt = 0;
    err = CGGetActiveDisplayList (32, dspys, dCnt);
    if (err) // err getting list
      *dCnt = 0; // 0 displays since can't correctly find any
    // zero list to ensure the routines are used correctly
    memset (dspys, 0, sizeof (CGDirectDisplayID) * *dCnt);
    return; // return dCnt
  }
  if (NULL == dCaps) return;

  err = CGGetActiveDisplayList(maxDspys, dspys, dCnt);
  if (err) return; // err getting list
  if (0 == *dCnt) return; // no displays
  
  memset (dCaps, 0, size * *dCnt); // zero memory
  
  for (i = 0; i < *dCnt; i++) {
    // get device ids
    dCaps[i].cgDisplayID = dspys[i];
    dCaps[i].cglDisplayMask = CGDisplayIDToOpenGLDisplayMask(dspys[i]);
    
    { // get current geometry
      CGRect displayRect = CGDisplayBounds (dspys[i]);
      // get mode dictionary
      CFDictionaryRef dispMode = CGDisplayCurrentMode (dspys[i]); 
      dCaps[i].deviceWidth = (long) displayRect.size.width;   
      dCaps[i].deviceHeight = (long) displayRect.size.height;   
      dCaps[i].deviceOriginX = (long) displayRect.origin.x;   
      dCaps[i].deviceOriginY = (long) displayRect.origin.y;   
      dCaps[i].deviceDepth = (short) _getDictLong (dispMode,  
                                              kCGDisplayBitsPerPixel);    
      dCaps[i].deviceRefresh = (short) (_getDictDouble (dispMode,
                                        kCGDisplayRefreshRate) + 0.5); 
    }

    { // find gDevice device by bounds
      GDHandle hGD;
      for (hGD = GetDeviceList (); hGD; hGD = GetNextDevice (hGD))
      {
        if (!TestDeviceAttribute (hGD, screenDevice) ||
            !TestDeviceAttribute (hGD, screenActive))
          continue;
  
        // if postion and sizes match
        if (((*hGD)->gdRect.top == dCaps[i].deviceOriginY) &&
          ((*hGD)->gdRect.left == dCaps[i].deviceOriginX) &&
          (((*hGD)->gdRect.bottom - (*hGD)->gdRect.top) ==
                                                  dCaps[i].deviceHeight) &&
          (((*hGD)->gdRect.right - (*hGD)->gdRect.left) ==
                                                  dCaps[i].deviceWidth)) {
          dCaps[i].hGDevice = hGD;
          break;
        }
      }
      if (dCaps[i].hGDevice == NULL)
        return; // err
      if (noErr != DMGetDisplayIDByGDevice (dCaps[i].hGDevice,
                                            &dCaps[i].displayID, false))
        dCaps[i].displayID = 0; // err getting display ID
    }
    
    
    { // get renderer info based on gDevice
      CGLRendererInfoObj info;
      GLint j, numRenderers = 0, rv = 0;
      err = (CGLError)0;
      
      err = CGLQueryRendererInfo (dCaps[i].cglDisplayMask, 
                                  &info, 
                                  &numRenderers);
      if (0 == err) {
        CGLDescribeRenderer (info, 0, kCGLRPRendererCount, &numRenderers);
        for (j = 0; j < numRenderers; j++) {
          // find accelerated renderer (assume only one)
          CGLDescribeRenderer (info, j, kCGLRPAccelerated, &rv); 
          if (true == rv) { // if accelerated
            // what is the renderer ID
            CGLDescribeRenderer (info, j, kCGLRPRendererID,
                                 &dCaps[i].rendererID);
            // can we do full screen?
            CGLDescribeRenderer (info, j, kCGLRPFullScreen, &rv); 
            dCaps[i].fullScreenCapable = (bool) rv;
            // what is the VRAM?
            CGLDescribeRenderer (info, j, kCGLRPVideoMemory,
                                 &dCaps[i].deviceVRAM);
            // what is the current texture memory? 
            CGLDescribeRenderer (info, j, kCGLRPTextureMemory,
                                 &dCaps[i].deviceTextureRAM);
            break; // done
          }
        }
      }
      CGLDestroyRendererInfo (info);
    }

    { // build context and context specific info
      CGLPixelFormatAttribute attribs[] = { (CGLPixelFormatAttribute)kCGLPFADisplayMask,
                                            (CGLPixelFormatAttribute)dCaps[i].cglDisplayMask, 
                                            (CGLPixelFormatAttribute)NULL };
      CGLPixelFormatObj pixelFormat = NULL;
      GLint numPixelFormats = 0;
      CGLContextObj cglContext;
      
      curr_ctx = CGLGetCurrentContext (); // get current CGL context
      CGLChoosePixelFormat (attribs, &pixelFormat, &numPixelFormats);
      if (pixelFormat) {
        CGLCreateContext(pixelFormat, NULL, &cglContext);
        CGLDestroyPixelFormat (pixelFormat);
        CGLSetCurrentContext (cglContext);
        if (cglContext) {
          const GLubyte * strExt;
          const GLubyte * strRend;
          const GLubyte * strVers;
          const GLubyte * strVend;

          // get renderer strings
          strRend = glGetString (GL_RENDERER);
          strncpy (dCaps[i].strRendererName, (const char*)strRend, 255);
          strVend = glGetString (GL_VENDOR);
          strncpy (dCaps[i].strRendererVendor, (const char*)strVend, 255);
          strVers = glGetString (GL_VERSION);
          strncpy (dCaps[i].strRendererVersion, (const char*)strVers, 255);
          { // get BCD version
            short j = 0;
            short shiftVal = 8;
            while (((strVers[j] <= '9') && (strVers[j] >= '0')) ||
                                            (strVers[j] == '.')) { 
            // get only basic version info (until first non-digit or non-.)
              if ((strVers[j] <= '9') && (strVers[j] >= '0')) {
                dCaps[i].glVersion += (strVers[j] - '0') << shiftVal;
                shiftVal -= 4;
              }
              j++;
            }
          }
          strExt = glGetString (GL_EXTENSIONS);

          // get caps
          glGetIntegerv (GL_MAX_TEXTURE_UNITS, 
                         &dCaps[i].textureUnits);
          glGetIntegerv (GL_MAX_TEXTURE_SIZE,
                         &dCaps[i].maxTextureSize); 
          glGetIntegerv (GL_MAX_3D_TEXTURE_SIZE, 
                         &dCaps[i].max3DTextureSize);
          glGetIntegerv (GL_MAX_CUBE_MAP_TEXTURE_SIZE, 
                         &dCaps[i].maxCubeMapTextureSize);

          // get functionality info
          dCaps[i].fSpecularVector = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_specular_vector", strExt);
          dCaps[i].fTransformHint = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_transform_hint", strExt);
          dCaps[i].fPackedPixels = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_packed_pixels", strExt) || 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_packed_pixel", strExt)  || 
            (dCaps[i].glVersion >= 0x0120);
          dCaps[i].fClientStorage = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_client_storage", strExt);
          dCaps[i].fYCbCr = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_ycbcr_422", strExt);
          dCaps[i].fTextureRange = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_texture_range", strExt);
          dCaps[i].fFence = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_fence", strExt);
          dCaps[i].fVAR = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_vertex_array_range", strExt);
          dCaps[i].fVAO = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_vertex_array_object", strExt);
          dCaps[i].fElementArray = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_element_array", strExt);
          dCaps[i].fVPEvals = 
            gluCheckExtension((const GLubyte*)"GL_APPLE_vertex_program_evaluators",strExt);
          dCaps[i].fFloatPixels = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_float_pixels", strExt);
          dCaps[i].fFlushRenderer = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_flush_render", strExt);
          dCaps[i].fPixelBuffer = 
            gluCheckExtension ((const GLubyte*)"GL_APPLE_pixel_buffer", strExt);
          dCaps[i].fImaging = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fTransposeMatrix = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_transpose_matrix", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fMultitexture = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_multitexture", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fTexEnvAdd = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_env_add", strExt) ||
            gluCheckExtension ((const GLubyte*)"GL_EXT_texture_env_add", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fTexEnvCombine = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_env_combine", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fTexEnvDot3 = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_env_dot3", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fTexEnvCrossbar = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_env_crossbar", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fTexCubeMap = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_cube_map", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fTexCompress = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_compression", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fMultisample = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_multisample", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fTexBorderClamp = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_border_clamp", strExt) ||
            (dCaps[i].glVersion >= 0x0130);
          dCaps[i].fPointParam = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_point_parameters", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fVertexProg = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_vertex_program", strExt);
          dCaps[i].fFragmentProg = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_fragment_program", strExt);
          dCaps[i].fTexMirrorRepeat = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_texture_mirrored_repeat", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fDepthTex = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_depth_texture", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fShadow = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_shadow", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fShadowAmbient = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_shadow_ambient", strExt);
          dCaps[i].fVertexBlend = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_vertex_blend", strExt);
          dCaps[i].fWindowPos = 
            gluCheckExtension ((const GLubyte*)"GL_ARB_window_pos", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fTex3D = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_texture3D", strExt) ||
            (dCaps[i].glVersion >= 0x0120);
          dCaps[i].fClipVolHint = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_clip_volume_hint", strExt);
          dCaps[i].fRescaleNorm = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_rescale_normal", strExt) ||
            (dCaps[i].glVersion >= 0x0120);
          dCaps[i].fBlendColor = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_blend_color", strExt) ||
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fBlendMinMax = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_blend_minmax", strExt) ||
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fBlendSub = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_blend_subtract", strExt) ||
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fCVA = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_compiled_vertex_array", strExt);
          dCaps[i].fTexLODBias = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_texture_lod_bias", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fABGR = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_abgr", strExt);
          dCaps[i].fBGRA = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_bgra", strExt) ||
            (dCaps[i].glVersion >= 0x0120);
          dCaps[i].fTexFilterAniso = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_texture_filter_anisotropic",strExt);
          dCaps[i].fPaletteTex = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_paletted_texture", strExt);
          dCaps[i].fShareTexPalette = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_shared_texture_palette", strExt);
          dCaps[i].fSecColor = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_secondary_color", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fTexCompressS3TC = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_texture_compression_s3tc", strExt);
          dCaps[i].fTexRect = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_texture_rectangle", strExt);
          dCaps[i].fFogCoord = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_fog_coord", strExt);
          dCaps[i].fDrawRangeElements = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_draw_range_elements", strExt);
          dCaps[i].fStencilWrap = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_stencil_wrap", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fBlendFuncSep = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_blend_func_separate", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fMultiDrawArrays = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_multi_draw_arrays", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fShadowFunc = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_shadow_funcs", strExt);
          dCaps[i].fStencil2Side = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_stencil_two_side", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fColorSubtable = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_color_subtable", strExt) || 
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fConvolution = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_convolution", strExt) || 
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fHistogram = 
            gluCheckExtension ((const GLubyte*)"GL_EXT_histogram", strExt) || 
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fColorTable = 
            gluCheckExtension ((const GLubyte*)"GL_SGI_color_table", strExt) || 
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fColorMatrix = 
            gluCheckExtension ((const GLubyte*)"GL_SGI_color_matrix", strExt) || 
            gluCheckExtension ((const GLubyte*)"GL_ARB_imaging", strExt);
          dCaps[i].fTexEdgeClamp = 
            gluCheckExtension ((const GLubyte*)"GL_SGIS_texture_edge_clamp", strExt) ||
            (dCaps[i].glVersion >= 0x0120);
          dCaps[i].fGenMipmap = 
            gluCheckExtension ((const GLubyte*)"GL_SGIS_generate_mipmap", strExt);
          dCaps[i].fTexLOD = 
            gluCheckExtension ((const GLubyte*)"GL_SGIS_texture_lod", strExt) ||
            (dCaps[i].glVersion >= 0x0120);
          dCaps[i].fPointCull = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_point_cull_mode", strExt);
          dCaps[i].fTexMirrorOnce = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_texture_mirror_once", strExt);
          dCaps[i].fPNtriangles = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_pn_triangles", strExt) ||
            gluCheckExtension ((const GLubyte*)"GL_ATIX_pn_triangles", strExt);
          dCaps[i].fTextFragShader = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_text_fragment_shader", strExt);
          dCaps[i].fBlendEqSep = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_blend_equation_separate", strExt);
          dCaps[i].fBlendWeightMinMax = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_blend_weighted_minmax", strExt);
          dCaps[i].fCombine3 = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_texture_env_combine3", strExt);
          dCaps[i].fSepStencil = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_separate_stencil", strExt);
          dCaps[i].fArrayRevComps4Byte = 
            gluCheckExtension ((const GLubyte*)"GL_ATI_array_rev_comps_in_4_bytes",strExt);
          dCaps[i].fPointSprite = 
            gluCheckExtension ((const GLubyte*)"GL_NV_point_sprite", strExt);
          dCaps[i].fRegCombiners = 
            gluCheckExtension ((const GLubyte*)"GL_NV_register_combiners", strExt);
          dCaps[i].fRegCombiners2 = 
            gluCheckExtension ((const GLubyte*)"GL_NV_register_combiners2", strExt);
          dCaps[i].fTexEnvCombine4 = 
            gluCheckExtension ((const GLubyte*)"GL_NV_texture_env_combine4", strExt);
          dCaps[i].fBlendSquare = 
            gluCheckExtension ((const GLubyte*)"GL_NV_blend_square", strExt) ||
            (dCaps[i].glVersion >= 0x0140);
          dCaps[i].fFogDist = 
            gluCheckExtension ((const GLubyte*)"GL_NV_fog_distance", strExt);
          dCaps[i].fMultisampleFilterHint = 
            gluCheckExtension ((const GLubyte*)"GL_NV_multisample_filter_hint", strExt);
          dCaps[i].fTexGenReflect = 
            gluCheckExtension ((const GLubyte*)"GL_NV_texgen_reflection", strExt);
          dCaps[i].fTexShader = 
            gluCheckExtension ((const GLubyte*)"GL_NV_texture_shader", strExt);
          dCaps[i].fTexShader2 = 
            gluCheckExtension ((const GLubyte*)"GL_NV_texture_shader2", strExt);
          dCaps[i].fTexShader3 = 
            gluCheckExtension ((const GLubyte*)"GL_NV_texture_shader3", strExt);
          dCaps[i].fDepthClamp = 
            gluCheckExtension ((const GLubyte*)"GL_NV_depth_clamp", strExt);
          dCaps[i].fLightMaxExp = 
            gluCheckExtension ((const GLubyte*)"GL_NV_light_max_exponent", strExt);
          dCaps[i].fRasterPosClip = 
            gluCheckExtension ((const GLubyte*)"GL_IBM_rasterpos_clip", strExt);
          dCaps[i].fConvBorderModes = 
            gluCheckExtension ((const GLubyte*)"GL_HP_convolution_border_modes", strExt) ||
            gluCheckExtension ((const GLubyte*)(const GLubyte*)"GL_ARB_imaging", strExt);

          if (dCaps[i].fTexRect) // only check if extension supported
            glGetIntegerv (GL_MAX_RECTANGLE_TEXTURE_SIZE_EXT,
                           &dCaps[i].maxRectTextureSize);
          else
            dCaps[i].maxRectTextureSize = 0;

          CGLDestroyContext (cglContext);
        }
      }
      CGLSetCurrentContext (curr_ctx); // reset current CGL context
    }
  }
}
