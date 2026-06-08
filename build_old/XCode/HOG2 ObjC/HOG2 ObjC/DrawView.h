//
//  DrawView.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 8/1/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#include <vector>
#include <unordered_map>
#include "Graphics.h"


@interface DrawView : NSView {
	float height, width;
	float xoffset, yoffset, xscale, yscale;
}

@property Graphics::Display *display;

// Convert from screen coordinate to HOG coordinates
-(point3d)convertToGlobalHogCoordinate:(NSPoint)currPoint;
-(int)getViewport:(NSPoint)currPoint;

@end
