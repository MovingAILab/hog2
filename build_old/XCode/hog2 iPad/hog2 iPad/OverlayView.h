//
//  OverlayView.h
//  Mathle
//
//  Created by Nathan Sturtevant on 8/28/09.
//  Copyright 2009 NS Software. All rights reserved.
//

#import <UIKit/UIKit.h>
#include <deque>
#include <vector>
#include "Graphics.h"

#ifndef OVERLAY
#define OVERLAY

//struct lineInfo {
//	Graphics::point start;
//	Graphics::point end;
//	rgbColor color;
//	float weight;
//};

@interface OverlayView : UIView {
//	Map *m;
//	Map2DHeading *h;
//	float cellSize;
//	float x1, x2, y1, y2;
//	std::vector<lineInfo> drawLines;
//	std::vector<std::pair<Graphics::rect, rgbColor>> drawRects;
//	std::vector<std::pair<Graphics::rect, rgbColor>> frameRects;
//	std::vector<std::pair<Graphics::rect, rgbColor>> drawCircle;
//	std::vector<std::pair<Graphics::rect, rgbColor>> frameCircle;
	float height, width;
	float xoffset, yoffset, xscale, yscale;
//	Graphics::Display *display;
//	bool isBackground;
	//	bool drawMap;
};

@property Graphics::Display *display;
@property bool isBackground;

//- (void)clear;
//- (bool)setLineStartX:(float)x1 StartY:(float)y1 EndX:(float)x2 EndY:(float)y2;
//- (void)clearLine;

//- (void)addLine:(Graphics::point2d)start lineEnd:(Graphics::point2d)end lineColor:(rgbColor)c;
//- (void)addLine:(Graphics::point)start lineEnd:(Graphics::point)end lineColor:(rgbColor)c lineWeight:(float)w;
//- (void)addRect:(Graphics::rect)r color:(rgbColor)c;
//- (void)addFramedRect:(Graphics::rect)r color:(rgbColor)c;
//- (void)addCircle:(Graphics::rect)r color:(rgbColor)c;
//- (void)addFramedCircle:(Graphics::rect)r color:(rgbColor)c;
- (Graphics::point)getWorldCoordinate:(Graphics::point)point;
-(point3d)convertToGlobalHogCoordinate:(CGPoint)currPoint;


@end

#endif
