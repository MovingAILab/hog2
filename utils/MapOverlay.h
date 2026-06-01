/*
 *  mapOverlay.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/21/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include <vector>
#include "Map.h"
#include <float.h>
#include <string>
#include <unordered_map>
#include "Graphics.h"

#ifndef MAPOVERLAY_H
#define MAPOVERLAY_H

class MapOverlay {
public:
	MapOverlay(Map *m);
	Map *GetMap() { return m; }
	void Clear();
	
	void SetOverlayValue(int x, int y, double value);
	double GetOverlayValue(int x, int y);
//	void OpenGLDraw() const;
	std::string SVGDraw() const;
	void Draw(Graphics::Display &display) const;
	
	void SetTransparentValue(double v) { ignoreVal = v; }
	rgbColor GetValueColor(double value) const;
	void SetColorMap(int val) { colorMap = val; }
	int GetColorMap() { return colorMap; }
	void SetColor(int value, rgbColor c) { colors[value] = c; }
	void IncreaseColorMap() { colorMap++; }
	void DecreaseColorMap() { colorMap--; }
	double GetMaxValue() { return maxVal; }
	double GetMinValue() { return minVal; }

	static const int customColorMap = -1;
private:
	void resetValues();
	Map *m;
	std::vector<double> values;
	std::unordered_map<int, rgbColor> colors;
	double maxVal, minVal;
	double ignoreVal;
	int colorMap;
	bool drawBorders;
//	mutable GLuint displayList;
};

#endif
