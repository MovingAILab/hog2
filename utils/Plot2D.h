/*
 *  Plot2D.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/18/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef PLOT2D_H
#define PLOT2D_H

#include <vector>
//#include "Constants.h"
#include "FPUtil.h"
#include "Graphics.h"

namespace Plotting {

	enum tPlotType {
		kLinePlot, // draw from point to point
		kImpulsePlot // draw impulses at at y-axis value
	};

	class Line {
	public:
		Line(const char *label, tPlotType = kLinePlot);
		void AddPoint(double x, double y);
		void AddPoint(double x);
		void Clear();
		const char *GetLabel() { return name; }
		
		void ClearColor();
		void SetColor(double, double, double);
		void SetColor(const rgbColor &c);
		void SetWidth(float w) { width = w; }
//		void OpenGLDraw() const;
		void Draw(Graphics::Display &display, double xOff, double yOff, double xScale, double yScale) const;
		void SetHidden( bool val) { hidden = val; }
		bool IsHidden() { return hidden; }

		inline void Smooth(unsigned int windowSize);
		inline bool GetChanged() { return changedLine; }
		inline void SetChange(bool val) { changedLine = val; }
		inline double GetMaxX() { return xMax; }
		inline double GetMaxY() { return yMax; }
		inline double GetMinX() { return xMin; }
		inline double GetMinY() { return yMin; }
		
		double DistanceToLine(double xp, double yp);
		double VerticalDistanceToLine(double xp, double yp);
	private:
		bool changedLine;
		tPlotType plotType;
		double xMin, xMax, yMin, yMax;
		double r, g, b;
		std::vector<double> x;
		std::vector<double> y;
		bool hidden;
		char name[1024];
		float width;
	};

	struct Point {
		double x, y, r;
		rgbColor c;
		void Draw(Graphics::Display &display, double xOff, double yOff, double xScale, double yScale) const;
	};
	
	class Plot2D {
	public:
		Plot2D();
		void Clear();
		void AddLine(Line *);
		void AddPoint(const Point &p);
		void IncludeInX(double x);
		void IncludeInY(double y);
		void SetXAxisLabel(const char *);
		void SetYAxisLabel(const char *);
		//		void SetCurrMouse(double, double, Rect &winRect);
//		void Recenter(double, double, Rect &rectPort);
		double GetMaxX() { return dRight; }
		double GetMinX() { return dLeft; }
		double GetMaxY() { return dTop; }
		double GetMinY() { return dBottom; }
		void OffsetCurrMouse(double deltaX, double deltaY);
		void SetAxis(double, double, double, double);
		void Zoom(double);
		void ResetAxis();
//		void OpenGLDraw() const;
		void Draw(Graphics::Display &display) const;
		void SmoothLines();
		void NormalizeAxes();
	private:
		point3d MakeHOG(double x, double y) const;
		double MakeHOGWidth(double w) const;
		double mouseXLoc, mouseYLoc;
		double dLeft, dRight, dTop, dBottom;
		double xMin, xMax, yMin, yMax;
		bool forceAxis, drawMouse, recomputeBorders;
		int lastSelectedLine;
		std::vector<Point> points;
		std::vector<Line *> lines;
		std::string xLabel, yLabel;
		mutable double xOffset;
		mutable double yOffset;
		mutable double xScale;
		mutable double yScale;
};

}

#endif
