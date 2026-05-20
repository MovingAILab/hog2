#include <math.h>
#include <emscripten.h>
#include <stdio.h>
#include <vector>
#include "Common.h"
#include "SVGUtil.h"


int canvasWidth = 640;
int canvasHeight = 640;
float canvasScale = 0;
int canvasXOffset = 0;
int canvasYOffset = 0;

const char *HOGDoFrame();
void HOGDoMouse(int x, int y, bool up, bool down, bool drag);
void HOGInit(const char *arguments);
void HOGHitKey(const char *key);
int hog_main(int argc, char **argv);
const char *getTextBuffer();
void DrawToCanvas(const Graphics::Display &disp);

extern "C" {

void EMSCRIPTEN_KEEPALIVE SetCanvasSize(int x, int y)
{
	canvasWidth = x;
	canvasHeight = y;
	
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->display.windowHeight = canvasHeight;
	pContextInfo->display.windowWidth = canvasWidth;
	canvasScale = std::min(canvasWidth, canvasHeight);
	canvasXOffset = (canvasWidth>canvasHeight)?(canvasWidth-canvasHeight):0;
	canvasYOffset = (canvasHeight>canvasWidth)?(canvasHeight-canvasWidth):0;
	canvasXOffset /= 2;
	canvasYOffset /= 2;
	pContextInfo->display.ForceBackgroundRedraw();
}

void EMSCRIPTEN_KEEPALIVE MouseEvent(int x, int y, bool up, bool down, bool drag)
{
	HOGDoMouse(x, y, up, down, drag);
}

const char * EMSCRIPTEN_KEEPALIVE DoFrame()
{
	return HOGDoFrame();
}

void EMSCRIPTEN_KEEPALIVE InitHOGArg(const char *data)
{
	HOGInit(data);
	SetCanvasSize(canvasWidth, canvasHeight);
}

void EMSCRIPTEN_KEEPALIVE InitHOG()
{
	HOGInit(0);
	SetCanvasSize(canvasWidth, canvasHeight);
}

void EMSCRIPTEN_KEEPALIVE HitKey(const char *key)
{
	HOGHitKey(key);
}

}

const char *HOGDoFrame()
{
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->display.StartFrame();
	for (int x = 0; x < pContextInfo->display.numViewports; x++)
	{
		pContextInfo->display.SetViewport(x);
		HandleFrame(pContextInfo, x);
	}
	//	if (getTextBuffer() != 0)
	//		pContextInfo->display.DrawText(getTextBuffer(), {0,-0.95}, Colors::gray, 0.04);
	static std::string result;
	pContextInfo->display.EndFrame();
	DrawToCanvas(pContextInfo->display);
	return "";
}

point3d CanvasToPoint(int x, int y)
{
	x -= canvasXOffset;
	y -= canvasYOffset;
	return {static_cast<float>(2.0*x/canvasScale-1.0), static_cast<float>(2.0*y/canvasScale-1.0), 0};
}

float WidthToCanvas(float p, int viewport)
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input1(p, 0.f, 0.f);
	point3d input2(0, 0.f, 0.f);
	point3d result1 = ViewportToGlobalHOG(pContextInfo, pContextInfo->display.viewports[viewport], input1);
	point3d result2 = ViewportToGlobalHOG(pContextInfo, pContextInfo->display.viewports[viewport], input2);
	//	if (v == 1)
	//printf("X:%f -> %f\n", x, ((result.x+1.0))/2.0);
	return ((result1.x+1.0)*canvasWidth)/2.0-((result2.x+1.0)*canvasWidth)/2.0;
}

float PointXToCanvas(float p, int viewport)
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(p, 0.f, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->display.viewports[viewport], input);
	//	if (v == 1)
	//printf("X:%f -> %f\n", x, ((result.x+1.0))/2.0);
	return ((result.x+1.0)*canvasWidth)/2.0;

	// old code
	//	return ((p+1)*canvasScale/2.0)+canvasXOffset;
}

float PointYToCanvas(float p, int viewport)
{
	pRecContext pContextInfo = getCurrentContext();
	point3d input(0.f, p, 0.f);
	point3d result = ViewportToGlobalHOG(pContextInfo, pContextInfo->display.viewports[viewport], input);
	return ((result.y+1.0)*canvasHeight)/2.0;

	// old code
	//	return ((p+1)*canvasScale/2.0)+canvasYOffset;
}

point3d ConvertCanvasToGlobalHOG(int x, int y)
{
	point3d p;
	p.x = 2.0*x/canvasWidth-1.0;
	p.y = 2.0*y/canvasHeight-1.0;
	p.z = 0;
	return p;

}

void PointToCanvas(Graphics::point &p, int viewport)
{
	p.x = PointXToCanvas(p.x, viewport);
	p.y = PointYToCanvas(p.y, viewport);
	//	p.x = ((p.x+1)*canvasScale/2.0)+canvasXOffset;
	//	p.y = ((p.y+1)*canvasScale/2.0)+canvasYOffset;
}

void DoDrawCommand(const Graphics::Display::data &d, const char *which)
{
	switch (d.what)
	{
		case Graphics::Display::kLine:
		{
			const Graphics::Display::lineInfo &o = d.line; //disp.lines[x];
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;

			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.lineCap = "round";
				ctx.beginPath();
				ctx.moveTo($4, $5);
				ctx.lineTo($6, $7);
				ctx.lineWidth=$8;
				ctx.stroke();
			}, which, r,g,b,
					PointXToCanvas(o.start.x, d.viewport), PointYToCanvas(o.start.y, d.viewport),
					PointXToCanvas(o.end.x, d.viewport), PointYToCanvas(o.end.y, d.viewport),
					WidthToCanvas(o.width, d.viewport));
			break;
		}
		case Graphics::Display::kFillTriangle:
		{
			const Graphics::Display::triangleInfo &o = d.triangle;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;

			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				ctx.moveTo($4, $5);
				ctx.lineTo($6, $7);
				ctx.lineTo($8, $9);
				ctx.fill();
			}, which, r,g,b,
					PointXToCanvas(o.p1.x, d.viewport), PointYToCanvas(o.p1.y, d.viewport),
					PointXToCanvas(o.p2.x, d.viewport), PointYToCanvas(o.p2.y, d.viewport),
					PointXToCanvas(o.p3.x, d.viewport), PointYToCanvas(o.p3.y, d.viewport));
			break;
		}
		case Graphics::Display::kFrameTriangle:
		{
			const Graphics::Display::triangleInfo &o = d.triangle;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;

			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				ctx.moveTo($4, $5);
				ctx.lineTo($6, $7);
				ctx.lineTo($8, $9);
				ctx.lineWidth=$10;
				ctx.stroke();
			}, which, r,g,b,
					PointXToCanvas(o.p1.x, d.viewport), PointYToCanvas(o.p1.y, d.viewport),
					PointXToCanvas(o.p2.x, d.viewport), PointYToCanvas(o.p2.y, d.viewport),
					PointXToCanvas(o.p3.x, d.viewport), PointYToCanvas(o.p3.y, d.viewport),
					WidthToCanvas(o.width, d.viewport));
			break;
		}
		case Graphics::Display::kFillRoundedRectangle:
		{
			const Graphics::Display::rrInfo &o = d.rr;;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cWidth = ceilf(PointXToCanvas(o.r.r.right, d.viewport))-floorf(PointXToCanvas(o.r.r.left, d.viewport));
			float cHeight = ceilf(PointYToCanvas(o.r.r.bottom, d.viewport))-floorf(PointYToCanvas(o.r.r.top, d.viewport));
//			float cWidth = (PointXToCanvas(o.r.right, d.viewport))-(PointXToCanvas(o.r.left, d.viewport));
//			float cHeight = (PointYToCanvas(o.r.bottom, d.viewport))-(PointYToCanvas(o.r.top, d.viewport));
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.beginPath();
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.roundRect($4,$5,$6,$7,$8);
				ctx.fill();
			}, which, r,g,b,
					floorf(PointXToCanvas(o.r.r.left, d.viewport)), floorf(PointYToCanvas(o.r.r.top, d.viewport)),
					cWidth, cHeight, WidthToCanvas(o.r.rad, d.viewport));
			break;
		}
		case Graphics::Display::kFrameRoundedRectangle:
		{
			const Graphics::Display::rrInfo &o = d.rr;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cWidth = PointXToCanvas(o.r.r.right, d.viewport)-PointXToCanvas(o.r.r.left, d.viewport);
			float cHeight = PointYToCanvas(o.r.r.bottom, d.viewport)-PointYToCanvas(o.r.r.top, d.viewport);
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.beginPath();
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.lineWidth = $8;
				ctx.roundRect($4,$5,$6,$7,$9);
				ctx.stroke();
				//ctx.lineWidth = 1;
			}, which, r,g,b,
					PointXToCanvas(o.r.r.left, d.viewport), PointYToCanvas(o.r.r.top, d.viewport),
					cWidth, cHeight, WidthToCanvas(o.width, d.viewport), WidthToCanvas(o.r.rad, d.viewport));
			break;
		}
		case Graphics::Display::kFillRectangle:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cWidth = ceilf(PointXToCanvas(o.r.right, d.viewport))-floorf(PointXToCanvas(o.r.left, d.viewport));
			float cHeight = ceilf(PointYToCanvas(o.r.bottom, d.viewport))-floorf(PointYToCanvas(o.r.top, d.viewport));
//			float cWidth = (PointXToCanvas(o.r.right, d.viewport))-(PointXToCanvas(o.r.left, d.viewport));
//			float cHeight = (PointYToCanvas(o.r.bottom, d.viewport))-(PointYToCanvas(o.r.top, d.viewport));
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.fillRect($4,$5,$6,$7);
			}, which, r,g,b,
					floorf(PointXToCanvas(o.r.left, d.viewport)), floorf(PointYToCanvas(o.r.top, d.viewport)),
					cWidth, cHeight);
			break;
		}
		case Graphics::Display::kFrameRectangle:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cWidth = PointXToCanvas(o.r.right, d.viewport)-PointXToCanvas(o.r.left, d.viewport);
			float cHeight = PointYToCanvas(o.r.bottom, d.viewport)-PointYToCanvas(o.r.top, d.viewport);
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.lineWidth = $8;
				ctx.strokeRect($4,$5,$6,$7);
				//ctx.lineWidth = 1;
			}, which, r,g,b,
					PointXToCanvas(o.r.left, d.viewport), PointYToCanvas(o.r.top, d.viewport),
					cWidth, cHeight, WidthToCanvas(o.width, d.viewport));
			break;
		}
		case Graphics::Display::kFillOval:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cx = (PointXToCanvas(o.r.right, d.viewport)+PointXToCanvas(o.r.left, d.viewport))/2.0;
			float cy = (PointYToCanvas(o.r.top, d.viewport)+PointYToCanvas(o.r.bottom, d.viewport))/2.0;
			float rad = (PointXToCanvas(o.r.right, d.viewport)-PointXToCanvas(o.r.left, d.viewport))/2.0;
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				ctx.arc($4,$5,$6,0,2*Math.PI);
				ctx.fill();
			}, which, r,g,b,
					cx,
					cy,
					rad);
			break;
		}
		case Graphics::Display::kFrameOval:
		{
			const Graphics::Display::drawInfo &o = d.shape;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cx = (o.r.left+o.r.right)/2.0;
			float cy = (o.r.top+o.r.bottom)/2.0;
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				ctx.strokeStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.lineWidth = $7;
				ctx.beginPath();
				ctx.arc($4,$5,$6,0,2*Math.PI);
				ctx.stroke();
			}, which, r,g,b,
					PointXToCanvas(cx, d.viewport),
					PointYToCanvas(cy, d.viewport),
					(PointXToCanvas(o.r.right, d.viewport)-PointXToCanvas(o.r.left, d.viewport))/2.0,
					WidthToCanvas(o.width, d.viewport));
			break;
		}
		case Graphics::Display::kFillNGon:
		{
			const Graphics::Display::shapeInfo &o = d.polygon;
			int r = o.c.r*255.0;
			int g = o.c.g*255.0;
			int b = o.c.b*255.0;
			float cx = PointXToCanvas(o.center.x, d.viewport);
			float cy = PointYToCanvas(o.center.y, d.viewport);
			float radius = fabs(PointXToCanvas(o.radius, d.viewport)-PointXToCanvas(0, d.viewport));
			EM_ASM_({
				var c=document.getElementById(UTF8ToString($0));
				var ctx=c.getContext("2d");
				var resolution = 6.283185307/$8;

				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.beginPath();
				for (var x = 0; x <= $8; x++)
				{
					var nextx = $4+Math.sin(resolution*x+$7*0.01745329252)*$6;
					var nexty = $5+Math.cos(resolution*x+$7*0.01745329252)*$6;

					if (x == 0)
						ctx.moveTo(nextx, nexty);
					else
						ctx.lineTo(nextx, nexty);
				}
				ctx.fill();
			}, which, r,g,b, cx, cy, radius, o.rotate, o.segments);

		}
	}
}

void DrawToCanvas(const Graphics::Display &disp)
{
	if (disp.BackgroundNeedsRedraw())
	{
		//printf("Drawing background\n");
		EM_ASM({
			var c=document.getElementById("bg");
			var ctx=c.getContext("2d");
			//ctx.clearRect(0, 0, c.width, c.height);
			ctx.fillStyle = "rgb(0,0,0)";
			ctx.fillRect(0, 0, c.width, c.height);
		});
		for (int x = 0; x < disp.backgroundDrawCommands.size(); x++)
		{
			//printf("Background command %d\n", x);
			DoDrawCommand(disp.backgroundDrawCommands[x], "bg");
		}


		const char* align[] = {"\"left\"", "\"center\"", "\"right\""};
		const char* base[] = {"\"top\"", "\"middle\"", "\"bottom\""};
		for (int x = 0; x < disp.backgroundText.size(); x++)
		{
			const auto &i = disp.backgroundText[x];
			int r = i.c.r*255.0;
			int g = i.c.g*255.0;
			int b = i.c.b*255.0;
			int alignType = 0;
			int baseType = 0;
			switch (i.align)
			{
				case Graphics::textAlignLeft: alignType = 0; break;
				case Graphics::textAlignCenter: alignType = 1; break;
				case Graphics::textAlignRight: alignType = 2; break;
			}
			switch (i.base)
			{
				case Graphics::textBaselineTop: baseType = 0; break;
				case Graphics::textBaselineMiddle: baseType = 1; break;
				case Graphics::textBaselineBottom: baseType = 2; break;
			}

			EM_ASM_({
				var c=document.getElementById("bg");
				var ctx=c.getContext("2d");
				ctx.font = $6+"px "+$7;
				ctx.textAlign = UTF8ToString($8);
				ctx.textBaseline = UTF8ToString($9);
				ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
				ctx.fillText(UTF8ToString($0),$4,$5);
			}, i.s.c_str(), r,g,b,
					PointXToCanvas(i.loc.x, i.viewport),
					PointYToCanvas(i.loc.y, i.viewport),
					WidthToCanvas(i.size, i.viewport),
//					PointYToCanvas(i.size, i.viewport)-
//					PointYToCanvas(0, i.viewport),
					i.typeface.c_str(),
					align[alignType], base[baseType]);

		}

	}

	EM_ASM({
		var c=document.getElementById("fg");
		var ctx=c.getContext("2d");
		ctx.clearRect(0, 0, c.width, c.height);
	});
	for (int x = 0; x < disp.drawCommands.size(); x++)
	{
		DoDrawCommand(disp.drawCommands[x], "fg");
	}

	const char* align[] = {"left", "center", "right"};
	const char* base[] = {"top", "middle", "bottom"};
	for (int x = 0; x < disp.text.size(); x++)
	{
		const auto &i = disp.text[x];
		int r = i.c.r*255.0;
		int g = i.c.g*255.0;
		int b = i.c.b*255.0;

		int alignType = 0;
		int baseType = 0;
		switch (i.align)
		{
			case Graphics::textAlignLeft: alignType = 0; break;
			case Graphics::textAlignCenter: alignType = 1; break;
			case Graphics::textAlignRight: alignType = 2; break;
		}
		switch (i.base)
		{
			case Graphics::textBaselineTop: baseType = 0; break;
			case Graphics::textBaselineMiddle: baseType = 1; break;
			case Graphics::textBaselineBottom: baseType = 2; break;
		}

		EM_ASM_({
			var c=document.getElementById("fg");
			var ctx=c.getContext("2d");
			ctx.font = ""+$6+"px "+UTF8ToString($7);
			//ctx.font = "75px Helvetica";
			ctx.textAlign = UTF8ToString($8);
			ctx.textBaseline = UTF8ToString($9);
			//				ctx.textAlign = "center";
			ctx.fillStyle = "rgb("+$1+", "+$2+", "+$3+")";
			ctx.fillText(UTF8ToString($0),$4,$5);
		}, i.s.c_str(), r,g,b,
				PointXToCanvas(i.loc.x, i.viewport),
				PointYToCanvas(i.loc.y, i.viewport),
				WidthToCanvas(i.size, i.viewport),
//				PointYToCanvas(i.size, i.viewport)-
//				PointYToCanvas(0, i.viewport), i.typeface.c_str(),
				i.typeface.c_str(),
				align[alignType], base[baseType]);


	}
	static std::vector<Graphics::point> outputPoints;
	for (int x = 0; x < disp.lineSegments.size(); x++)
	{
		const auto &i = disp.lineSegments[x];
		int r = i.c.r*255.0;
		int g = i.c.g*255.0;
		int b = i.c.b*255.0;
		outputPoints = i.points;
		EM_ASM_({
			var c=document.getElementById("fg");
			var ctx=c.getContext("2d");
			ctx.strokeStyle = "rgb("+$0+", "+$1+", "+$2+")";
			ctx.beginPath();
		}, r,g,b);
		for (int x = 0; x < outputPoints.size(); x++)
		{
			auto &j = outputPoints[x];
			PointToCanvas(j, i.viewport);
			if (x == 0)
			{
				EM_ASM_({
					var c=document.getElementById("fg");
					var ctx=c.getContext("2d");
					ctx.moveTo($0, $1);
				}, j.x, j.y);
			}
			else {
				//var c=document.getElementById(UTF8ToString($0));
				EM_ASM_({
					var c=document.getElementById("fg");
					var ctx=c.getContext("2d");
					ctx.lineTo($0, $1);
				}, j.x, j.y);
			}
		}
		EM_ASM_({
			var c=document.getElementById("fg");
			var ctx=c.getContext("2d");
			ctx.lineWidth=$0;
			ctx.stroke();
			ctx.lineWidth=1;
		}, WidthToCanvas(i.size, disp.lineSegments[x].viewport));
//		ctx.lineWidth=$8;
//		ctx.stroke();
//	}, which, r,g,b,
//			PointXToCanvas(o.start.x, d.viewport), PointYToCanvas(o.start.y, d.viewport),
//			PointXToCanvas(o.end.x, d.viewport), PointYToCanvas(o.end.y, d.viewport),
//			WidthToCanvas(o.width, d.viewport));


	}
}

//int GetViewport(int x, int y)
//{
//	pRecContext pContextInfo = getCurrentContext();
//	if (pContextInfo->display.GetNumViewports() == 1)
//		return 0;
//	point3d p = {static_cast<GLfloat>(2.0*x/canvasWidth-1), static_cast<GLfloat>(2.0*y/canvasHeight-1), 0};
//	if (p.y < 2.0*(1.0-0.75))
//		return 0;
//	return 1;
//}


void HOGDoMouse(int x, int y, bool up, bool down, bool drag)
{
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->display.windowHeight = canvasHeight;
	pContextInfo->display.windowWidth = canvasWidth;
	tMouseEventType t = kMouseDown;

	if (up)
		t = kMouseUp;
	if (drag && down)
		t = kMouseDrag;
	if (!up && !down && drag)
		t = kMouseMove;
	//	int viewport = GetViewport(x, y);

	HandleMouse(pContextInfo, ConvertCanvasToGlobalHOG(x, y), kLeftButton, t);
	//	HandleMouseClick(getCurrentContext(), viewport, x, y, p, kLeftButton, t);
}

void HOGInit(const char *data)
{
	if (data == 0)
	{
		hog_main(0, 0);
	}
	else {
		std::string s(data);
		std::vector<char*> args;
		args.push_back(&(s[0]));
		for (int x = 0; x < s.size(); x++)
		{
			if (s[x] == ',')
			{
				s[x] = '\0';
				args.push_back(&(s[x+1]));
			}
		}
		// int argc, char **argv
		int argc = args.size();
		hog_main(argc, &(args[0]));
	}

	pRecContext pContextInfo = getCurrentContext();
	initialConditions(pContextInfo);
	HandleWindowEvent(pContextInfo, kWindowCreated);
}

void HOGHitKey(const char *key)
{
	if (key != 0)
	{
		printf("Hit key: '%c'\n", key[0]);
		DoKeyboardCommand(getCurrentContext(), key[0], false, false, false);
	}
}

recContext context;
pRecContext pContextInfo = &context;

void updateProjection(pRecContext pContextInfo, int viewPort)
{
}

pRecContext GetContext(unsigned long windowID)
{
	return pContextInfo;
}

pRecContext getCurrentContext()
{
	return pContextInfo;
}

void updateModelView(pRecContext pContextInfo, int currPort)
{

}

//GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
//pRecContext gTrackingContextInfo = NULL;

void renderScene()
{

}

void RunHOGGUI(int argc, char* argv[], int windowDimension)
{
	RunHOGGUI(argc, argv, windowDimension, windowDimension);
}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
	// We don't handle command-line arguments here
	ProcessCommandLineArgs(argc, argv);
	//initialConditions(pContextInfo);
	//HandleWindowEvent(pContextInfo, kWindowCreated);
}

char *textBuffer = 0;

void setTextBufferVisibility(bool)
{
#warning "text buffer visibility flag not implemented"
}

bool getTextBufferVisibility()
{
	return true;
}

void submitTextToBuffer(const char *val)
{
	if (val)
	{
		delete [] textBuffer;
		int len = strlen(val);
		textBuffer = new char[len+1];
		strncpy(textBuffer, val, len);
		textBuffer[len] = 0;
		//printf("Buffer: '%s'\n", val);

		EM_ASM_({
			document.getElementById("message").innerHTML = UTF8ToString($0);
		}, textBuffer);
	}
}

void appendTextToBuffer(const char *next)
{
	if (textBuffer == 0)
	{
		submitTextToBuffer(next);
		return;
	}
	int oldLen = strlen(textBuffer);
	int newLen = strlen(next)+oldLen;
	char *newStr = new char[newLen+1];
	strcpy(newStr, textBuffer);
	strcpy(&newStr[oldLen], next);
	delete [] textBuffer;
	textBuffer = newStr;

	EM_ASM_({
		document.getElementById("message").innerHTML = UTF8ToString($0);
	}, textBuffer);
}

const char *getTextBuffer()
{
    char *buffer = (char*)EM_ASM_PTR({
        return stringToNewUTF8(document.getElementById("message").innerHTML);
    });
    delete [] textBuffer;
    int len = strlen(buffer);
    textBuffer = new char[len+1];
    strncpy(textBuffer, buffer, len);
    textBuffer[len] = 0;
    free(buffer);
	return textBuffer;
}
