//
//  ViewController.mm
//  hog2 iPad
//
//  Created by Nathan Sturtevant on 7/15/19.
//  Copyright © 2019 MovingAI. All rights reserved.
//

#import "ViewController.h"

@interface ViewController ()

@end

int hog_main(int argc, char **argv);
std::vector<char> keys;
std::vector<std::string> descr;

@implementation ViewController

#include "Common.h"
const float FRAMERATE = 1.0f/30.0f;

- (void)viewDidLoad {
	[super viewDidLoad];
	
	NSArray *arguments = [[NSProcessInfo processInfo] arguments];
	// use -objectAtIndex: to obtain an element of the array
	// and -count to obtain the number of elements in the array
	char **args = new char*[[arguments count]];
	int len = [arguments count];
	for (int x = 0; x < len; x++)
	{
		int nextLen = [[arguments objectAtIndex:x] length];
		args[x] = new char[nextLen + 1];
		strncpy(args[x], [[arguments objectAtIndex:x] cStringUsingEncoding:NSASCIIStringEncoding], nextLen);
		args[x][nextLen] = 0;
	}
	
	hog_main([arguments count], args);
	//	[[[self view] window] setAcceptsMouseMovedEvents:YES];
//	NSTrackingArea* trackingArea = [[NSTrackingArea alloc] initWithRect:[self.view bounds] options: (NSTrackingMouseMoved | NSTrackingActiveInKeyWindow | NSTrackingInVisibleRect) owner:self userInfo:nil];
//	[self.view addTrackingArea:trackingArea];
	
	
	
	[NSTimer scheduledTimerWithTimeInterval:FRAMERATE
									 target:self
								   selector:@selector(onTick:)
								   userInfo:nil
									repeats:YES];
	
	pRecContext pContextInfo = getCurrentContext();
	initialConditions(pContextInfo);
	HandleWindowEvent(pContextInfo, kWindowCreated);
	_backView.display = &pContextInfo->display;
	_backView.isBackground = true;
	_frontView.display = &pContextInfo->display;
	_frontView.isBackground = false;

	{
		//NSMutableArray *tmp = [[NSMutableArray alloc] init];
		GetKeyAssignments(keys);
		GetKeyAssignmentDescriptions(descr);
		float itemWidth = (self.view.frame.size.width)/(descr.size()+1);
		for (int x = 0; x < descr.size(); x++)
		{
//			[tmp addObject:[NSString stringWithUTF8String:descr[x].c_str()]];
			UIButton *button = [UIButton buttonWithType:UIButtonTypeSystem];
			[button addTarget:self
					   action:@selector(hitButton:)
			 forControlEvents:UIControlEventTouchUpInside];
			[button setTitle:[NSString stringWithUTF8String:descr[x].c_str()] forState:UIControlStateNormal];
			button.frame = CGRectMake(itemWidth/2+x*itemWidth,
									  self.view.frame.size.height-10-40,
									  itemWidth, 40.0);
			button.tag = x;
			[self.view addSubview:button];
		}

		
		
//		NSArray *itemArray = [NSArray arrayWithArray:tmp];
//		//arrayWithObjects: @"Map 1",@"Map 2", @"Map 3", @"Map 4", @"Map 5", @"Map 6", @"Map 7", nil;
//		UISegmentedControl *segmentedControl = [[UISegmentedControl alloc] initWithItems:itemArray];
//		segmentedControl.frame = CGRectMake(10, self.view.frame.size.height-10-40, self.view.frame.size.width-10, 40);
//		segmentedControl.backgroundColor = [UIColor whiteColor];
//
//		[segmentedControl addTarget:self action:@selector(mapSegmentHit:) forControlEvents: UIControlEventValueChanged];
////		[segmentedControl addTarget:self action:@selector(mapSegmentHit:) forControlEvents: UIControlEventTouchDown];
//		segmentedControl.selectedSegmentIndex = 0;
//		[[UISegmentedControl appearance] setTitleTextAttributes:@{NSForegroundColorAttributeName : [UIColor whiteColor]} forState:UIControlStateSelected];
//		[self.view addSubview:segmentedControl];
	}

	//	initialConditions(pContextInfo);
//	drawingView.display = &pContextInfo->display;
	//	self.drawingView.display = pContextInfo->display;
}

- (void)hitButton:(UIButton *)item
{
//	printf("ding\n");
	DoKeyboardCommand(getCurrentContext(), keys[item.tag], kNoModifier, false, false);
}

- (void)mapSegmentHit:(UISegmentedControl *)segment
{
	DoKeyboardCommand(getCurrentContext(), keys[segment.selectedSegmentIndex], kNoModifier, false, false);
}
//- (void)setRepresentedObject:(id)representedObject {
//	[super setRepresentedObject:representedObject];
//
//	// Update the view, if already loaded.
//}

-(void)onTick:(NSTimer *)timer {
	pRecContext pContextInfo = getCurrentContext();
	pContextInfo->display.StartFrame();
	for (int x = 0; x < pContextInfo->display.numViewports; x++)
	{
		pContextInfo->display.SetViewport(x);
		//if (pContextInfo->drawing)
		{
			// set projection
			HandleFrame(pContextInfo, x);
		}
	}
	pContextInfo->display.EndFrame();
	if (pContextInfo->display.BackgroundNeedsRedraw())
		[_backView setNeedsDisplay];
	[_frontView setNeedsDisplay];
	if (getTextBuffer() != 0)
	{
		NSString *tmp = [NSString stringWithUTF8String:getTextBuffer()];
		if ([tmp length] > 99)
			tmp = [tmp substringToIndex:99];
		[_textField setText:tmp];
	}
}

//-(tButtonType)getButton:(NSEvent *)event
//{
//	switch ([event type])// == NSEventTypeLeftMouseUp)
//	{
//		case NSEventTypeLeftMouseDown: return kLeftButton; break;
//		case NSEventTypeLeftMouseUp: return kLeftButton; break;
//		case NSEventTypeRightMouseDown: return kRightButton; break;
//		case NSEventTypeRightMouseUp:  return kRightButton; break;
//		case NSEventTypeLeftMouseDragged:  return kLeftButton; break;
//		case NSEventTypeRightMouseDragged:  return kRightButton; break;
//	}
//	return kLeftButton;
//}
//
//-(void)mouseDown:(NSEvent *)event
//{
//	tButtonType bType = [self getButton:event];
//	NSPoint curPoint = [event locationInWindow];
//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
//	HandleMouse(getCurrentContext(), p, bType, kMouseDown);
//	//	int viewport = [drawingView getViewport:curPoint];
//	//	HandleMouseClick(getCurrentContext(), viewport, curPoint.x, curPoint.y, p, bType, kMouseDown);
//}
//
//
//
//-(void)mouseUp:(NSEvent *)event
//{
//	tButtonType bType = [self getButton:event];
//	NSPoint curPoint = [event locationInWindow];
//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
//	HandleMouse(getCurrentContext(), p, bType, kMouseUp);
//	//	int viewport = [drawingView getViewport:curPoint];
//	//	HandleMouseClick(getCurrentContext(), viewport, curPoint.x, curPoint.y, p, bType, kMouseUp);
//}
//
//-(void)mouseMoved:(NSEvent *)event
//{
//	NSPoint curPoint = [event locationInWindow];
//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
//	HandleMouse(getCurrentContext(), p, kNoButton, kMouseMove);
//}
//
//-(void)mouseDragged:(NSEvent *)event
//{
//	tButtonType bType = [self getButton:event];
//	NSPoint curPoint = [event locationInWindow];
//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
//	HandleMouse(getCurrentContext(), p, bType, kMouseDrag);
//	//	int viewport = [drawingView getViewport:curPoint];
//	//	HandleMouseClick(getCurrentContext(), viewport, curPoint.x, curPoint.y, p, bType, kMouseDrag);
//}

-(void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
	//Get all the touches.
	NSSet *allTouches = [event allTouches];
	
	//Number of touches on the screen
	UITouch *touch = [[allTouches allObjects] objectAtIndex:0];
	CGPoint tmp = [touch locationInView:_frontView];
	printf("%f in bound height %f (%f)\n", tmp.y, self.view.frame.size.height, tmp.y/self.view.frame.size.height);
	point3d p;// = {static_cast<float>(tmp.x), static_cast<float>(tmp.y)};
	//p = [_frontView getWorldCoordinate:p];
	p = [_frontView  convertToGlobalHogCoordinate:tmp];//getWorldCoordinate:p];
	printf("converts to %f\n", p.y);
	HandleMouse(getCurrentContext(), p, kLeftButton, kMouseDown);
	//	tButtonType bType = [self getButton:event];
	//	NSPoint curPoint = [event locationInWindow];
	//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
	//	HandleMouse(getCurrentContext(), p, bType, kMouseDown);
}

-(void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
	//Get all the touches.
	NSSet *allTouches = [event allTouches];
	
	// use first touch by default
	UITouch *touch = [[allTouches allObjects] objectAtIndex:0];
	CGPoint start = [touch locationInView:_frontView];
	point3d p;// = {static_cast<float>(start.x), static_cast<float>(start.y)};
	p = [_frontView  convertToGlobalHogCoordinate:start];//getWorldCoordinate:p];
	//	tButtonType bType = [self getButton:event];
	//	NSPoint curPoint = [event locationInWindow];
	//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
	HandleMouse(getCurrentContext(), p, kLeftButton, kMouseDrag);
}

-(void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
	//Get all the touches.
	NSSet *allTouches = [event allTouches];
	
	UITouch *touch = [[allTouches allObjects] objectAtIndex:0];
	CGPoint start = [touch locationInView:_frontView];
	point3d p;// = {static_cast<float>(start.x), static_cast<float>(start.y)};
	p = [_frontView  convertToGlobalHogCoordinate:start];//getWorldCoordinate:p];
	//	point3d p = [drawingView convertToGlobalHogCoordinate:curPoint];
	HandleMouse(getCurrentContext(), p, kLeftButton, kMouseUp);
}


//-(void)keyDown:(NSEvent *)event
//{
//	NSString *characters;
//	characters = [event characters];
//	NSLog(characters);
//}

-(BOOL)acceptsFirstResponder
{
	return YES;
}

//- (void)keyDown:(NSEvent *)event
//{
//	NSString *characters;
//	characters = [event characters];
//	//	printf("%c : %d!\n", [characters characterAtIndex:0], [characters characterAtIndex:0]);
//	bool shift = false;
//	if (isupper([characters characterAtIndex:0]))
//		shift = true;
//	DoKeyboardCommand(getCurrentContext(), [characters characterAtIndex:0], shift?kShiftDown:kNoModifier, false, false);
//}

@end
