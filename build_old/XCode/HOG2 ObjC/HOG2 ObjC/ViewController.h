//
//  ViewController.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 7/27/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#include "Common.h"
#import "DrawView.h"

@interface ViewController : NSViewController
{
	__weak IBOutlet DrawView *drawingView;
	__weak IBOutlet NSTextField *messageField;
}

@end

