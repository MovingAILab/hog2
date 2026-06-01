//
//  AppDelegate.m
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 7/27/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#import "AppDelegate.h"

@interface AppDelegate ()

@end

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
	// Insert code here to initialize your application
/*
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
*/
}


- (void)applicationWillTerminate:(NSNotification *)aNotification {
	// Insert code here to tear down your application
}


@end
