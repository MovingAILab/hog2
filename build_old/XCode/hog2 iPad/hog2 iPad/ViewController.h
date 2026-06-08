//
//  ViewController.h
//  hog2 iPad
//
//  Created by Nathan Sturtevant on 7/15/19.
//  Copyright © 2019 MovingAI. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "OverlayView.h"

@interface ViewController : UIViewController

@property (weak, nonatomic) IBOutlet UITextField *textField;
@property (weak, nonatomic) IBOutlet OverlayView *backView;
@property (weak, nonatomic) IBOutlet OverlayView *frontView;


@end

