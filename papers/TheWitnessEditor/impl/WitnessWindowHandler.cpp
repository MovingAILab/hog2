#include "Driver.h"
#include "Globals.h"

void WitnessWindowHandler(unsigned long windowID, tWindowEventType eType)
{
    if (eType == kWindowDestroyed)
    {
        printf("Window %ld destroyed\n", windowID);
        RemoveFrameHandler(WitnessFrameHandler, windowID, nullptr);
    }
    else if (eType == kWindowCreated)
    {
        //		for (int x = 0; x < 50; x++)
        //			DrawPaperLevel(x);

        printf("Window %ld created\n", windowID);
        InstallFrameHandler(WitnessFrameHandler, windowID, nullptr);
        SetNumPorts(windowID, 1);

        //		w.AddTriangleConstraint(0, 0, 3);
        //		w.AddGoal(1, -1);
        //		w.AddGoal(-1, 1);
        //		w.AddGoal(puzzleWidth, 0);
        //		w.AddGoal(0, puzzleHeight);
        //		w.AddGoal(puzzleWidth, puzzleHeight+1);
        //		w.AddTriangleConstraint(0, 1, 2);
        //		w.AddTriangleConstraint(1, 0, 1);
        //		ExamineMustCross(numRequiredPieces);
        //        witness.AddStarConstraint(0, 0, Colors::pink);
        //		w.AddSeparationConstraint(0, 0, Colors::green);
        //		w.AddSeparationConstraint(0, 4, Colors::white);
        //		w.AddSeparationConstraint(4, 0, Colors::white);
        //        witness.AddSeparationConstraint(2, 2, Colors::pink);
        //		w.AddSeparationConstraint(4, 4, Colors::black);
        //        witness.AddSeparationConstraint(2, 0, Colors::black);
        //		w.AddSeparationConstraint(2, 4, Colors::black);
        //        witness.AddSeparationConstraint(0, 2, Colors::orange);
        //        witness.AddSeparationConstraint(0, 0, Colors::black);

        //		w.AddSeparationConstraint(4, 2, Colors::orange);

        //		std::string s =
        //"{\"dim\":\"3x3\",\"cc\":{\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"4;3;#385CDE\",\"3;10;#DCA700\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000000000000000000000000000000000000\"}\"";
        //		w.LoadFromHashString(s);

        //		std::string s2 =
        //"{\"dim\":\"4x4\",\"cc\":{\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"10000000000000000000000000001100000000000000000000000000000000000\"}";
        //		std::string s =
        //"{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;1077084160;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"00000000010000010000000000000001000000000000000000000000000000000\"}";
        //		s =
        //"{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"1;0;#FFFFFF\"},\"mc\":\"00000010000000000000000000000000000000000000000000000000000000000\"}";
        //		s =
        //"{\"dim\":\"4x4\",\"cc\":{\"1;0;#0000FF\",\"0;0;#000026D\",\"0;0;#0029D00\",\"2;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\",\"2;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#0000FF\"},\"mc\":\"10000000000010000000000000000000100000000000000000000000000000000\"}";

        //		w.LoadFromHashString(s);
        //		w.AddTriangleConstraint(0, 0, 1);
        //		w.AddTriangleConstraint(1, 1, 2);
        //		w.AddTriangleConstraint(2, 2, 3);
        //		w.AddStarConstraint(3, 3, Colors::orange);
        //		w.AddCannotCrossConstraint(1, 4);
        //		w.AddCannotCrossConstraint(3, 1);
        //		w.AddCannotCrossConstraint(false /*vert*/, 2, 1);
        //		w.AddCannotCrossConstraint(true /*horiz*/, 0, 2);
        //		w.AddTetrisConstraint(1, 1, 10);
        //		w.AddNegativeTetrisConstraint(1, 0, 3);
        //		w.AddTetrisConstraint(2, 1, 2);
        //		w.AddTetrisConstraint(1, 2, 2);
        //		w.AddTetrisConstraint(2, 2, 2);
        //		w.AddTetrisConstraint(1, 3, 2);
        //		w.AddTetrisConstraint(2, 3, 2);
        //		w.AddMustCrossConstraint(false, 2, 0);
        //		w.AddMustCrossConstraint(false, 2, 4);

        //		w.AddTetrisConstraint(0, 2, 2);
        //		w.AddTetrisConstraint(0, 1, 1);
        //		w.AddTetrisConstraint(3, 3, );
        //		w.AddTetrisConstraint(0, 0, 10);
        //		w.AddTetrisConstraint(3, 0, 12);
        //		w.AddTetrisConstraint(3, 1, 14);
        //		w.AddTetrisConstraint(2, 3, -8);
        //		w.AddTetrisConstraint(2, 2, -12);
        //		w.AddTetrisConstraint(1, 1, -13);
        //		w.AddTetrisConstraint(1, 0, 4);
        //		w.AddTetrisConstraint(1, 1, 5);
        //		w.AddTetrisConstraint(1, 2, 6);
        //		w.AddTetrisConstraint(1, 3, 7);
        //		w.AddTetrisConstraint(2, 0, 8);
        //		w.AddTetrisConstraint(2, 1, 9);
        //		w.AddTetrisConstraint(2, 2, 10);
        //		w.AddTetrisConstraint(2, 3, 11);
        ReinitViewports(windowID, {-1.0f, -1.0f, 0.0f, 1.0f}, kScaleToSquare);
        AddViewport(windowID, {1.0f, -1.0f, 2.0f, 1.0f}, kScaleToSquare);
        //        AddViewport(windowID, {0.0f, -1.0f, 1.0f, 1.0f}, kScaleToSquare);
        AddViewport(windowID, {1.0f, 0.0f, 2.0f, 2.0f}, kScaleToSquare);
        //        AddViewport(windowID, {0.0f, 1.0f, 1.0f, 2.0f}, kScaleToFill);
    }
}
