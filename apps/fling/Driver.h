/*
 *  $Id: sample.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void MyDisplayHandler(unsigned long windowID, tKeyboardModifier, char key);
void SolveAndSaveInstance(unsigned long windowID, tKeyboardModifier, char key);
void SolveRandomFlingInstance(unsigned long windowID, tKeyboardModifier, char key);
void TestRanking(unsigned long windowID, tKeyboardModifier, char key);
void BuildTables(unsigned long windowID, tKeyboardModifier, char key);
void ExtractUniqueStates(int depth);
void ReadTables(unsigned long windowID, tKeyboardModifier, char key);
void AnalyzeBoard(unsigned long windowID, tKeyboardModifier, char key);
void DeepAnalyzeBoard(unsigned long windowID, tKeyboardModifier, char key);
void BFSearch(unsigned long windowID, tKeyboardModifier, char key);
void MassAnalysis(unsigned long , tKeyboardModifier mod, char c);
void CaptureScreen(unsigned long , tKeyboardModifier mod, char c);
void FindLogicalMoves(unsigned long , tKeyboardModifier , char);
void RemoveDuplicates(unsigned long , tKeyboardModifier , char );

int MyCLHandler(char *argument[], int maxNumArgs);
bool MyClickHandler(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);
void InstallHandlers();
