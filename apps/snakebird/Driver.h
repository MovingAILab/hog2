/*
 *  Driver.h
 *  hog2 - snakebird
 */

void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void GamePlayKeyboardHandler(unsigned long windowID, tKeyboardModifier, char key);
void EditorKeyBoardHandler(unsigned long windowID, tKeyboardModifier, char key);
void EditorStudyKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key);
int MyCLHandler(char *argument[], int maxNumArgs);
bool MyClickHandler(unsigned long windowID, int viewport, int x, int y, point3d loc, tButtonType, tMouseEventType);
void ShowSolutionLength(int length = 0);
void InstallHandlers();
