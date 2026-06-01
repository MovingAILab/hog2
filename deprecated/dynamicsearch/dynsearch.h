void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void MyDisplayHandler(unsigned long windowID, tKeyboardModifier, char key);
int MyCLHandler(char *argument[], int maxNumArgs);
bool MyClickHandler(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);
void InstallHandlers();
