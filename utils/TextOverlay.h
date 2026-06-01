/*
 *  TextOverlay.h
 *
 *  Created by Nathan Sturtevant on 7/15/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef TEXTOVERLAY_H
#define TEXTOVERLAY_H

#include <vector>
#include <string>
#include "Graphics.h"

class TextOverlay {
public:
	TextOverlay(int maxLines = 34);
	~TextOverlay();
	void SetBold(bool useBold);
	void AddLine(const char *);
	void DeleteChar();
    void AppendToLine(const char *);
	const char *GetLastLine();
    void Clear();
//	void OpenGLDraw(int window);
	void Draw(Graphics::Display &display) const;
private:
    int DrawString(std::string, int start = 0);
    std::vector<std::string> text;
    int index;
	int maxNumLines;
	bool bold;
};

#endif
