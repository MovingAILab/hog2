/*
 *  TextOverlay.cpp
 *
 *  Created by Nathan Sturtevant on 7/15/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "TextOverlay.h"
#include "Constants.h"
#include <string.h>
#include <stdint.h>

using std::string;

TextOverlay::TextOverlay(int maxLines)
{
	maxNumLines = maxLines;
    index = 0;
	bold = false;
}

TextOverlay::~TextOverlay()
{
    for (uint32_t x = 0; x < text.size(); x++)
    {
        //delete [] text[x];
        //text[x] = 0;
    }
}

void TextOverlay::SetBold(bool useBold)
{
	bold = useBold;
}

void TextOverlay::AddLine(const char *line)
{
    if (line == 0)
        return;
    string txt(line);
    text.push_back(txt);
    if (text.size()-index > maxNumLines)
        index++;
}

const char *TextOverlay::GetLastLine()
{
    if (text.size() > 0)
		return text.back().c_str();
	return 0;
}

void TextOverlay::DeleteChar()
{
    if (text.size() > 0)
        text.back().resize(text.back().length()-1);
}

void TextOverlay::AppendToLine(const char *line)
{
    string txt(line);
    if (text.size() > 0)
        text.back().append(txt);
    else
        text.push_back(txt);
}

void TextOverlay::Clear()
{
    text.resize(0);
    index = 0;
}

//void TextOverlay::OpenGLDraw(int window)
//{
//    if (text.size() == 0)
//        return;
//    
//    glDisable(GL_LIGHTING);
//	if (bold)
//		glLineWidth(3.0);
//	else
//		glLineWidth(2.0);
//    glEnable(GL_LINE_SMOOTH);
//    
//    GLint matrixMode;
//    
//    glDisable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND); // for text fading
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ditto
//                                                        // set orthograhic 1:1  pixel transform in local view coords
//    glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
//    glMatrixMode(GL_PROJECTION);
//    glPushMatrix();
//    glLoadIdentity();
//    glFrustum(-1, 1, -1, 1, 1.9, 2.1);	
//    //gluLookAt(0, 0, -2, 0, 0, 0, 0, -1, 0);
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    
//    // draw transparent background behind text
//    glLoadIdentity();
//    glBegin(GL_QUADS);
//    glColor4f(0.0, 0.0, 0.0, 0.5);
//    glVertex3f(-1-0.01, -1-0.07, 0.0);
//    glVertex3f( 1+0.01, -1-0.07, 0.0);
////    glVertex3f( 1+0.01, -1+(text.size()-index-1)*0.06+0.01, 0.0);
////    glVertex3f(-1-0.01, -1+(text.size()-index-1)*0.06+0.01, 0.0);
//	glVertex3f( 1+0.01, -1+(maxNumLines-index-1)*0.06+0.01, 0.0);
//	glVertex3f(-1-0.01, -1+(maxNumLines-index-1)*0.06+0.01, 0.0);
//    glEnd();
//    
//	int lineOffset = 0;
//    for (unsigned int x = index; x < text.size(); x++)
//    {
//		
//		int charOffset = 0;
//		while (charOffset != -1)
//		{
//			glLoadIdentity();
//			glTranslatef(-1, -1+lineOffset*0.06, 0);
//			glScalef(1.0/(24*120.0), 1.0/(24*120.0), 1);
//			glRotatef(180, 0.0, 0.0, 1.0);
//			glRotatef(180, 0.0, 1.0, 0.0);
//			glColor3f(1.0, 1.0, 1.0);
//			lineOffset++;
////			printf("Drawing '%s' on line %d\n", &(text[x].c_str()[charOffset]), lineOffset);
//			if (x == text.size()-1)
//				charOffset = DrawString(text[x]+"_", charOffset);
//			else
//				charOffset = DrawString(text[x], charOffset);
//		}
//    }
//	if (lineOffset > maxNumLines)
//		index++;
//    
//    // reset orginal martices
//    glPopMatrix(); // GL_MODELVIEW
//    glMatrixMode(GL_PROJECTION);
//    glPopMatrix();
//    glMatrixMode(matrixMode);
//    glEnable(GL_DEPTH_TEST);
//	glDisable(GL_LINE_SMOOTH);
//	glLineWidth(1.0);
//
//}

void TextOverlay::Draw(Graphics::Display &display) const
{
	const float linesPerPage = 30.0f;
//	int lineOffset = 0;
//	printf("%f: ", -1.0f+maxNumLines*2.0f/(linesPerPage));
	display.FillRect({-1.0, -1.0, 1.0, -1.0f+maxNumLines*2.0f/(linesPerPage)}, Colors::darkgray);
	display.FrameRect({-1.0, -1.0, 1.0, -1.0f+maxNumLines*2.0f/(linesPerPage)}, Colors::white, 0.1*2.0/linesPerPage);
	for (unsigned int x = index; x < text.size(); x++)
	{
		Graphics::point p(-1, -1+(x-index+1)*(2.0/(linesPerPage)));
		display.DrawText(text[x].c_str(), p, Colors::white, 2.0/(linesPerPage),
						 Graphics::textAlignLeft, Graphics::textBaselineBottom, "Courier");
//		printf("%f ", p.y);
	}
//	printf("\n");
}

int TextOverlay::DrawString(std::string txt, int start)
{
	int cnt = 0;
    for (unsigned int x = start; x < txt.length(); x++)
    {
		if (txt[x] == '\n')
		{
			return x+1;
		}
		if (txt[x] == '\t')
		{
			//			for (int x = 0; x < 3; x++)
			//	glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, ' ');
			//return x+1;
			//cnt+=3;
		}
		else {
			//glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, txt[x]);
			cnt++;
		}
		if ((cnt > 45) && (txt[x] == ' '))
		{
			return x+1;
		}
		if (cnt > 50)
		{
			return x+1;
		}
    }
	return -1;
}
