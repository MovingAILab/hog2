/*
 *  $Id: TextBox.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 04/12/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <string.h>
#include "TextBox.h"
#include "GLUtil.h"

TextBox::TextBox(char *_text, int _charLine, point3d topLeft, point3d bottomRight, double _duration, bool _deform, bool _scrolling)
{
	tl = topLeft;
	br = bottomRight;
	text = new char[strlen(_text)+1];
	strcpy(text, _text);
	charLine = _charLine;
	dList = 0;
	elapsedTime = 0;
	duration = _duration;
	deform = _deform;
	scrolling = _scrolling;
}

TextBox::~TextBox()
{
	delete [] text;
  if (dList)
    glDeleteLists(dList, 1);
}

void TextBox::destroy()
{
	elapsedTime = duration;
}

void TextBox::stepTime(double amount)
{
	elapsedTime += amount;
}

void TextBox::draw()
{
	glLineWidth(1);
	double wide, charWidth;
	wide = br.x-tl.x;
	//high = tl.y-br.y;
	charWidth = wide/charLine;
	GLint matrixMode;
	if (scrolling)
		glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
	
	glEnable(GL_BLEND); // for text fading
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ditto

	if (dList == 0)
	{
		// set orthograhic 1:1  pixel transform in local view coords
		glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
		
		dList = glGenLists(1);
		glNewList(dList, GL_COMPILE);

		glDisable(GL_DEPTH_TEST);
		if ((deform) && (!scrolling))
		{
			glMatrixMode(GL_PROJECTION);
			glPushMatrix();
			glLoadIdentity();
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glLoadIdentity();
			glColor3f(myColor.r, myColor.g, myColor.b);
		}
		glBegin(GL_LINES);
    //printf("drawing text \"%s\"\n", text);
		for (unsigned int x = 0; x < strlen(text); x++)
		{
			point3d np = tl;
			np.x+=(x%charLine)*charWidth+charWidth/7.5;
			np.z-=(x/charLine)*charWidth*2+charWidth/7.5;
			drawChar(text[x], np, 2*(charWidth-charWidth/(7.5/2)));
      //printf("%c/%d ", text[x], (int)text[x]);
		}
    //printf("\n");
		glEnd();

		// reset orginal martices
		if ((deform) && (!scrolling))
		{
			glPopMatrix(); // GL_MODELVIEW
			glMatrixMode(GL_PROJECTION);
			glPopMatrix();
			glMatrixMode(matrixMode);
		}
		glEnable(GL_DEPTH_TEST);
		glEndList();

		//glReportError();
	}
	if (dList)
	{
		if (scrolling)
		{
			if (deform)
			{
				glMatrixMode(GL_PROJECTION);
				glPushMatrix();
				glLoadIdentity();
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glLoadIdentity();
			}
			glColor3f(myColor.r, myColor.g, myColor.b);
			glTranslated(0, elapsedTime/30.0, 0);
			glCallList(dList);
			glColor4f(myColor.r, myColor.g, myColor.b, .5);
			glTranslated(0, -1.0/350.0, 0); // 700 pixel = 2, 1 pixel = 1/350
			glCallList(dList);
			glColor4f(myColor.r, myColor.g, myColor.b, .25);
			glTranslated(0, -1.0/350.0, 0); // 700 pixel = 2, 1 pixel = 1/350
			glCallList(dList);
			glTranslated(0, -elapsedTime/30.0+2.0/350.0, 0);

			if (deform)
			{
				glPopMatrix(); // GL_MODELVIEW
				glMatrixMode(GL_PROJECTION);
				glPopMatrix();
				glMatrixMode(matrixMode);
			}
		}
		else
			glCallList(dList);
	}
	glDisable(GL_BLEND); // for text fading
}

point3d TextBox::getLocation()
{
	return tl;
}
/*
 *  _ _   0, 1
 * |X|X|  2, 3/, 4\, 5, 6/, 7\, 8
 * |\|/|  9, 10, 11, 12, 13
 *  - -   14, 15
 * |/|\|  16, 17, 18, 19, 20
 * |X|X|  21, 22/, 23\, 24, 25/, 26\, 27
 *  - -   28, 29
 *
 */

void TextBox::drawChar(char c, point3d where, double height)
{
	uint32_t bMap = getBitmap(toupper(c));
	for (int x = 0; x < 30; x++)
	{
		if (bMap&0x1)
		{
			switch (x) {
				case 0: drawLine(where, 0, 0, 1, 0, height); break;
				case 1: drawLine(where, 1, 0, 1, 0, height); break;
				case 2: drawLine(where, 0, 0, 0, 1, height); break;
				case 3: drawLine(where, 0, 1, 1, -1, height); break;
				case 4: drawLine(where, 0, 0, 1, 1, height); break;
				case 5: drawLine(where, 1, 0, 0, 1, height); break;
				case 6: drawLine(where, 1, 1, 1, -1, height); break;
				case 7: drawLine(where, 1, 0, 1, 1, height); break;
				case 8: drawLine(where, 2, 0, 0, 1, height); break;
				case 9: drawLine(where, 0, 1, 0, 1, height); break;
				case 10: drawLine(where, 0, 1, 1, 1, height); break;
				case 11: drawLine(where, 1, 1, 0, 1, height); break;
				case 12: drawLine(where, 1, 2, 1, -1, height); break;
				case 13: drawLine(where, 2, 1, 0, 1, height); break;
				case 14: drawLine(where, 0, 2, 1, 0, height); break;
				case 15: drawLine(where, 1, 2, 1, 0, height); break;
				case 16: drawLine(where, 0, 2, 0, 1, height); break;
				case 17: drawLine(where, 0, 3, 1, -1, height); break;
				case 18: drawLine(where, 1, 2, 0, 1, height); break;
				case 19: drawLine(where, 1, 2, 1, 1, height); break;
				case 20: drawLine(where, 2, 2, 0, 1, height); break;
				case 21: drawLine(where, 0, 3, 0, 1, height); break;
				case 22: drawLine(where, 0, 4, 1, -1, height); break;
				case 23: drawLine(where, 0, 3, 1, 1, height); break;
				case 24: drawLine(where, 1, 3, 0, 1, height); break;
				case 25: drawLine(where, 1, 4, 1, -1, height); break;
				case 26: drawLine(where, 1, 3, 1, 1, height); break;
				case 27: drawLine(where, 2, 3, 0, 1, height); break;
				case 28: drawLine(where, 0, 4, 1, 0, height); break;
				case 29: drawLine(where, 1, 4, 1, 0, height); break;
			}
		}
		bMap >>= 1;
	}
}

uint32_t TextBox::getBitmap(char c)
{
	uint32_t index[128] = {
		0x0, //   0
		0x0, //   1
		0x0, //   2
		0x0, //   3
		0x0, //   4
		0x0, //   5
		0x0, //   6
		0x0, //   7
		0x0, //   8
		0x0, //   9
		0x0, //  10
		0x0, //  11
		0x0, //  12
		0x0, //  13
		0x0, //  14
		0x0, //  15
		0x0, //  16
		0x0, //  17
		0x0, //  18
		0x0, //  19
		0x0, //  20
		0x0, //  21
		0x0, //  22
		0x0, //  23
		0x0, //  24
		0x0, //  25
		0x0, //  26
		0x0, //  27
		0x0, //  28
		0x0, //  29
		0x0, //  30
		0x0, //  31
		0x0, //  32        
		(1<<5)|(1<<11)|(1<<24), //  33       !
		(1<<5)|(1<<8), //  34       "
		0x0, //  35       #
		0x0, //  36       $
		0x0, //  37       %
		0x0, //  38       &
		(1<<5), //  39       '
		(1<<3)|(1<<9)|(1<<16)|(1<<23), //  40       (
		(1<<7)|(1<<13)|(1<<20)|(1<<25), //  41       )
		(1<<10)|(1<<11)|(1<<12)|(1<<14)|(1<<15)|(1<<17)|(1<<18)|(1<<19), //  42       *
		0x0, //  43       +
		(1<<22), //  44       ,
		(1<<15)|(1<<14), //  45       -
		(1<<22)|(1<<23), //  46       .
		(1<<8)|(1<<12)|(1<<18)|(1<<22), //  47       /
		//  48       0
		(1<<3)|(1<<7)|(1<<9)|(1<<12)|(1<<13)|(1<<16)|(1<<17)|(1<<20)|(1<<23)|(1<<25),
		//  49       1
		(1<<3)|(1<<5)|(1<<11)|(1<<18)|(1<<24)|(1<<28)|(1<<29),
		//  50       2
		(1<<3)|(1<<1)|(1<<8)|(1<<13)|(1<<15)|(1<<17)|(1<<21)|(1<<28)|(1<<29),
		//  51       3
		(1<<0)|(1<<7)|(1<<13)|(1<<15)|(1<<20)|(1<<25)|(1<<28),
		//  52       4
		(1<<3)|(1<<9)|(1<<14)|(1<<15)|(1<<11)|(1<<18)|(1<<24),
		//  53       5
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<19)|(1<<27)|(1<<28)|(1<<29),
		//  54       6
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<19)|(1<<27)|(1<<28)|(1<<29)|(1<<16)|(1<<21),
		//  55       7
		(1<<0)|(1<<1)|(1<<8)|(1<<12)|(1<<18)|(1<<24),
		//  56       8
		(1<<0)|(1<<1)|(1<<2)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<27)|(1<<28)|(1<<29),
		//  57       9
		(1<<0)|(1<<1)|(1<<2)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<20)|(1<<27)|(1<<28)|(1<<29),
		(1<<24)|(1<<11), //  58       :
		(1<<22)|(1<<11), //  59       ;
		0x0, //  60       <
		0x0, //  61       =
		0x0, //  62       >
		(1<<2)|(1<<1)|(1<<0)|(1<<8)|(1<<12)|(1<<24), //  63       ?
		0x0, //  64       @
				 //  65       A
		(1<<1)|(1<<3)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<27),
		//  66       B
		(1<<0)|(1<<2)|(1<<7)|(1<<9)|(1<<12)|(1<<14)|(1<<16)|(1<<19)|(1<<21)|(1<<27)|(1<<28)|(1<<29),
		//  67       C
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<28)|(1<<29),
		//  68       D
		(1<<0)|(1<<2)|(1<<7)|(1<<9)|(1<<13)|(1<<16)|(1<<20)|(1<<21)|(1<<25)|(1<<28),
		//  69       E
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<16)|(1<<21)|(1<<28)|(1<<29),
		//  70       F
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<16)|(1<<21),
		//  71       G
		(1<<0)|(1<<1)|(1<<2)|(1<<8)|(1<<9)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<25)|(1<<28),
		//  72       H
		(1<<2)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<27),
		//  73       I
		(1<<0)|(1<<1)|(1<<5)|(1<<11)|(1<<18)|(1<<24)|(1<<28)|(1<<29),
		//  74       J
		(1<<1)|(1<<8)|(1<<13)|(1<<20)|(1<<21)|(1<<25)|(1<<28),
		//  75       K
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<14)|(1<<12)|(1<<19)|(1<<8)|(1<<27),
		//  76       L
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<28)|(1<<29),
		//  77       M
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<0)|(1<<1)|(1<<5)|(1<<11),
		//  78       N
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<0)|(1<<7)|(1<<13)|(1<<20)|(1<<27),
		//  79       O
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<0)|(1<<1)|(1<<28)|(1<<29),
		//  80       P
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<0)|(1<<14)|(1<<7)|(1<<12),
		//  81       Q
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<0)|(1<<1)|(1<<28)|(1<<25)|(1<<26),
		//  82       R
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<0)|(1<<14)|(1<<7)|(1<<13)|(1<<15)|(1<<19)|(1<<27),
		//  83       S
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<15)|(1<<20)|(1<<27)|(1<<28)|(1<<29),
		//  84       T
		(1<<0)|(1<<1)|(1<<5)|(1<<11)|(1<<18)|(1<<24),
		//  85       U
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<28)|(1<<29),
		//  86       V
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<25)|(1<<28),
		//  87       W
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<25)|(1<<28)|(1<<18)|(1<<24),
		//  88       X
		(1<<2)|(1<<21)|(1<<10)|(1<<17)|(1<<12)|(1<<19)|(1<<8)|(1<<27),
		//  89       Y
		(1<<2)|(1<<10)|(1<<18)|(1<<12)|(1<<24)|(1<<8),
		//  90       Z
		(1<<0)|(1<<1)|(1<<8)|(1<<12)|(1<<17)|(1<<21)|(1<<28)|(1<<29),
		// 91
		(1<<0)|(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<28), //  91       [  // 0 2 9 16 21 28 // 1 8 13 20 27 29
		0x0, //  92       (backslash)
		(1<<1)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<29), //  93       ]
		0x0, //  94       ^
		(1<<28)|(1<<29), //  95       _
		0x0, //  96       `
		0x0, //  97       a
		0x0, //  98       b
		0x0, //  99       c
		0x0, // 100       d
		0x0, // 101       e
		0x0, // 102       f
		0x0, // 103       g
		0x0, // 104       h
		0x0, // 105       i
		0x0, // 106       j
		0x0, // 107       k
		0x0, // 108       l
		0x0, // 109       m
		0x0, // 110       n
		0x0, // 111       o
		0x0, // 112       p
		0x0, // 113       q
		0x0, // 114       r
		0x0, // 115       s
		0x0, // 116       t
		0x0, // 117       u
		0x0, // 118       v
		0x0, // 119       w
		0x0, // 120       x
		0x0, // 121       y
		0x0, // 122       z
		0x0, // 123       {
		0x0, // 124       |
		0x0, // 125       }
		0x0, // 126       ~
		0x0, // 127       ESC
	};
	return index[(int)c];
}

void TextBox::drawLine(point3d where, int startx, int starty, int offsetx, int offsety, double scale)
{
	double unit = scale/4;
	glVertex2f(where.x+startx*unit, where.y-starty*unit);
	glVertex2f(where.x+startx*unit+offsetx*unit, where.y-starty*unit-offsety*unit);
}

