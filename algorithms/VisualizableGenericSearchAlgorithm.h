/** An interface for generic search algorithms with visualization.
*
* @file VisualizableGenericSearchAlgorithm.h
* @package hog2
* 
* @author Renee Jansen
* @date 06/27/2007
*
* This file is part of HOG2.
* HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
* HOG2: http://code.google.com/p/hog2/
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef VISUALIZABLEGENERICSEARCHALGORITHM_H
#define VISUALIZABLEGENERICSEARCHALGORITHM_H

#include "GenericSearchAlgorithm.h"

template <class state, class action>
class VisualizableGenericSearchAlgorithm : public GenericSearchAlgorithm
{
	VisualizableGenericSearchAlgorithm() {};
	virtual ~VisualizableGenericSearchAlgorithm() {};
	
	bool Initialize(SearchEnvironment<state, action> *env, state &start, state &goal);
	bool StepAlgorithm(std::vector<action> &path);
	bool StepAlgorithm(std::vector<state> &path);
//	void OpenGlDraw();
};

#endif
