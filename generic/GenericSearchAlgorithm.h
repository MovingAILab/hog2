/** An interface for generic search algorithms. 
* @file GenericSearchAlgorithm.h
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

#ifndef GENERICSEARCHALGORITHM_H
#define GENERICSEARCHALGORITHM_H

#include <vector>
#include "SearchEnvironment.h"
#include "StatCollection.h"

template <class state, class action, class environment>
class GenericSearchAlgorithm
{
public:
	GenericSearchAlgorithm() {};
	virtual ~GenericSearchAlgorithm() {};
	virtual void GetPath(environment *env, const state &from, const state &to, std::vector<state> &path) = 0;
	virtual void GetPath(environment *env, const state &from, const state &to, std::vector<action> &path) = 0;
	virtual bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath) { assert(false); return false; }
	virtual bool DoSingleSearchStep(std::vector<state> &thePath) { assert(false); return false; }
	virtual const char *GetName() = 0;
	virtual uint64_t GetNodesExpanded() const = 0;
	virtual uint64_t GetNodesTouched() const = 0;
	virtual void LogFinalStats(StatCollection *stats) = 0; 
//	virtual void OpenGLDraw() const {}
//	virtual void OpenGLDraw(const environment *env) const {}
};


#endif
