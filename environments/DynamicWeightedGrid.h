//
//  DynamicWeightedGrid.h
//  Dynamic Weighted Abstraction
//
//  Created by Nathan Sturtevant on 5/16/19.
//  Copyright © 2019 University of Denver. All rights reserved.
//

#ifndef DynamicWeightedGrid_h
#define DynamicWeightedGrid_h

#include <stdio.h>
#include "Map2DEnvironment.h"

namespace DWG {
	
	enum TerrainType {
		kBlack = 0,//A
 		kDeepWater = 1,//B
		kWater = 2,//C
		kRiver = 3,//D
		kSwamp = 4,//E
		kGround = 5,//F
		kTrees = 6,//G
		kRoad = 7,//H
		kDesert = 8,//I
		kIce = 9,//J
		kJungle = 10,//K
		kPlains = 11,//L
		kBlight = 12,//M
		kTundra = 13,//N
		kMountain = 14,//O
		kHills = 15//P
	};
	
	struct abstractState {
		int sector, region;
	};
	
	static bool operator==(const abstractState &s1, const abstractState &s2)
	{
		return s1.sector == s2.sector && s2.region == s1.region;
	}
	
	struct edge {
//		edge(int f, int t) :from(f), to(t) {}
		int sectorFrom, regionFrom;
		int sectorTo, regionTo;
	};

static bool operator==(const edge &s1, const edge &s2)
{
	return s1.sectorFrom == s2.sectorFrom && s2.regionFrom == s1.regionFrom &&
	s1.sectorTo == s2.sectorTo && s2.regionTo == s1.regionTo;
}

	const uint8_t kNoRegion = 0xFF;
	
	struct regionData {
		xyLoc center;
		uint8_t terrain;
		uint16_t count;
	};
	
	template <int sectorSize>
	struct SectorData {
		uint8_t cells[sectorSize][sectorSize];
		uint8_t region[sectorSize][sectorSize];
		std::vector<edge> edges;
		std::vector<regionData> regionCenters;
	};
	
	// This is a dynamic weighted grid with an abstraction.
	// As a search environment it returns abstract states.
	template <int sectorSize>
	class DynamicWeightedGrid : public SearchEnvironment<abstractState, edge>
	{
	public:
		DynamicWeightedGrid(int width, int height);
		DynamicWeightedGrid(const char *map, int minRegionSize = 80);
		void SaveMap(const char *filename);
		int GetWidth() const;
		int GetHeight() const;
		void SetTerrainType(const xyLoc &l, TerrainType t);
		TerrainType GetTerrainType(const xyLoc &l) const;
		void GetSuccessors(const abstractState &nodeID, std::vector<abstractState> &neighbors) const;
		void GetActions(const abstractState &nodeID, std::vector<edge> &actions) const;
		void ApplyAction(abstractState &s, edge a) const;
		bool InvertAction(edge &a) const;
		edge GetAction(const abstractState &s1, const abstractState &s2) const;
		/** Heuristic value between two arbitrary nodes. **/
		double HCost(const abstractState &node1, const abstractState &node2) const;
		double GCost(const abstractState &node1, const abstractState &node2) const;
		double GCost(const abstractState &node, const edge &act) const;
		bool GoalTest(const abstractState &node, const abstractState &goal) const;

		uint64_t GetStateHash(const abstractState &node) const;
		uint64_t GetActionHash(edge act) const;
		
		void Draw(Graphics::Display &display) const;
		void Draw(Graphics::Display &display, const abstractState &) const;
//		void OpenGLDraw() const {}
//		void OpenGLDraw(const abstractState&) const {}
//		void OpenGLDraw(const abstractState&, const edge&) const { }
//		void GLDrawLine(const abstractState &x, const abstractState &y) const {}
		
		void GetPointFromCoordinate(point3d loc, int &px, int &py) const;
		void SetCosts(std::vector<double> &c)
		{
			costs = c;
			ValidateEdges();
		}
		void SetCost(TerrainType t, double cost)
		{
			costs[t] = cost;
		}
		std::vector<double> &GetCosts()
		{
			return costs;
		}
		abstractState GetState(const xyLoc &l);
		xyLoc GetLocation(const abstractState &a);
		uint64_t EstimateMemoryInBytes()
		{
			int numEdges = 0;
			int numSectors = sectors.size();
			int numRegions = 0;
			for (const auto &i : sectors)
			{
				numEdges += i.edges.size();
				numRegions += i.regionCenters.size();
			}
			// 8 bits per cell for type
			// 3 pointers in the sectors for regions, edges, and map data
			return mWidth*mHeight+numEdges*sizeof(edge)+numRegions+sizeof(regionData)+3*numSectors*sizeof(SectorData<sectorSize>*);
		}
		int GetNumEdges()
		{
			int numEdges = 0;
			for (const auto &i : sectors)
			{
				numEdges += i.edges.size();
			}
			return numEdges;
		}
		int GetNumRegions()
		{
			int numRegions = 0;
			for (const auto &i : sectors)
			{
				numRegions += i.regionCenters.size();
			}
			return numRegions;
		}
		void SetDrawAbstraction(bool draw) {drawAbstraction = draw;}
		bool GetDrawAbstraction() { return drawAbstraction; }
		void ValidateEdges() const;
	private:
		bool FindEdge(int fromSector, int fromRegion, int toSector, int toRegion) const;
		void DrawSector(Graphics::Display &display, int sector) const;

		void EliminateSmallRegions(int limit);
		void BuildAbstraction();
		void GetRegions(int sector);
		void GetEdges(int sector);
		regionData BFS(SectorData<sectorSize> &d, int x, int y, int whichRegion);
		void AddEdge(SectorData<sectorSize> &d, int s1, int r1, int s2, int r2);

		int GetNumSectors() const;
		int GetNumXSectors() const;
		int GetNumYSectors() const;
		int GetRegion(const xyLoc &l);
		int GetSector(const xyLoc &l) const;
		void GetSectorOffset(const xyLoc &l, int &x, int &y) const;
		void SetTerrainTypeNoRepair(const xyLoc &l, TerrainType t);
		std::vector<SectorData<sectorSize>> sectors;
		int mWidth, mHeight;
		bool drawAbstraction;
		std::vector<double> costs;
	public:
		void GetCoordinate(const xyLoc &l, float &x, float &y, float &r) const;
		static rgbColor GetTerrainColor(TerrainType t)
		{
			uint8_t val = (uint8_t)t;
			if (val >= 16)
				val = 15;
//			return rgbColor(val/15.0, val/15.0, val/15.0);
			switch (t)
			{
				case kBlack: return Colors::black;
				case kGround: return Colors::brown;
				case kTrees: return  Colors::darkgreen;
				case kSwamp: return  Colors::bluegreen;
				case kRoad: return  Colors::yellow;
				case kWater: return  Colors::blue;
				case kDeepWater: return Colors::darkblue;
				case kRiver: return Colors::lightblue;
				case kDesert: return Colors::lightyellow;
				case kIce: return (Colors::white+Colors::cyan);
				case kJungle: return Colors::darkbluegray;
				case kPlains: return Colors::lightbrown;
				case kBlight: return Colors::darkyellow;
				case kTundra: return Colors::lightgray;
				case kMountain: return Colors::darkgray;
				case kHills: return Colors::green;
				default: return Colors::orange;
			}
		}

	};

#pragma mark -
#pragma mark DynamicWeightedGridEnvironment
#pragma mark -

	// This is just a dynamic weighted grid
	class DynamicWeightedGridEnvironment : public SearchEnvironment<xyLoc, tDirection> {
	public:
		DynamicWeightedGridEnvironment(const char *map);
		DynamicWeightedGridEnvironment(int width, int height);
		void GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
		void GetActions(const xyLoc &nodeID, std::vector<tDirection> &actions) const;
		void ApplyAction(xyLoc &s, tDirection a) const;
		bool InvertAction(tDirection &a) const;
		tDirection GetAction(const xyLoc &s1, const xyLoc &s2) const;
		/** Heuristic value between two arbitrary nodes. **/
		double HCost(const xyLoc &node1, const xyLoc &node2) const;
		double GCost(const xyLoc &node1, const xyLoc &node2) const;
		double GCost(const xyLoc &node, const tDirection &act) const;
		bool GoalTest(const xyLoc &node, const xyLoc &goal) const;
		
		uint64_t GetStateHash(const xyLoc &node) const;
		uint64_t GetMaxHash() const { return mWidth*mHeight; }
		uint64_t GetActionHash(tDirection act) const;
		
		void Draw(Graphics::Display &display) const;
		void Draw(Graphics::Display &display, const xyLoc &) const;
//		void OpenGLDraw() const {}
//		void OpenGLDraw(const xyLoc&) const {}
//		void OpenGLDraw(const xyLoc&, const tDirection&) const {}
//		void GLDrawLine(const xyLoc &x, const xyLoc &y) const {}
		void SetCosts(std::vector<double> &c)
		{
			costs = c;
		}
		void SetCost(TerrainType t, double cost)
		{
			costs[t] = cost;
		}
		std::vector<double> &GetCosts()
		{
			return costs;
		}
		void SetTerrainType(const xyLoc &l, TerrainType t)
		{
			terrain[l.y*mWidth+l.x] = t;
		}
	private:
		void GetCoordinate(const xyLoc &l, float &x, float &y, float &r) const;
		std::vector<double> costs;
		std::vector<uint8_t> terrain;
		int mWidth, mHeight;
	};

	
#pragma mark -
#pragma mark DynamicWeightedGrid Implementation
#pragma mark -

	
	template <int sectorSize>
	DynamicWeightedGrid<sectorSize>::DynamicWeightedGrid(int width, int height)
	:mWidth(width), mHeight(height)
	{
		drawAbstraction = false;
		sectors.resize(GetNumSectors());
		for (uint16_t y = 0; y < mHeight; y++)
		{
			for (uint16_t x = 0; x < mWidth; x++)
			{
				SetTerrainTypeNoRepair({x, y}, kGround);
			}
		}
		BuildAbstraction();
	}

	template <int sectorSize>
	DynamicWeightedGrid<sectorSize>::DynamicWeightedGrid(const char *map, int minRegionSize)
	{
		drawAbstraction = false;
		FILE *f = fopen(map, "r");
		if (f == 0)
		{
			printf("Unable to open file '%s'\n", map);
			exit(0);
		}
		char format[32];
		int num = fscanf(f, "type %s\nheight %d\nwidth %d\nmap\n", format, &mHeight, &mWidth);
		if (num != 3)
		{
			printf("Bad file format: '%s'\n", map);
			exit(0);
		}
		sectors.resize(GetNumSectors());
		char what;
		for (uint16_t y = 0; y < mHeight; y++)
		{
			for (uint16_t x = 0; x < mWidth; x++)
			{
				fscanf(f, "%c", &what);
				SetTerrainTypeNoRepair({x, y}, (DWG::TerrainType)(what-'A'));
			}
			fscanf(f, "%c", &what);
			if (what != '\n')
				printf("Error loading\n");
		}
		EliminateSmallRegions(minRegionSize);
		BuildAbstraction();
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::SaveMap(const char *filename)
	{
		FILE *f = fopen(filename, "w");
		if (f == 0)
		{
			printf("Error opening '%s'\n", filename);
			return;
		}
		fprintf(f, "type octile\nheight %d\nwidth %d\nmap\n", mHeight, mWidth);
		for (uint16_t y = 0; y < mHeight; y++)
		{
			for (uint16_t x = 0; x < mWidth; x++)
			{
				fprintf(f, "%c", 'A'+(char)GetTerrainType({x, y}));
			}
			fprintf(f, "\n");
		}
		fclose(f);
	}
	
	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetWidth() const
	{ return mWidth; }

	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetHeight() const
	{ return mHeight; }

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::SetTerrainType(const xyLoc &l, TerrainType t)
	{
		int sec = GetSector(l);
		int x, y;
		GetSectorOffset(l, x, y);
		if (sectors[sec].cells[x][y] != t)
		{
			sectors[sec].cells[x][y] = t;
			GetRegions(sec);
			GetEdges(sec);
			GetEdges(sec-1);
			GetEdges(sec+1);
			GetEdges(sec-GetNumXSectors());
			GetEdges(sec-GetNumXSectors()+1);
			GetEdges(sec-GetNumXSectors()-1);
			GetEdges(sec+GetNumXSectors());
			GetEdges(sec+GetNumXSectors()+1);
			GetEdges(sec+GetNumXSectors()-1);
		}
	}

	template <int sectorSize>
	TerrainType DynamicWeightedGrid<sectorSize>::GetTerrainType(const xyLoc &l) const
	{
		int sec = GetSector(l);
		int x, y;
		GetSectorOffset(l, x, y);
		return static_cast<TerrainType>(sectors[sec].cells[x][y]);
	}


	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::BuildAbstraction()
	{
		for (int x = 0; x < GetNumSectors(); x++)
		{
			GetRegions(x);
		}
		for (int x = 0; x < GetNumSectors(); x++)
		{
			GetEdges(x);
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetRegions(int sector)
	{
		SectorData<sectorSize> &d = sectors[sector];
		d.regionCenters.clear();
		int regionOffsetX = (sector%GetNumXSectors())*sectorSize;
		int regionOffsetY = (sector/GetNumXSectors())*sectorSize;

		for (int x = 0; x < sectorSize; x++)
		{
			for (int y = 0; y < sectorSize; y++)
			{
				d.region[x][y] = kNoRegion;
			}
		}
		int whichRegion = 0;
		for (int x = 0; x < sectorSize; x++)
		{
			for (int y = 0; y < sectorSize; y++)
			{
				if (d.region[x][y] == kNoRegion)
				{
					d.regionCenters.push_back(BFS(d, x, y, whichRegion));
					d.regionCenters.back().center.x += regionOffsetX;
					d.regionCenters.back().center.y += regionOffsetY;
					whichRegion++;
				}
			}
		}
	}

	template <int sectorSize>
	regionData DynamicWeightedGrid<sectorSize>::BFS(SectorData<sectorSize> &d, int x, int y, int whichRegion)
	{
		int avgx = 0, avgy = 0;
		float count = 0;
		static std::vector<std::pair<int, int>> inRegion;
		static std::vector<std::pair<int, int>> stack;
		inRegion.clear();
		stack.clear();
		stack.push_back({x, y});
		while (stack.size() > 0)
		{
			auto i = stack.back();
			stack.pop_back();
			if (d.region[i.first][i.second] == kNoRegion)
			{
				d.region[i.first][i.second] = whichRegion;
				inRegion.push_back({i.first, i.second});
				count++;
				avgx += i.first;
				avgy += i.second;
				if (i.first+1 < sectorSize && (d.cells[i.first][i.second] == d.cells[i.first+1][i.second]))
					stack.push_back({i.first+1, i.second});
				if (i.first > 0 && (d.cells[i.first][i.second] == d.cells[i.first-1][i.second]))
					stack.push_back({i.first-1, i.second});
				if (i.second+1 < sectorSize && (d.cells[i.first][i.second] == d.cells[i.first][i.second+1]))
					stack.push_back({i.first, i.second+1});
				if (i.second > 0 && (d.cells[i.first][i.second] == d.cells[i.first][i.second-1]))
					stack.push_back({i.first, i.second-1});
			}
		}
		xyLoc middle(avgx/count, avgy/count);
		xyLoc best(inRegion[0].first, inRegion[0].second);
		for (auto &loc : inRegion)
		{
			if ((loc.first-middle.x)*(loc.first-middle.x)+(loc.second-middle.y)*(loc.second-middle.y) <
				(best.x-middle.x)*(best.x-middle.x)+(best.y-middle.y)*(best.y-middle.y))
			{
				best.x = loc.first;
				best.y = loc.second;
			}
		}
		return {best, static_cast<uint8_t>(d.cells[x][y]), static_cast<uint16_t>(count)};
	}

	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetEdges(int sector)
	{
		if (sector < 0 || sector >= GetNumSectors())
			return;
		SectorData<sectorSize> &d = sectors[sector];
		d.edges.clear();
		
		int regionOffsetX = (sector%GetNumXSectors())*sectorSize;
		int regionOffsetY = (sector/GetNumXSectors())*sectorSize;
		
		for (int x = 0; x < sectorSize; x++)
		{
			for (int y = 0; y < sectorSize; y++)
			{
				xyLoc middle(regionOffsetX+x, regionOffsetY+y);
				int r = GetRegion(middle);
				int s = GetSector(middle);
				
				for (int x1 = -1; x1 <= 1; x1++)
				{
					for (int y1 = -1; y1 <= 1; y1++)
					{
						xyLoc next(middle.x+x1, middle.y+y1);
						if (next.x >= mWidth || next.y >= mHeight)
							continue;
						int r1 = GetRegion(next);
						int s1 = GetSector(next);
						if (r1 != r || s1 != s)
						{
							AddEdge(d, s, r, s1, r1);
						}
					}
				}
			}
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::AddEdge(SectorData<sectorSize> &d, int s1, int r1, int s2, int r2)
	{
		for (auto &e : d.edges)
		{
			if (e.sectorFrom == s1 && e.sectorTo == s2 && e.regionFrom == r1 && e.regionTo == r2)
				return;
		}
		edge e = {s1, r1, s2, r2};
		d.edges.push_back(e);
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::ValidateEdges() const
	{
		bool success = true;
		for (const auto &s : sectors)
		{
			for (const auto &e : s.edges)
			{
				bool result = FindEdge(e.sectorTo, e.regionTo, e.sectorFrom, e.regionFrom);
				abstractState f = {e.sectorFrom, e.regionFrom};
				abstractState t = {e.sectorTo, e.regionTo};
				assert(fequal(HCost(f, t), HCost(t, f)));
				if (result == false)
				{
					printf("Found edge from %d:%d to %d:%d, but missing reverse edge\n", e.sectorFrom, e.regionFrom, e.sectorTo, e.regionTo);
					success = false;
				}
			}
		}
		if (success)
			printf("Edges validated\n");
		else
			printf("Edges did not validate\n");
	}

	template <int sectorSize>
	bool DynamicWeightedGrid<sectorSize>::FindEdge(int fromSector, int fromRegion, int toSector, int toRegion) const
	{
		for (const auto &e : sectors[fromSector].edges)
			if (e.sectorFrom == fromSector && e.regionFrom == fromRegion &&
				e.sectorTo == toSector && e.regionTo == toRegion)
				return true;
		return false;
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::EliminateSmallRegions(int limit)
	{
		std::vector<int> regionCount;
		std::vector<int32_t> regions(mWidth*mHeight, -1);
		int whichRegion = 0;
		for (int x = 0; x < mWidth; x++)
		{
			for (int y = 0; y < mHeight; y++)
			{
				if (regions[y*mWidth+x] == -1)
				{
					regionCount.resize(regionCount.size()+1);
					std::vector<xyLoc> stack;
					stack.push_back({static_cast<uint16_t>(x), static_cast<uint16_t>(y)});
					while (stack.size() > 0)
					{
						xyLoc i = stack.back();
						stack.pop_back();
						if (regions[i.y*mWidth+i.x] == -1)
						{
							regions[i.y*mWidth+i.x] = whichRegion;
							regionCount[whichRegion]++;
							xyLoc tmp;
							tmp = i; tmp.x++;
							if (tmp.x < mWidth && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
							tmp = i; tmp.x--;
							if (i.x > 0 && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
							tmp = i; tmp.y++;
							if (tmp.y < mHeight && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
							tmp = i; tmp.y--;
							if (i.y > 0 && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
						}
					}
					whichRegion++;
				}
			}
		}
		printf("%d regions after first BFS\n", whichRegion);
		for (int x = 0; x < mWidth; x++)
		{
			for (int y = 0; y < mHeight; y++)
			{
				if (regionCount[regions[y*mWidth+x]] > limit)
				{
					xyLoc i(x, y), tmp;
					tmp = i; tmp.x++;
					if (tmp.x < mWidth && (regionCount[regions[tmp.y*mWidth+tmp.x]]) <= limit)
					{
						SetTerrainTypeNoRepair(tmp, GetTerrainType(i));
						regions[tmp.y*mWidth+tmp.x] = regions[y*mWidth+x];
					}
					tmp = i; tmp.x--;
					if (i.x > 0 && (regionCount[regions[tmp.y*mWidth+tmp.x]]) <= limit)
					{
						SetTerrainTypeNoRepair(tmp, GetTerrainType(i));
						regions[tmp.y*mWidth+tmp.x] = regions[y*mWidth+x];
					}
					tmp = i; tmp.y++;
					if (tmp.y < mHeight && (regionCount[regions[tmp.y*mWidth+tmp.x]]) <= limit)
					{
						SetTerrainTypeNoRepair(tmp, GetTerrainType(i));
						regions[tmp.y*mWidth+tmp.x] = regions[y*mWidth+x];
					}
					tmp = i; tmp.y--;
					if (i.y > 0 && (regionCount[regions[tmp.y*mWidth+tmp.x]]) <= limit)
					{
						SetTerrainTypeNoRepair(tmp, GetTerrainType(i));
						regions[tmp.y*mWidth+tmp.x] = regions[y*mWidth+x];
					}
				}
			}
		}
		regionCount.resize(0);
		for (auto &x : regions)
			x = -1;
		whichRegion = 0;
		for (int x = 0; x < mWidth; x++)
		{
			for (int y = 0; y < mHeight; y++)
			{
				if (regions[y*mWidth+x] == -1)
				{
					regionCount.resize(regionCount.size()+1);
					std::vector<xyLoc> stack;
					stack.push_back({static_cast<uint16_t>(x), static_cast<uint16_t>(y)});
					while (stack.size() > 0)
					{
						xyLoc i = stack.back();
						stack.pop_back();
						if (regions[i.y*mWidth+i.x] == -1)
						{
							regions[i.y*mWidth+i.x] = whichRegion;
							regionCount[whichRegion]++;
							xyLoc tmp;
							tmp = i; tmp.x++;
							if (tmp.x < mWidth && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
							tmp = i; tmp.x--;
							if (i.x > 0 && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
							tmp = i; tmp.y++;
							if (tmp.y < mHeight && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
							tmp = i; tmp.y--;
							if (i.y > 0 && (GetTerrainType(i) == GetTerrainType(tmp)))
								stack.push_back(tmp);
						}
					}
					whichRegion++;
				}
			}
		}
		printf("%d regions after second BFS\n", whichRegion);
//		for (int x = 0; x < regionCount.size(); x++)
//			printf("%d : %d\n", x, regionCount[x]);
	}

	template <int sectorSize>
	abstractState DynamicWeightedGrid<sectorSize>::GetState(const xyLoc &l)
	{
		abstractState s;
		s.sector = GetSector(l);
		s.region = GetRegion(l);
		return s;
	}

	template <int sectorSize>
	xyLoc DynamicWeightedGrid<sectorSize>::GetLocation(const abstractState &a)
	{
		return sectors[a.sector].regionCenters[a.region].center;
	}

	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetNumSectors() const
	{
		return ((mWidth+sectorSize-1)/sectorSize)*((mHeight+sectorSize-1)/sectorSize);
		
	}
	
	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetNumXSectors() const
	{
		return ((mWidth+sectorSize-1)/sectorSize);
		
	}
	
	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetNumYSectors() const
	{
		return ((mHeight+sectorSize-1)/sectorSize);
		
	}
	
	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetRegion(const xyLoc &l)
	{
		int x, y;
		GetSectorOffset(l, x, y);
		return sectors[GetSector(l)].region[x][y];
	}
	
	template <int sectorSize>
	int DynamicWeightedGrid<sectorSize>::GetSector(const xyLoc &l) const
	{
		int secx = l.x/sectorSize;
		int secy = l.y/sectorSize;
		return secy*GetNumXSectors()+secx;
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetSectorOffset(const xyLoc &l, int &x, int &y) const
	{
		x = l.x%sectorSize;
		y = l.y%sectorSize;
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::SetTerrainTypeNoRepair(const xyLoc &l, TerrainType t)
	{
		int sec = GetSector(l);
		int x, y;
		GetSectorOffset(l, x, y);
		if (sectors[sec].cells[x][y] != t)
		{
			sectors[sec].cells[x][y] = t;
		}
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetSuccessors(const abstractState &nodeID, std::vector<abstractState> &neighbors) const
	{
		neighbors.clear();
		const SectorData<sectorSize> &d = sectors[nodeID.sector];
		for (auto e : d.edges)
		{
			if (e.regionFrom != nodeID.region)
				continue;
			neighbors.push_back({e.sectorTo, e.regionTo});
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetActions(const abstractState &nodeID, std::vector<edge> &actions) const
	{
		actions.clear();
		const SectorData<sectorSize> &d = sectors[nodeID.sector];
		for (auto e : d.edges)
		{
			if (e.regionFrom != nodeID.region)
				continue;
			actions.push_back(e);
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::ApplyAction(abstractState &s, edge a) const
	{
		s.sector = a.sectorTo;
		s.region = a.regionTo;
	}

	template <int sectorSize>
	bool DynamicWeightedGrid<sectorSize>::InvertAction(edge &a) const
	{
		std::swap(a.sectorFrom, a.sectorTo);
		std::swap(a.regionFrom, a.regionTo);
		return true;
	}

	template <int sectorSize>
	edge DynamicWeightedGrid<sectorSize>::GetAction(const abstractState &s1, const abstractState &s2) const
	{
		assert(!"Not implemented");
		return edge();
	}

	template <int sectorSize>
	double DynamicWeightedGrid<sectorSize>::HCost(const abstractState &n1, const abstractState &n2) const
	{
		xyLoc l1 = sectors[n1.sector].regionCenters[n1.region].center;
		xyLoc l2 = sectors[n2.sector].regionCenters[n2.region].center;
		
		const double DIAGONAL_COST = M_SQRT2;
		double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
		double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
		//		if (a == 0 && b == 0)
		//			return 0;
		return ((a>b)?(b*DIAGONAL_COST+a-b):(a*DIAGONAL_COST+b-a));
	}

	template <int sectorSize>
	double DynamicWeightedGrid<sectorSize>::GCost(const abstractState &n1, const abstractState &n2) const
	{
		xyLoc l1 = sectors[n1.sector].regionCenters[n1.region].center;
		xyLoc l2 = sectors[n2.sector].regionCenters[n2.region].center;
		
		const double DIAGONAL_COST = M_SQRT2;
		double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
		double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
		//		if (a == 0 && b == 0)
		//			return 0;
		double c1 = costs[sectors[n1.sector].regionCenters[n1.region].terrain];
		double c2 = costs[sectors[n2.sector].regionCenters[n2.region].terrain];
		return 0.5*(c1+c2)*((a>b)?(b*DIAGONAL_COST+a-b):(a*DIAGONAL_COST+b-a));
	}

	template <int sectorSize>
	double DynamicWeightedGrid<sectorSize>::GCost(const abstractState &node, const edge &act) const
	{
		abstractState tmp = node;
		ApplyAction(tmp, act);
		return GCost(node, tmp);
	}

	template <int sectorSize>
	bool DynamicWeightedGrid<sectorSize>::GoalTest(const abstractState &node, const abstractState &goal) const
	{
		return node == goal;
	}

	template <int sectorSize>
	uint64_t DynamicWeightedGrid<sectorSize>::GetStateHash(const abstractState &node) const
	{
		uint64_t a = node.sector;
		uint64_t b = node.region;
		return (a<<32)|b;
	}

	template <int sectorSize>
	uint64_t DynamicWeightedGrid<sectorSize>::GetActionHash(edge act) const
	{
		assert(!"not implemented");
		return 0;
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetCoordinate(const xyLoc &l, float &x, float &y, float &radius) const
	{
		float _scale, xOffset, yOffset;
		if (mHeight > mWidth)
		{
			_scale = 2.0/(float)(mHeight);
			xOffset = (2.0-mWidth*_scale)*0.5;
			yOffset = 0;
		}
		else {
			_scale = 2.0/(float)(mWidth);
			yOffset = (2.0-mHeight*_scale)*0.5;
			xOffset = 0;
		}
		float epsilon = _scale/2.0;
		x = -1+l.x*_scale+epsilon+xOffset;
		y = -1+l.y*_scale+epsilon+yOffset;
		//	x = (2*_x-width)*_scale+epsilon;
		//	y = (2*_y-height)*_scale+epsilon;
		radius = epsilon;
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetPointFromCoordinate(point3d loc, int &px, int &py) const
	{
		double _x, _y;
		double _scale, xOffset, yOffset;
		if (mHeight > mWidth)
		{
			_scale = 2.0/(double)(mHeight);
			xOffset = (2.0-mWidth*_scale)*0.5;
			yOffset = 0;
		}
		else {
			_scale = 2.0/(double)(mWidth);
			yOffset = (2.0-mHeight*_scale)*0.5;
			xOffset = 0;
		}
		double epsilon = _scale/2.0;
		
		_x = (loc.x-epsilon+1-xOffset)/_scale;
		_y = (loc.y-epsilon+1-yOffset)/_scale;
		
		px = (int)(_x+0.5); // round off!
		py = (int)(_y+0.5);
		if ((px < 0) || (py < 0) || (px >= mWidth) || (py >= mHeight))
		{
			px = py = -1;
		}
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::DrawSector(Graphics::Display &display, int sector) const
	{
		auto &s = sectors[sector];
		int regionOffsetX = (sector%GetNumXSectors())*sectorSize;
		int regionOffsetY = (sector/GetNumXSectors())*sectorSize;
		uint16_t largest = 0;
		for (size_t x = 1; x < s.regionCenters.size(); x++)
		{
			if (s.regionCenters[x].count > s.regionCenters[largest].count)
				largest = x;
		}
		
		float x, y, r;
		xyLoc l(regionOffsetX, regionOffsetY);
		GetCoordinate(l, x, y, r);
		//r=r*sectorSize*0.5f;
		display.FillSquare({x-r+r*sectorSize, y-r+r*sectorSize}, r*sectorSize,
						   GetTerrainColor((TerrainType)s.regionCenters[largest].terrain));
		for (int yy = 0; yy < sectorSize; yy++)
		{
			int startx = 0;
			int starty = yy;
			for (int xx = 0; xx < sectorSize; xx++)
			{
				// Same type of cell
				if (s.cells[xx][yy] == s.cells[startx][starty])
					continue;
				
				// draw previous area
				if (s.cells[startx][starty] != (TerrainType)s.regionCenters[largest].terrain)
				{
					l.x = regionOffsetX+startx;
					l.y = regionOffsetY+starty;
					GetCoordinate(l, x, y, r);
					display.FillRect({x-r, y-r, x-r+2*r*(xx-startx), y+r},
									 GetTerrainColor((TerrainType)s.cells[startx][starty]));
				}
				startx = xx;
			}
			if (startx < sectorSize)
			{
				if (s.cells[startx][starty] != (TerrainType)s.regionCenters[largest].terrain)
				{
					l.x = regionOffsetX+startx;
					l.y = regionOffsetY+starty;
					GetCoordinate(l, x, y, r);
					display.FillRect({x-r, y-r, x-r+2*r*(sectorSize-startx), y+r},
									 GetTerrainColor((TerrainType)s.cells[startx][starty]));
				}
			}
		}
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::Draw(Graphics::Display &display) const
	{
		for (int t = 0; t < sectors.size(); t++)
		{
			DrawSector(display, t);
		}
		if (!drawAbstraction)
			return;
		
		// Draw sector boundaries
		for (int x = 0; x < mWidth; x+=sectorSize)
		{
			float xx1, yy1, rr, xx2, yy2;
			xyLoc l1(x, 0), l2(x, mHeight);
			GetCoordinate(l1, xx1, yy1, rr);
			GetCoordinate(l2, xx2, yy2, rr);
			display.DrawLine({xx1-rr, yy1-rr}, {xx2-rr, yy2-rr}, rr*0.5f, Colors::darkgray);
		}
		for (int y = 0; y < mHeight; y+=sectorSize)
		{
			float xx1, yy1, rr, xx2, yy2;
			xyLoc l1(0, y), l2(mWidth, y);
			GetCoordinate(l1, xx1, yy1, rr);
			GetCoordinate(l2, xx2, yy2, rr);
			display.DrawLine({xx1-rr, yy1-rr}, {xx2-rr, yy2-rr}, rr*0.5f, Colors::darkgray);
		}
		// Draw edges in sectors
		for (int s = 0; s < GetNumSectors(); s++)
		{
			const SectorData<sectorSize> &d = sectors[s];
			for (auto i : d.regionCenters)
			{
				float x, y, r;
				GetCoordinate(i.center, x, y, r);
				display.FillCircle({x, y}, r, Colors::gray);
			}
			for (auto e : d.edges)
			{
				xyLoc l1 = sectors[e.sectorFrom].regionCenters[e.regionFrom].center;
				xyLoc l2 = sectors[e.sectorTo].regionCenters[e.regionTo].center;
				
				float xx1, yy1, rr, xx2, yy2;
				GetCoordinate(l1, xx1, yy1, rr);
				GetCoordinate(l2, xx2, yy2, rr);
				display.DrawLine({xx1, yy1}, {xx2, yy2}, rr*0.5f, Colors::green);
			}
		}
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::Draw(Graphics::Display &display, const abstractState &absState) const
	{
		xyLoc l = sectors[absState.sector].regionCenters[absState.region].center;
		float x, y, r;
		GetCoordinate(l, x, y, r);
		display.FillCircle({x, y}, 4*r, GetColor());
	}
	

}
#endif /* DynamicWeightedGrid_h */
