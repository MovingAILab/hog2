//
//  Combinations.h
//
//  Created by Nathan Sturtevant on 11/9/18.
//  Distributed with MIT License
//

#ifndef Combinations_h
#define Combinations_h

constexpr uint64_t ConstFactorial(int n)
{
	return (n < 2)?1:(n*ConstFactorial(n-1));
}

constexpr uint64_t ConstChooseHelper(int n, int k)
{
	return (k > 0)?(n*ConstChooseHelper(n-1, k-1)):1;
	
}
constexpr uint64_t ConstChoose(int n, int k)
{	
	return (2*k > n)?(ConstChooseHelper(n, n-k)/ConstFactorial(n-k)):(ConstChooseHelper(n, k)/ConstFactorial(k));
}

template <int N>
class Combinations {
public:
	uint64_t MaxRank(int k) const { return ConstChoose(N, k); }
	uint64_t Rank(int *items, int k) const { return RankHelper(items, k, N, 0); }
	void Unrank(uint64_t hash, int *items, int k) const { return UnrankHelper(hash, items, k, N, N); }
private:
	uint64_t RankHelper(int *board, int count, int spaces, int offset) const
	{
		if (count == 0)
			return 0;
		if (board[0]-offset == 0)
			return RankHelper(&board[1], count-1, spaces-1, offset+1);
		uint64_t res = ConstChoose(spaces-1, count-1);
		return res+RankHelper(board, count, spaces-1, offset+1);
	}
	
	void UnrankHelper(uint64_t rank, int *board, int count, int spaces, int total) const
	{
		if (count == 0)
			return;
		uint64_t res = ConstChoose(spaces-1, count-1);
		if (rank >= res)
			UnrankHelper(rank-res, board, count, spaces-1, total);
		else {
			board[0] = total-spaces;
			UnrankHelper(rank, &board[1], count-1, spaces-1, total);
		}
	}

};


#endif /* Combinations_h */
