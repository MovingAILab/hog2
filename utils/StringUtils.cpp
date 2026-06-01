#include "StringUtils.h"
#include <cmath>

std::string int_to_string(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

std::string double_to_string(double i) {
	std::stringstream s;
	s << i;
	return s.str();
}

std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	std::stringstream ss(s);
	std::string item;
	while(std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}

std::string to_string_separator(int i)
{
	if (i == 0)
		return std::string("0");
	if (i < 0)
		return std::string("-")+to_string_separator(-i);
	std::string tmp;
	int numdigits = floor(log10(i))+1;
	int divisor = pow(10, numdigits-1);
	int currdigit = 0;
	while (currdigit != numdigits)
	{
		tmp += ('0' + (i/divisor)%10);
		currdigit++;
		divisor/= 10;
		if ((0 == (numdigits-currdigit)%3) && (numdigits != currdigit))
			tmp += ',';
	}
	return tmp;
}
