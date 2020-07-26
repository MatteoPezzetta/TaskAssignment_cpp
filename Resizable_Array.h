#ifndef RESIZABLE_ARRAY_H
#define RESIZABLE_ARRAY_H

#include <iostream>
#include <vector>

template <typename T>
class Resizable_Array
{
public:
	std::vector<T> Body;

	void Resize_Rows(__int64 N_rows);
	void Initialize(__int64 N_rows, T value);
};

template <typename T>
void Resizable_Array<T>::Resize_Rows(__int64 N_rows)
{
	for (int i{ 0 }; i < N_rows; ++i)
	{
		Body.resize(N_rows);
	}
}

template <typename T>
void Resizable_Array<T>::Initialize(__int64 N_rows, T value)
{
	for (int i{ 0 }; i < N_rows; ++i)
	{
		Body.push_back(value);
	}
}

#endif RESIZABLE_ARRAY_H