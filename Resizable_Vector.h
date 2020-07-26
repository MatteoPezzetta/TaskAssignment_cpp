#ifndef RESIZABLE_VECTOR_H
#define RESIZABLE_VECTOR_H

#include <iostream>
#include <vector>

template <typename T>
class Resizable_Vector
{
public:
	std::vector<std::vector<T>> Body;

	void Resize_Rows(int N_rows);
	void Resize_Body(int N_rows, int N_cols);
	void Initialize(int N_rows, T value);
};

template <typename T>
void Resizable_Vector<T>::Resize_Rows(int N_rows)
{
	std::vector<T> a;
	for (int i{ 0 }; i < N_rows; ++i)
	{
		Body.push_back(a);
	}
}

template <typename T>
void Resizable_Vector<T>::Resize_Body(int N_rows, int N_cols)
{
	std::vector<T> a;
	for (int i{ 0 }; i < N_rows; ++i)
	{
		Body.push_back(a);

		for (int j{ 0 }; j < N_cols; ++j)
		{
			Body[i].push_back(0);
		}
	}
}

template <typename T>
void Resizable_Vector<T>::Initialize(int N_rows, T value)
{
	std::vector<T> a;
	for (int i{ 0 }; i < N_rows; ++i)
	{
		Body.push_back(a);
		Body[i][0] = 0;
	}
}
/*
int main()
{

	Resizable_Vector Adj;

	Adj.Resize_Body(5, 5);

	std::cout << "Num of cols of Adj is: " << Adj.Body.size() << '\n';

	for (int i{ 0 }; i < Adj.Body.size(); ++i)
	{
		for (int j{ 0 }; j < Adj.Body[i].size(); ++j)
		{
			std::cout << Adj.Body[i][j] << " ";
		}
		std::cout << '\n';
	}

	return 0;
}
*/

#endif RESIZABLE_VECTOR_H