#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <array>
#include <vector>
#include "V_Class.h"
#include "R_Class.h"
#include "Read_Assign.h"

// This function reads from the requests list and assign those data to the Requests
void Read_Assign_Requests(std::vector<Request>& R)
{
    std::ifstream inFile; // ofstream: write only, ifstream: read only, fstream: both write and read
    inFile.open("C:\\Users\\Matteo\\Desktop\\C++\\Optimal_task_assignment_Classes\\Requests_Data.txt");

    if (!inFile)
    {
        std::cerr << "Unable to open file Requests_Data.txt";
        exit(1); // call system to stop
    }

    //int num_of_requests{ 0 };

    std::array<double, 10> a;

    std::string str;
    std::getline(inFile, str); // skip the first line

    std::string line;
    while (std::getline(inFile, line))
    {
        std::istringstream iss(line);

        int i{ 0 };
        while (!iss.eof())
        {
            if (!(iss))
                break;
            else
            {
                for (int j{ 0 }; j < 10; ++j)
                    iss >> a[j];
            }
            R.push_back({ a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], static_cast<int>(a[8]), static_cast<int>(a[9]) });
            //std::cout << a[0] << " " << a[1] << '\n';
            ++i;
        }

        //std::cout << a[0] << " " << a[1] << " " << a[2] << " " << a[3] << " " << a[4] << " " << a[5] << " " << a[6] << " " << a[7] << " " << a[8] << " " << a[9] << '\n';

        //++num_of_requests;
    }

    // Close the previously opened file.txt, reopen it, and clear its content
    //inFile.close();
    //inFile.open("C:\\Users\\Matteo\\GIT_projects\\ride_sharing_optimal_assignment_new_travel3\\Requests_Data2.txt", std::fstream::out | std::fstream::trunc);
    inFile.close();

    //std::cout << "Number of requests arrived = " << num_of_requests << '\n';
}

// This function reads from the vehicles list and assign those data to the Vehicles
void Read_Assign_Vehicles(std::vector<Vehicle>& V)
{
    std::ifstream inFile; // ofstream: write only, ifstream: read only, fstream: both write and read
    inFile.open("C:\\Users\\Matteo\\Desktop\\C++\\Optimal_task_assignment_Classes\\Vehicles_Data.txt");

    if (!inFile)
    {
        std::cerr << "Unable to open file Vehicles_Data.txt";
        exit(1); // call system to stop
    }

    //int num_of_requests{ 0 };

    std::array<double, 5> a;

    std::string str;
    std::getline(inFile, str); // skip the first line

    std::string line;
    while (std::getline(inFile, line))
    {
        std::istringstream iss(line);

        int i{ 0 };
        while (!iss.eof())
        {
            if (!(iss))
                break;
            else
            {
                for (int j{ 0 }; j < 5; ++j)
                    iss >> a[j];
            }
            V.push_back({ a[0], a[1], a[2], a[3], static_cast<int>(a[4]) });
            //std::cout << a[0] << " " << a[1] << '\n';
            ++i;
        }

        //std::cout << a[0] << " " << a[1] << " " << a[2] << " " << a[3] << " " << a[4] << " " << a[5] << " " << a[6] << " " << a[7] << " " << a[8] << " " << a[9] << '\n';

        //++num_of_requests;
    }

    // Close the previously opened file.txt, reopen it, and clear its content
    //inFile.close();
    //inFile.open("C:\\Users\\Matteo\\GIT_projects\\ride_sharing_optimal_assignment_new_travel3\\Vehicles_Data2.txt", std::fstream::out | std::fstream::trunc);
    inFile.close();

    //std::cout << "Number of requests arrived = " << num_of_requests << '\n';
}

// This function gets the maximum capacity within all the vehicles in the vehicles list
double get_max_cap(std::vector<Vehicle>& V)
{

    double max_cap{ 0.0 };

    for (unsigned __int64 i{ 0 }; i < V.size(); ++i) // Do I know vector sized at this point?
    {
        if (V[i].c > max_cap)
            max_cap = V[i].c;
    }

    return max_cap;
}

// This function write the title in the text file of the Requests
void write_title_R(std::ofstream& File)
{
    File << "Xo" << '\t' << "Yo" << '\t' << "Xd" << '\t' << "Yd" << '\t'
        << "Sta_T" << '\t' << "Arr_T" << '\t' << "Pass" << '\t' << "ID" << '\t'
        << "Status" << '\t' << "Priority" << '\n';
}

// This function write the title in the text file of the Vehicles
void write_title_V(std::ofstream& File)
{
    File << "X" << '\t' << "Y" << '\t' << "Cap" << '\t' << "ID" << '\t' << "Status" << '\n';
}

void write_title_Assignments_Size_One(std::ofstream& File)
{
    File << "Cost" << '\t' << "Vehicle" << '\t' << "Request" << '\t' << "Pass" << '\n';
}