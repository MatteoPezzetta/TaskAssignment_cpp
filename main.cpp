#include <iostream>
#include <vector>
#include "V_Class.h"
#include "R_Class.h"
#include "Read_Assign.h"
#include "Resizable_Vector.h"
#include "Resizable_Array.h"
#include "Couplings.h"
#include "Edges.h"
#include "Opt_ass.h"
#include <time.h>

// Global variables
double Delta{ 5.0 };
double DeltaInf{ 10.0 };

extern unsigned __int64 N{};
extern unsigned __int64 num_v{};

int main()
{

    clock_t start, end;
    start = clock();

    // [READ DATA ABOUT REQUESTS AND VEHICLES]
    std::vector<Request> R;
    Read_Assign_Requests(R);
    N = R.size();
    //std::cout << "Number of tasks is: " << N << '\n';

    std::vector<Vehicle> V;
    Read_Assign_Vehicles(V);
    num_v = V.size();
    //std::cout << "Number of vehicles is: " << num_v << '\n';

    // [FIND MATCHING BETWEEN REQUESTS AND VEHICLES]
    Couplings coupling; // Generate object that contains info about the couplings;
    coupling.RV_Compute(R, V);


    // [FEASIBLE SET GENERATION]
    Edges trips; // Generate object that contains info about the edges (feasible trips);
    trips.RTV_Compute(coupling, R, V); // How to pass fewer arguments


    // [OPTIMAL ASSIGNMENT]
    Opt_ass opt_ass; // Generate object that contains info about the assignments;
    opt_ass.Optimal_Assignment(trips, R, V);
        
    end = clock();

    double time_taken = (double)(end - start) / CLOCKS_PER_SEC;
    std::cout << "Time taken by this program is: " << fixed << time_taken << setprecision(5);
    std::cout << " sec " << endl;

    return 0;

}