#ifndef OPT_ASS_H
#define OPT_ASS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include "Data_Structures.h"
#include "Resizable_Array.h"
#include <ilcplex/ilocplex.h>
#include <ilconcert/iloexpression.h>
#include "cplex_modelling.h"
#include "V_Class.h"
#include "R_Class.h"
#include "Read_Assign.h"
#include "Edges.h"

ILOSTLBEGIN

extern unsigned __int64 N;
extern unsigned __int64 num_v;

class Opt_ass
{

    public:

        class assignment {

            public:

                std::vector<double> tasks;
                std::vector<double> pick_up;
                std::vector<double> drop_off;
                double vehicle;
                std::vector<double> R_i;
                double V_i;
                int dim;
                double pass;
                int status;
                int priority;
                double cost;

                assignment(double cost_time, std::vector<double>& task_set_PU, std::vector<double>& task_set_DO, double veh,
                    double passengers, std::vector<Request>& R, std::vector<Vehicle>& V);

                int size_of_trip(std::vector<double>& task_set);

        };

        std::vector<Opt_ass::assignment> assignments; // definition of the assignment objects, that are inside the object opt_ass

        Resizable_Array <int> Init_guess;
        Resizable_Array <int> X_k_init;

        std::vector<std::vector<double>> e3_save;
        std::vector<std::vector<double>> e2_save;
        std::vector<std::vector<double>> e1_save;

        std::vector<double> R_ass;
        std::vector<double> V_ass;

        std::vector<double> priority_tasks;
        std::vector<double> new_vehicles;
        std::vector<std::vector<double>> priority_ass;

        // void LP_Prioririty(std::vector<

        void Optimal_Assignment(Edges& trips, std::vector<Request>& R, std::vector<Vehicle>& V);

    private:

        void Priority_Assignment(std::vector<double>& priority_tasks, std::vector<double>& new_vehicles, std::vector<Request>& R, std::vector<Vehicle>& V, std::vector<std::vector<double>>& e1);
        void Greedy_Assignment(std::vector<std::vector<double>>& e3, std::vector<std::vector<double>>& e2, std::vector<std::vector<double>>& e1, std::vector<Request>& R);
        void MILP_LP_Assignment(std::vector<std::vector<int>>& RTV_Adj, int tot, std::vector<Request>& R, std::vector<Vehicle>& V, int size1_exist, std::vector<std::vector<double>>& size1_assigned);
        void LP_Rebalancing(std::vector<Request>& R, std::vector<Vehicle>& V);

        void print_size_one(std::vector<Request>& R, std::vector<Vehicle>& V);
        bool check_priority(std::vector<Request>& R, std::vector<Vehicle>& V);

        std::vector<std::vector<int>> MatrixProduct(std::vector<std::vector<int>>& Matrix_1, std::vector<std::vector<int>>& Matrix_2, unsigned __int64 M1_rows, unsigned __int64 M2_cols, unsigned __int64 M1_cols);
        
};

void Opt_ass::Optimal_Assignment(Edges& trips, std::vector<Request>& R, std::vector<Vehicle>& V)
{
    // --> check for presence of priority, if exist, save the priority tasks indeces
    // --> check for availability of vehicles
    // --> call Priority_Assignment();
 
    if (check_priority(R, V))
        Priority_Assignment(priority_tasks, new_vehicles, R, V, trips.e1);

    Greedy_Assignment(trips.e3, trips.e2, trips.e1, R);
    MILP_LP_Assignment(trips.RTV_Adj.Body, trips.tot, R, V, trips.size1_exist, trips.size1_assigned);
}

// This function is by the most equal to the LP_Rebalancing function. I can think of reusing the LP_Rebalancing function with new arguments
void Opt_ass::Priority_Assignment(std::vector<double>& priority_tasks, std::vector<double>& new_vehicles, std::vector<Request>& R, std::vector<Vehicle>& V, std::vector<std::vector<double>>& e1)
{
    unsigned __int64 L_priority_tasks{ priority_tasks.size() };
    unsigned __int64 L_new_vehicles{ new_vehicles.size() };

    int v_indice{};
    int r_indice{};

    std::array<double, 2> vehicle_pose;
    std::array<double, 2> request_orgn;
    std::array<double, 2> request_dest;

    std::vector<double> t_cost;

    std::vector<std::vector<double>> e_prt;

    std::vector<std::vector<int>> A_prt_in(L_priority_tasks + L_new_vehicles, std::vector<int>(L_priority_tasks * L_new_vehicles));

    int r_count{ 0 };

    for (unsigned __int64 v{ 0 }; v < L_new_vehicles; ++v)
    {
        for (unsigned __int64 r{ 0 }; r < L_priority_tasks; ++r)
        {
            v_indice = static_cast<int>(new_vehicles[v]);
            r_indice = static_cast<int>(priority_tasks[r]);

            vehicle_pose = { V[v_indice].x, V[v_indice].y };
            request_orgn = { R[r_indice].xo, R[r_indice].yo };
            request_dest = { R[r_indice].xd, R[r_indice].yd };

            t_cost.push_back(tt(vehicle_pose, request_orgn) + tt(request_orgn, request_dest));

            e_prt.push_back({ t_cost[r_count], V[v_indice].ID, R[r_indice].ID, R[r_indice].pass });

            A_prt_in[v][r + v * L_priority_tasks] = 1;
            A_prt_in[L_new_vehicles + r][r + v * L_priority_tasks] = 1;

            ++r_count;
        }
    }

    std::vector<int> b_prt_in(L_new_vehicles + L_priority_tasks, 1);

    std::vector<int> A_prt_eq(L_new_vehicles * L_priority_tasks, 1);
    unsigned __int64 b_prt_eq(std::min({ L_new_vehicles,L_priority_tasks }));

    unsigned __int64 num_var(L_priority_tasks * L_new_vehicles);

    IloEnv env_PRT;

    try {

        IloModel model_PRT(env_PRT);
        IloNumVarArray var_PRT(env_PRT);
        IloRangeArray con_PRT(env_PRT);
        populatebyrowLP(model_PRT, var_PRT, con_PRT, A_prt_in, A_prt_eq, b_prt_in, b_prt_eq, num_var, t_cost);

        IloCplex cplex(model_PRT);
        cplex.solve();

        IloNumArray X_reb(env_PRT);
        cplex.getValues(X_reb, var_PRT);

        for (unsigned __int64 i{ 0 }; i < e_prt.size(); ++i)
        {
            if (X_reb[i] == 1)
                priority_ass.push_back({ e_prt[i][1], e_prt[i][2] }); // Here I save IDs not indeces

            int flag_e_prt{ 0 };
            for (unsigned __int64 j{ 0 }; j < e1.size(); ++j)
            {
                if ((e_prt[i][1] == e1[j][1]) && (e_prt[i][2] == e1[j][3]))
                    flag_e_prt = 1;
            }

            if (flag_e_prt)
            {
                e1.push_back({ e_prt[i][0],e_prt[i][1],-1,e_prt[i][2],-1,-1,e_prt[i][3],e_prt[i][2],-1,-1 });
                flag_e_prt = 0;
            }
        }
    }

    // add to opt_ass the assignments after rebalancing

    catch (IloException e_PRT)
    {
        std::cerr << "Concert technology caught: " << e_PRT << std::endl;
    }

    catch (...)
    {
        std::cerr << "Unknown exception caught." << std::endl;
    }

    env_PRT.end();

}

void Opt_ass::Greedy_Assignment(std::vector<std::vector<double>>& e3, std::vector<std::vector<double>>& e2, std::vector<std::vector<double>>& e1, std::vector<Request>& R) // Here I should receive e3, e2, e1 for the greedy assignment. Are the only information needed
{

    std::vector<double> R_OK;
    std::vector<double> V_OK;

    unsigned __int64 a{ e3.size() };
    unsigned __int64 b{ e2.size() };
    unsigned __int64 c{ e1.size() };

    Init_guess.Resize_Rows(a + b + c);

    unsigned __int64 i_g{ b + c };

    if (!e3.empty()) // I could use the !T3.empty()
    {
        std::sort(e3.begin(), e3.end(), FirstColumnOnlyCmp());
        std::copy(e3.begin(), e3.end(), std::back_inserter(e3_save));

        while (!e3.empty())
        {
            // I can avoid these initializations
            double vehicle{ e3.back()[1] };
            double task1{ e3.back()[3] };
            double task2{ e3.back()[4] };
            double task3{ e3.back()[5] };

            bool isPresent_vehicle = std::find(V_OK.begin(), V_OK.end(), vehicle) != V_OK.end();
            bool isPresent_task1 = std::find(R_OK.begin(), R_OK.end(), task1) != R_OK.end();
            bool isPresent_task2 = std::find(R_OK.begin(), R_OK.end(), task2) != R_OK.end();
            bool isPresent_task3 = std::find(R_OK.begin(), R_OK.end(), task3) != R_OK.end();

            int flag{ 0 };

            if ((!isPresent_vehicle) && (!isPresent_task1) && (!isPresent_task2) && (!isPresent_task3))
            {
                R_OK.push_back(task1);
                R_OK.push_back(task2);
                R_OK.push_back(task3);

                V_OK.push_back(vehicle);

                flag = 1;
            }

            if (flag == 1)
            {
                Init_guess.Body[i_g] = 1; // Here I need to use an array or a vector of fixed length
                ++i_g;
            }
            else
            {
                Init_guess.Body[i_g] = 0;
                ++i_g;
            }

            e3.pop_back();

        }
    }

    i_g = c;

    if (!e2.empty()) // I could use the !T3.empty()
    {
        std::sort(e2.begin(), e2.end(), FirstColumnOnlyCmp());
        std::copy(e2.begin(), e2.end(), std::back_inserter(e2_save));

        while (!e2.empty())
        {
            // I can avoid these initializations
            double vehicle{ e2.back()[1] };
            double task1{ e2.back()[3] };
            double task2{ e2.back()[4] };

            bool isPresent_vehicle = std::find(V_OK.begin(), V_OK.end(), vehicle) != V_OK.end();
            bool isPresent_task1 = std::find(R_OK.begin(), R_OK.end(), task1) != R_OK.end();
            bool isPresent_task2 = std::find(R_OK.begin(), R_OK.end(), task2) != R_OK.end();

            int flag{ 0 };

            if ((!isPresent_vehicle) && (!isPresent_task1) && (!isPresent_task2))
            {
                R_OK.push_back(task1);
                R_OK.push_back(task2);

                V_OK.push_back(vehicle);

                flag = 1;
            }

            if (flag == 1)
            {
                Init_guess.Body[i_g] = 1; // Here I need to use an array or a vector of fixed length
                ++i_g;
            }
            else
            {
                Init_guess.Body[i_g] = 0;
                ++i_g;
            }

            e2.pop_back();

        }
    }

    i_g = 0;

    if (!e1.empty()) // I could use the !T3.empty()
    {
        std::sort(e1.begin(), e1.end(), FirstColumnOnlyCmp());
        std::copy(e1.begin(), e1.end(), std::back_inserter(e1_save));

        while (!e1.empty())
        {
            // I can avoid these initializations
            double vehicle{ e1.back()[1] };
            double task1{ e1.back()[3] };

            bool isPresent_vehicle = std::find(V_OK.begin(), V_OK.end(), vehicle) != V_OK.end();
            bool isPresent_task1 = std::find(R_OK.begin(), R_OK.end(), task1) != R_OK.end();

            int flag{ 0 };

            if ((!isPresent_vehicle) && (!isPresent_task1))
            {
                R_OK.push_back(task1);

                V_OK.push_back(vehicle);

                flag = 1;
            }

            if (flag == 1)
            {
                Init_guess.Body[i_g] = 1; // Here I need to use an array or a vector of fixed length
                ++i_g;
            }
            else
            {
                Init_guess.Body[i_g] = 0;
                ++i_g;
            }

            e1.pop_back();

        }
    }

    int q{ 0 };
    X_k_init.Resize_Rows(N); // Need to initialize it

    for (int w{ 0 }; w < N; ++w)
    {
        bool isPresent_task = std::find(R_OK.begin(), R_OK.end(), R[w].ID) != R_OK.end();
        if (!isPresent_task)
            X_k_init.Body[q] = 1; // Maybe I need to inizialize it to zero. (Add function in the class to initialize)
        else
            X_k_init.Body[q] = 0;
        ++q;
    }

    Init_guess.Body.insert(Init_guess.Body.end(), X_k_init.Body.begin(), X_k_init.Body.end());

}

void Opt_ass::MILP_LP_Assignment(std::vector<std::vector<int>>& RTV_Adj, int tot, std::vector<Request>& R, std::vector<Vehicle>& V, int size1_exist, std::vector<std::vector<double>>& size1_assigned)
{
    std::vector<std::vector<double>> e;
    std::vector<double> c_i_j; //vector of the costs

    if (!e1_save.empty())
    {
        std::copy(e1_save.begin(), e1_save.end(), std::back_inserter(e));
        for (unsigned __int64 i{ 0 }; i < e1_save.size(); ++i)
            c_i_j.push_back(e1_save[i][0]);
    }

    if (!e2_save.empty())
    {
        e.insert(e.end(), e2_save.begin(), e2_save.end());
        for (unsigned __int64 i{ 0 }; i < e2_save.size(); ++i)
            c_i_j.push_back(e2_save[i][0]);
    }

    if (!e3_save.empty())
    {
        e.insert(e.end(), e3_save.begin(), e3_save.end());
        for (unsigned __int64 i{ 0 }; i < e3_save.size(); ++i)
            c_i_j.push_back(e3_save[i][0]);
    }

    unsigned __int64 e_r{ e.size() }; // number of rows of 'e'

    // I_V matrix
    //std::vector<std::vector<int>> I_V(num_v, std::vector<int>(e_r));
    Resizable_Vector <int> I_V;
    I_V.Resize_Body(num_v, e_r);
    for (int v{ 0 }; v < num_v; ++v)
    {
        for (unsigned __int64 z{ 0 }; z < e_r; ++z)
        {
            if (v == e[z][1])
                I_V.Body[v][z] = 1;
            else
                I_V.Body[v][z] = 0;
        }
    }

    // I_T matrix
    //std::vector<std::vector<int>> I_T(tot, std::vector<int>(e_r));
    Resizable_Vector <int> I_T;
    I_T.Resize_Body(tot, e_r);
    for (int t{ 0 }; t < tot; ++t)
    {
        for (unsigned __int64 z{ 0 }; z < e_r; ++z)
        {
            if (t == e[z][2])
                I_T.Body[t][z] = 1;
            else
                I_T.Body[t][z] = 0;
        }
    }

    // I_R matrix
    //std::vector<std::vector<int>> I_R(N, std::vector<int>(tot));
    Resizable_Vector <int> I_R;
    I_R.Resize_Body(N, tot);
    std::vector<int> temp_row;

    for (int i{ 0 }; i < N; ++i)
    {
        //std::copy(RTV_Adj[i].begin() + (N + num_v), RTV_Adj[i].end(), std::back_inserter(temp_row));
        //std::copy(RTV_Adj[i].begin() + (N+num_v), RTV_Adj[i].end(), std::back_inserter(I_R[i]));
        I_R.Body[i].assign(RTV_Adj[i].begin() + N + num_v, RTV_Adj[i].end());
        for (unsigned __int64 k{ I_R.Body[i].size() }; k < tot; ++k)
            I_R.Body[i].push_back(0);
        //I_R.push_back(temp_row);
        //temp_row.clear();
    }

    // BUILD THE MATRICES OF THE CONSTRAINTS FOR THE ILP

    // Inequality constraint matrices
    //std::vector<std::vector<int>> A_const1(num_v);
    //std::vector<int> b_const1(num_v, 1);
    //std::vector<int> zeros_A_const(N, 0);

    Resizable_Vector <int> A_const1;
    A_const1.Resize_Rows(num_v);

    Resizable_Array <int> b_const1;
    b_const1.Initialize(num_v, 1); // I need to initialize it to 1

    Resizable_Vector <int> zeros_A_const;
    zeros_A_const.Resize_Body(num_v, N);

    for (int i{ 0 }; i < num_v; ++i)
    {
        // Maybe it would be better to first copy I_V into A and then expand A,
        // so that to not loose information on I_V
        std::copy(I_V.Body[i].begin(), I_V.Body[i].end(), std::back_inserter(A_const1.Body[i]));
        A_const1.Body[i].insert(A_const1.Body[i].end(), zeros_A_const.Body[i].begin(), zeros_A_const.Body[i].end());
    }

    // Here I need to add the part about vector1, vector2 and vector3

    // Equality constraint matrices
    //std::vector<std::vector<int>> A_const2(N);
    //std::vector<int> b_const2(N, 1);

    Resizable_Vector <int> A_const2;
    A_const2.Resize_Rows(N);

    Resizable_Array <int> b_const2;
    b_const2.Initialize(N, 1); // Need to initialize to 1

    // Product between matrices I_R and I_T
    std::vector<std::vector<int>> IR_IT_Product; // was __int64
    IR_IT_Product = MatrixProduct(I_R.Body, I_T.Body, I_R.Body.size(), I_T.Body[0].size(), I_R.Body[0].size());

    for (int i{ 0 }; i < N; ++i)
    {
        std::copy(IR_IT_Product[i].begin(), IR_IT_Product[i].end(), std::back_inserter(A_const2.Body[i]));
        A_const2.Body[i].insert(A_const2.Body[i].end(), zeros_A_const.Body[0].begin(), zeros_A_const.Body[0].end());
        A_const2.Body[i][e_r + i] = 1; // To add the identity matrix information
    }
    
    // I SHOULD READ FROM A TEXT OR SOMETHING THE INFO RELATED TO NEW TASKS, PRIORITIES ECC
    if (size1_exist == 1) // should make this a function
    {

        unsigned __int64 s{ 0 };
        unsigned __int64 rows_size1{ size1_assigned.size() }; // Shold be passed as argument from Edges.size1_assignment
        
        std::vector<std::vector<int>> vector1(rows_size1); // I know one size of vector1. // Maybe this does not work
        std::vector<std::vector<int>> vector2;

        std::vector<std::vector<double>> ee;
        for (unsigned __int64 i{ 0 }; i < e.size(); ++i)
            ee.push_back(e[i]);

        for (unsigned __int64 i{ 0 }; i < e_r; ++i)
            ee[i].push_back(0);
         
        //double vehicle_s1e{};
        //double request_s1e{};

        for (unsigned __int64 j{ 0 }; j < rows_size1; ++j)
        {

            for (unsigned __int64 i{ 0 }; i < e_r; ++i)
            {

                if ((e[i][1] == size1_assigned[j][0]) && ((e[i][3] == size1_assigned[j][1]) || (e[i][4] == size1_assigned[j][1]) || (e[i][5] == size1_assigned[j][1])))
                    vector1[j].push_back(1); // need to decide how to manage the use of this vector. This vector should go
                                             // entirely in a constraint so has to be fixed size
                else
                    vector1[j].push_back(0);

                // I should later fin a way to avoid to mark the vectors with 1 and to the search
                // in a more elegant way
                if ((((ee[i][1] == size1_assigned[j][0]) && ((ee[i][3] != size1_assigned[j][1]) && (ee[i][4] != size1_assigned[j][1]) && (ee[i][5] != size1_assigned[j][1])))
                    || (((ee[i][3] == size1_assigned[j][1]) || (ee[i][4] == size1_assigned[j][1]) || (e[i][5] == size1_assigned[j][1])) && (ee[i][1] != size1_assigned[j][0]))) && (ee[i][10] == 0))
                {
                    std::vector<int> zeros(e_r, 0);
                    vector2.push_back(zeros);
                    vector2[s][i] = 1;
                    ee[i][10] = 1;
                    ++s;
                }
            }
        }

        unsigned __int64 r_vector1{ rows_size1 };
        unsigned __int64 r_vector2{ s };

        // Here I attach vector1 and vector2 to A_const2 and modify also b_const2

        std::vector<int> temp_vect;
        Resizable_Array <int> zeros_N;
        zeros_N.Initialize(N, 0);

        for (unsigned __int64 i{ 0 }; i < r_vector1; ++i)
        {
            std::copy(vector1[i].begin(), vector1[i].end(), std::back_inserter(temp_vect));
            temp_vect.insert(temp_vect.end(), zeros_N.Body.begin(), zeros_N.Body.end());
            A_const2.Body.push_back(temp_vect);
            temp_vect.clear();

            b_const2.Body.push_back(1);
        }

        for (unsigned __int64 i{ 0 }; i < r_vector2; ++i)
        {
            std::copy(vector2[i].begin(), vector2[i].end(), std::back_inserter(temp_vect));
            temp_vect.insert(temp_vect.end(), zeros_N.Body.begin(), zeros_N.Body.end());
            A_const2.Body.push_back(temp_vect);
            temp_vect.clear();

            b_const2.Body.push_back(0);
        }
    }

    if (!priority_ass.empty()) // I should put it in a function
    {
        unsigned __int64 rows_priority_ass{ priority_ass.size() };
        std::vector<std::vector<int>> vector3(rows_priority_ass);

        for (unsigned __int64 j{ 0 }; j < rows_priority_ass; ++j)
        {
            for (unsigned __int64 i{ 0 }; i < e_r; ++i)
            {
                if ((e[i][3] == priority_ass[j][1]) || (e[i][4] == priority_ass[j][1]) || (e[i][5] == priority_ass[j][1]))
                    vector3[j].push_back(1);
                else
                    vector3[j].push_back(0);
            }
        }
        unsigned __int64 r_vector3{ rows_priority_ass };

        // Here I add vector3 to the constraints

        std::vector<int> temp_vect;
        Resizable_Array <int> zeros_N;
        zeros_N.Initialize(N, 0);

        for (unsigned __int64 i{ 0 }; i < r_vector3; ++i)
        {
            std::copy(vector3[i].begin(), vector3[i].end(), std::back_inserter(temp_vect));
            temp_vect.insert(temp_vect.end(), zeros_N.Body.begin(), zeros_N.Body.end());
            A_const2.Body.push_back(temp_vect);
            temp_vect.clear();

            b_const2.Body.push_back(1);
        }
    }

    // Cost vector
    double c_k0{ 1e2 };
    //std::vector<double> fixed_cost(N, c_k0);
    Resizable_Array <double> fixed_cost;
    fixed_cost.Initialize(N, c_k0); // Need to initialize it to c_k0;
    c_i_j.insert(c_i_j.end(), fixed_cost.Body.begin(), fixed_cost.Body.end());

    // SOLUTION OF THE MILP

    IloEnv env;

    //std::vector<Assignment> opt_ass; // Vector of objects that contains the assignment

    try {

        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);
        populatebyrowMILP(model, var, con, A_const1.Body, A_const2.Body, b_const1.Body, b_const2.Body, e_r, c_i_j, Init_guess.Body); // to build the optimization problem

        IloCplex cplex(model);
        cplex.solve();

        // Print status of the solution
        //env.out() << "Solution status = " << cplex.getStatus() << std::endl;
        //env.out() << "Solution value = " << cplex.getObjValue() << std::endl;

        IloNumArray X(env);
        cplex.getValues(X, var);
        /*
        env.out() << "X = " << X << std::endl;
        */

        // Print optimal assignment (I should print it on a text file with a function and not on command line)
        // The info will be contained in the assignment objects

        std::cout << "--->Optimal assignment is:\n";
        std::cout << "Cost" << '\t' << "Vehicle" << '\t' << "Trip ID" << '\t' << "Task1" << '\t'
            << "Task2" << '\t' << "Task3" << '\t' << "Pass" << "DO n1" << '\t' << "DO n2" << '\t'
            << "DO n3" << '\n';
        
        std::vector<double> task_PU;
        std::vector<double> task_DO;

        
        for (unsigned __int64 i{ 0 }; i < e_r; ++i)
        {
            if (X[i] == 1)
            {
                
                for (unsigned __int64 k{ 0 }; k < e[i].size(); ++k)
                    std::cout << e[i][k] << '\t';
                std::cout << '\n';
                
            }
        }
        

        cplex.exportModel("ipex.lp");

        for (unsigned __int64 i{ 0 }; i < e_r; ++i)
        {
            if (X[i] == 1)
            {

                //opt_ass.push_back(Assignment(e[i][0], T[e[i][2]].rPU, T[e[i][2]].rDO, e[i][1], e[i][6], R, V));

                // populate assignments
                task_PU = { e[i][3], e[i][4], e[i][5] };
                task_DO = { e[i][7], e[i][8], e[i][9] };
                assignments.push_back(Opt_ass::assignment(e[i][0], task_PU, task_DO, e[i][1], e[i][6], R, V));

                V_ass.push_back(e[i][1]);
                R_ass.push_back(e[i][3]);

                if (e[i][4] != -1)
                    R_ass.push_back(e[i][4]);
                if (e[i][5] != -1)
                    R_ass.push_back(e[i][5]);
            }
        }

        //std::cout << "number of assignmnets: " << assignments.size() << '\n';

        // Solution of the rebalancing stage
        LP_Rebalancing(R, V);

        // Print on text file info about assignments of size 1
        print_size_one(R, V);

    }

    catch (IloException exc)
    {
        std::cerr << "Concert exception caught: " << exc << std::endl;
    }

    catch (...)
    {
        std::cerr << "Unknown exception caught." << std::endl;
    }

    //env.out() << "Press return to exit..." << std::endl;
    //std::getchar();
    env.end();
}

void Opt_ass::LP_Rebalancing(std::vector<Request>& R, std::vector<Vehicle>& V)
{

    unsigned __int64 L_R_OK{ R_ass.size() };
    unsigned __int64 L_V_OK{ V_ass.size() };

    if ((L_R_OK < N) && (L_V_OK < num_v))
    {

        //std::vector<double> R_KO;
        //std::vector<double> V_idle;

        std::vector<double> R_indeces;
        std::vector<double> V_indeces;

        for (unsigned __int64 i{ 0 }; i < N; ++i)
        {
            bool isPresent_task = std::find(R_ass.begin(), R_ass.end(), R[i].ID) != R_ass.end();
            if (!isPresent_task)
            {
                //R_KO.push_back(R[i].ID);
                R_indeces.push_back(i);
            }

            //R_indeces.push_back(R[i].ID);
        }
        for (unsigned __int64 i{ 0 }; i < num_v; ++i)
        {
            bool isPresent_vehicle = std::find(V_ass.begin(), V_ass.end(), V[i].ID) != V_ass.end();
            if (!isPresent_vehicle)
            {
                //V_idle.push_back(V[i].ID);
                V_indeces.push_back(i);
            }

            //V_indeces.push_back(V[i].ID);
        }

        unsigned __int64 L_V_indeces{ V_indeces.size() };
        unsigned __int64 L_R_indeces{ R_indeces.size() };

        int v_indice{};
        int r_indice{};

        std::array<double, 2> vehicle_pose;
        std::array<double, 2> request_orgn;
        std::array<double, 2> request_dest;

        std::vector<double> t_cost;

        std::vector<std::vector<double>> e_reb;

        std::vector<std::vector<int>> A_reb_in(L_V_indeces + L_R_indeces, std::vector<int>(L_R_indeces * L_V_indeces));
        //std::vector<std::vector<int>> A_reb_2(L_R_indeces, std::vector<int>(L_R_indeces * L_V_indeces));
        //std::vector<int> zeros_A_reb(L_R_indeces, 0);

        int r_count{ 0 };

        for (unsigned __int64 v{ 0 }; v < L_V_indeces; ++v)
        {
            for (unsigned __int64 r{ 0 }; r < L_R_indeces; ++r)
            {
                v_indice = static_cast<int>(V_indeces[v]);
                r_indice = static_cast<int>(R_indeces[r]);

                vehicle_pose = { V[v_indice].x, V[v_indice].y };
                request_orgn = { R[r_indice].xo, R[r_indice].yo };
                request_dest = { R[r_indice].xd, R[r_indice].yd };

                t_cost.push_back(tt(vehicle_pose, request_orgn) + tt(request_orgn, request_dest));

                e_reb.push_back({ t_cost[r_count], V[v_indice].ID, R[r_indice].ID, R[r_indice].pass });

                // Update matric for inequality constraint
                A_reb_in[v][r + v * L_R_indeces] = 1;
                A_reb_in[L_V_indeces + r][r + v * L_R_indeces] = 1;

                ++r_count;
            }
        }

        std::vector<int> b_reb_in(L_V_indeces + L_R_indeces, 1);

        std::vector<int> A_reb_eq(L_R_indeces * L_V_indeces, 1);
        unsigned __int64 b_reb_eq(std::min({ L_R_indeces, L_V_indeces }));

        //std::cout << "L_R_indeces is: " << L_R_indeces << '\n';
        //std::cout << "L_V_indeces is: " << L_V_indeces << '\n';

        unsigned __int64 num_var{ L_R_indeces * L_V_indeces };

        // HERE RUN THE CPLEX FOR REBALANCING

        IloEnv env_LP;

        try {

            IloModel model_LP(env_LP);
            IloNumVarArray var_LP(env_LP);
            IloRangeArray con_LP(env_LP);
            populatebyrowLP(model_LP, var_LP, con_LP, A_reb_in, A_reb_eq, b_reb_in, b_reb_eq, num_var, t_cost);

            IloCplex cplex(model_LP);
            cplex.solve();

            IloNumArray X_reb(env_LP);
            cplex.getValues(X_reb, var_LP);

            //env_LP.out() << "X_reb = " << X_reb << std::endl;

            
            // Print Rebalancing results
            std::cout << "--->Optimal rebalancing assignment is:\n";
            std::cout << "Cost" << '\t' << "Vehicle" << '\t' << "Task" << '\t' << "Pass" << '\n';
            //std::vector<double> only_task;
            

            std::vector<double> task_PU;
            std::vector<double> task_DO;

            for (unsigned __int64 i{ 0 }; i < e_reb.size(); ++i)
            {
                if (X_reb[i] == 1)
                {
                    //only_task.push_back(e_reb[i][3]);
                    //opt_ass.push_back(Assignment(e_reb[i][0], only_task, only_task, e_reb[i][1], e_reb[i][6], R, V));
                    //only_task.clear();

                    task_PU = { e_reb[i][2], -1, -1 };
                    task_DO = { e_reb[i][2], -1, -1 };
                    assignments.push_back(Opt_ass::assignment(e_reb[i][0], task_PU, task_DO, e_reb[i][1], e_reb[i][3], R, V));
                    
                    // print of optimal rebalancing assignment
                    for (unsigned __int64 k{ 0 }; k < e_reb[i].size(); ++k)
                        std::cout << e_reb[i][k] << '\t';
                    std::cout << '\n';
                    
                }
            }
        }

        // add to opt_ass the assignments after rebalancing

        catch (IloException e_LP)
        {
            std::cerr << "Concert technology caught: " << e_LP << std::endl;
        }

        catch (...)
        {
            std::cerr << "Unknown exception caught." << std::endl;
        }

        env_LP.end();

    } // end of Rebalancing
}

// This function computes the Matrix Product
std::vector<std::vector<int>> Opt_ass::MatrixProduct(std::vector<std::vector<int>>& Matrix_1, std::vector<std::vector<int>>& Matrix_2, unsigned __int64 M1_rows, unsigned __int64 M2_cols, unsigned __int64 M1_cols)
{
    int scal_prod;
    std::vector<std::vector<int>> Product(M1_rows, std::vector<int>(M2_cols));

    for (unsigned __int64 i{ 0 }; i < M1_rows; ++i)
    {
        for (unsigned __int64 j{ 0 }; j < M2_cols; ++j)
        {
            scal_prod = 0;

            // Scalar product between the row and column vectors
            for (unsigned __int64 k{ 0 }; k < M1_cols; ++k)
                scal_prod += (Matrix_1[i][k] * Matrix_2[k][j]);

            Product[i][j] = scal_prod;
        }
    }

    return Product;
}

// This function write info about assignments of size one couplings and the info about the Requests and Vehicles involved
void Opt_ass::print_size_one(std::vector<Request>& R, std::vector<Vehicle>& V)
{
    
    std::ofstream File_1; // to write info about the size 1 assignments
    std::ofstream File_R; // to add the info about the tasks
    std::ofstream File_V; // to add the info about the vehicles

    File_1.open("C:\\Users\\Matteo\\Desktop\\C++\\Optimal_task_assignment_Classes\\Assignments_Size_One.txt");
    if (!File_1)
    {
        std::cerr << "Unable to open file Assignment_Size_One.txt for writing";
        exit(1); // call system to stop
    }

    File_R.open("C:\\Users\\Matteo\\Desktop\\C++\\Optimal_task_assignment_Classes\\Requests_Data.txt");
    if (!File_R)
    {
        std::cerr << "Unable to open file Requests_Data2.txt for writing";
        exit(1); // call system to stop
    }

    File_V.open("C:\\Users\\Matteo\\Desktop\\C++\\Optimal_task_assignment_Classes\\Vehicles_Data.txt");
    if (!File_V)
    {
        std::cerr << "Unable to open file Vehicles_Data2.txt for writing";
        exit(1); // call system to stop
    }

    write_title_Assignments_Size_One(File_1);

    write_title_R(File_R);

    write_title_V(File_V);

    // and I have also to write on the task and vehicles files the tasks and vehicles infomrations, because they will enter again the problem
    // for possible new reassignments to trips of size 2
    for (unsigned __int64 i{ 0 }; i < assignments.size(); ++i)
    {
        if (assignments[i].dim == 1)
        {
            File_1 << assignments[i].cost << '\t' << assignments[i].vehicle << '\t' << assignments[i].tasks[0] << '\t' << assignments[i].pass << '\n';
            R[assignments[i].R_i[0]].print_info_R(File_R);
            V[assignments[i].V_i].print_info_V(File_V);
        }
    }

    File_1.close();
    File_R.close();
    File_V.close();
    
}

bool Opt_ass::check_priority(std::vector<Request>& R, std::vector<Vehicle>& V)
{
    for (unsigned __int64 r{ 0 }; r < R.size(); ++r)
    {
        if (R[r].priority > 0)
            priority_tasks.push_back(r);
    }

    for (unsigned __int64 v{ 0 }; v < V.size(); ++v)
    {
        if (V[v].status == 0)
            new_vehicles.push_back(v);
    }

    return (!priority_tasks.empty() && !new_vehicles.empty());
}

// Constructor for Class 'assignment'
Opt_ass::assignment::assignment(double cost_time, std::vector<double>& task_set_PU, std::vector<double>& task_set_DO, double veh,
    double passengers, std::vector<Request>& R, std::vector<Vehicle>& V)
{

    std::copy(task_set_PU.begin(), task_set_PU.end(), std::back_inserter(tasks));
    std::copy(task_set_PU.begin(), task_set_PU.end(), std::back_inserter(pick_up));
    std::copy(task_set_DO.begin(), task_set_DO.end(), std::back_inserter(drop_off));
    vehicle = veh;
    dim = size_of_trip(task_set_PU); // should call the function to look for the -1
    pass = passengers;
    cost = cost_time;
    priority = 1;

    for (unsigned __int64 v{ 0 }; v < V.size(); ++v)
    {
        if (V[v].ID == vehicle)
        {
            V_i = v;
            V[v].status = 1;
        }
    }

    for (unsigned __int64 r{ 0 }; r < R.size(); ++r)
    {
        if ((R[r].ID == tasks[0]) || (R[r].ID == tasks[1]) || (R[r].ID == tasks[2]))
        {
            R_i.push_back(r);
            R[r].status = 1;
        }
    }

}

// Function used by the constructor to tell the size of the assignment
int Opt_ass::assignment::size_of_trip(std::vector<double>& task_set)
{

    int i{ 0 };
    while (task_set[i] != -1)
        ++i;

    return i;

}

#endif OPT_ASS_H