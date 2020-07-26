#ifndef EDGES_H
#define EDGES_H

#include <iostream>
#include <array>
#include <vector>
#include <random>
#include <string>
#include <sstream>
#include <fstream>
#include "Data_structures.h"
#include "V_Class.h"
#include "R_Class.h"
#include "Resizable_Array.h"
#include "Resizable_Vector.h"
#include "Distance_function.h"

extern unsigned __int64 N;
extern unsigned __int64 num_v;

class Edges
{
	public:
        
        // Adjacency matrix of RTV-graph
        Resizable_Vector <int> RTV_Adj;
        __int64 tot = 0; // tot is size(RTV_Adj - (N+num_v) so probably I do not need it outside

        // Edges of sizes: 1, 2, 3
        std::vector<std::vector<double>> e3;
        std::vector<std::vector<double>> e2;
        std::vector<std::vector<double>> e1;

        int size1_exist{ 0 };
        std::vector<std::vector<double>> size1_assigned;

        //void RTV_Compute(std::vector<std::vector<int>> Adj, std::vector<std::vector<int>>& Real_Adj, std::vector<std::vector<int>>& Kind_of_link, std::vector<std::vector<int>>& Kind_of_link2, std::vector<std::vector<int>>& priority_mat, std::vector<Request>& R, std::vector<Vehicle>& V);
        void RTV_Compute(Couplings& coupling, std::vector<Request>& R, std::vector<Vehicle>& V);

    private:
        int get_assignments_size_one(std::vector<std::vector<double>>& e1);

    private:

        // Function travel2() for feasibility computation of size 2 trips
        std::tuple<int, double, double, double, double> travel2(Vehicle V, Request R1, Request R2, int KoL);

        // Function travel2() for feasibility computation of size 3 trips
        std::tuple<int, double, std::vector<double>, double> travel3(std::array<int, 3>& task_id, Vehicle V, std::array<int, 3>& pu_order, std::array<int, 3>& do_order, int KoL, std::vector<Request>& R);

};

//void Edges::RTV_Compute(std::vector<std::vector<int>> Adj, std::vector<std::vector<int>>& Real_Adj, std::vector<std::vector<int>>& Kind_of_link, std::vector<std::vector<int>>& Kind_of_link2, std::vector<std::vector<int>>& priority_mat, std::vector<Request>& R, std::vector<Vehicle>& V)
void Edges::RTV_Compute(Couplings& coupling, std::vector<Request>& R, std::vector<Vehicle>& V)
{

	// I can then divide the RTV_computation into private functions like: size1_trips, size2_trips, size3_trips

    int k{ 0 };
    int kk{ 0 };
    int kkk{ 0 };

    __int64 out = 0;

    // vectors that contains all the trips of the different sizes
    std::vector<int> T1_index;
    std::vector<int> T2_index;
    std::vector<int> T3_index;

    // matrices that contains the vehicles by row and the assigned trips by column
    //std::vector<std::vector<int>> T1_idx(num_v, std::vector<int>());
    //std::vector<std::vector<int>> T2_idx(num_v, std::vector<int>());
    //std::vector<std::vector<int>> T3_idx(num_v, std::vector<int>());

    Resizable_Vector <int> T1_idx;
    T1_idx.Resize_Rows(num_v);

    Resizable_Vector <int> T2_idx;
    T2_idx.Resize_Rows(num_v);

    Resizable_Vector <int> T3_idx;
    T3_idx.Resize_Rows(num_v);

    // definition of vectors of data for the RTV graph

    // First initialization of the RTV_Adj matrix and the other matrices
    //std::vector<std::vector<int>> RTV_Adj(N + num_v, std::vector<int>(N + num_v));
    //std::vector<std::vector<double>> cost_edge(num_v, std::vector<double>());
    //std::vector<std::vector<double>> c_t_edge(num_v, std::vector<double>());
    
    RTV_Adj.Resize_Body(N + num_v, N + num_v);

    Resizable_Vector <double> cost_edge;
    cost_edge.Resize_Rows(num_v);

    Resizable_Vector <double> c_t_edge;
    c_t_edge.Resize_Rows(num_v);

    // vector of the trips
    std::vector<Trip> T;
    std::vector<int> idx; // need to verify if idx at the next cycle is overwritten or copied by element
                          // In the last case I'd need to clear its content at every different vehicle

    int n_1 = 7; // it was 7
    for (__int64 v{ 0 }; v < num_v; ++v) // I SHOULD INTEGRATE THIS PART INSIDE THE PREVIOUS (for v...) CYCLE
    {
        // [ADD TRIPS OF SIZE 1]
        std::vector<int> idx_original;
        for (int i{ 0 }; i < N; ++i)
        {
            if (coupling.Adj.Body[N + v][i] != 0)
                idx_original.push_back(i);
        }

        // I can clear idx here or set it to zero elements or null elements
        if (idx_original.size() > n_1)
        {
            std::sample(idx_original.begin(), idx_original.end(), std::back_inserter(idx), n_1, std::mt19937{ std::random_device{}() });
            /*
            int prior_lev2{ 2 }; // should use an enumerator
            bool isPresent_priority = std::find(coupling.priority_mat.Body[N + v].begin(), coupling.priority_mat.Body[N + v].end(), prior_lev2) != coupling.priority_mat.Body[N + v].end();

            //std::cout << "Bool is: " << isPresent_priority << '\n';

            if (isPresent_priority) // if this is true, I want the index of that 2
            {
                std::vector<int>::iterator itr = std::find(coupling.priority_mat.Body[N + v].begin(), coupling.priority_mat.Body[N + v].end(), 2);
                int r_indice{ 0 };
                r_indice = std::distance(coupling.priority_mat.Body[N + v].begin(), itr);

                bool isPresent_r_indice = std::find(idx.begin(), idx.end(), r_indice) != idx.end();
                if (!isPresent_r_indice)
                    idx.push_back(r_indice);
            }
            */
        }
        else
        {
            if (idx_original.size() > 0)
                std::copy(idx_original.begin(), idx_original.end(), std::back_inserter(idx));
            else
                idx.clear();
        }

        for (int i{ 0 }; i < idx.size(); ++i)
        {
            int index{ idx[i] };
            std::array<double, 2> vehicle_pose = { V[v].x, V[v].y };
            std::array<double, 2> request_start = { R[index].xo, R[index].yo };
            std::array<double, 2> request_end = { R[index].xd, R[index].yd };

            double cost{ 0.0 };
            cost = tt(vehicle_pose, request_start) + tt(request_start, request_end);
            double c_t{ cost - R[index].arr_t };

            int found_T1{ 0 }; // maybe it costs a lot to initialize it every time

            if (!T.empty())
            {
                int t_index{ 0 };

                for (int t{ 0 }; t < T1_index.size(); ++t)
                {
                    t_index = T1_index[t];

                    std::vector<double> task = { R[index].ID }; //////////////////////////////////////////////////

                    if (task == T[t_index].rPU) // here I need to review the condition
                    {

                        if (RTV_Adj.Body[N + v].size() <= N + num_v + t_index)
                        {
                            // should integrate all this content in a function

                            for (unsigned __int64 y{ RTV_Adj.Body[N + v].size() }; y < N + num_v + t_index; ++y)
                                RTV_Adj.Body[N + v].push_back(0);

                            RTV_Adj.Body[N + v].push_back(1);

                            for (unsigned __int64 y{ cost_edge.Body[v].size() }; y < t_index; ++y)
                            {
                                cost_edge.Body[v].push_back(0);
                                c_t_edge.Body[v].push_back(0);
                            }
                            c_t_edge.Body[v].push_back(c_t);
                            cost_edge.Body[v].push_back(cost);
                        }
                        else
                        {
                            RTV_Adj.Body[N + v][N + num_v + t_index] = 1;
                            cost_edge.Body[v][t_index] = cost;
                            c_t_edge.Body[v][t_index] = c_t;
                        }
                        found_T1 = 1;
                    }
                }
            }

            if (found_T1 == 1) // I can avoid this if()
            {
                //++p; // maybe I don't need variable 'p';
                T1_idx.Body[v].push_back(index);
            }
            else
            {

                index = idx[i];
                T.push_back({ {R[index].ID}, {R[index].ID}, 0, 0, 1, 0, R[index].pass, static_cast<double>(index), 0 });

                T1_index.push_back(tot);
                T1_idx.Body[v].push_back(index); // should be okay cause then I go picking up the requests instead of trips of size1 trips (essentially they are the same thing

                ++k;
                ++tot;
                //++p;

                // This solution should push an additional trip to the whole set of trips
                for (unsigned __int64 y{ RTV_Adj.Body[index].size() }; y < N + num_v + tot - 1; ++y)
                    RTV_Adj.Body[index].push_back(0);

                RTV_Adj.Body[index].push_back(1);

                for (unsigned __int64 y{ RTV_Adj.Body[N + v].size() }; y < N + num_v + tot - 1; ++y)
                    RTV_Adj.Body[N + v].push_back(0);

                RTV_Adj.Body[N + v].push_back(1);

                for (unsigned __int64 y{ cost_edge.Body[v].size() }; y < tot - 1; ++y)
                {
                    cost_edge.Body[v].push_back(0);
                    c_t_edge.Body[v].push_back(0);
                }
                c_t_edge.Body[v].push_back(c_t);
                cost_edge.Body[v].push_back(cost);
            }
        }

        // [ADD TRIPS OF SIZE 2]

        for (int i{ 0 }; i < idx.size(); ++i)
        {
            for (int j{ 0 }; j < idx.size(); ++j)
            {
                int index1{ T1_idx.Body[v][i] };
                int index2{ T1_idx.Body[v][j] };

                double c_t_1{ 0.0 };
                double c_t_2{ 0.0 };

                if ((index1 != index2) && (coupling.Real_Adj.Body[index1][index2] == 1))
                {
                    // call of travel2 function

                    auto [value, cost, c_t_1, c_t_2, pass] = travel2(V[v], R[index1], R[index2], coupling.Kind_of_link.Body[index1][index2]);

                    if (value == 1)
                    {

                        int found_T2{ 0 };
                        int t_index{ 0 }; // declared here beacuse I need it on the 'if (found_T2 == 1)'

                        // search for existing trips of size 2
                        if (!T.empty()) // maybe better if (!T2_index.empy())
                        {

                            for (int t{ 0 }; t < T2_index.size(); ++t)
                            {
                                t_index = T2_index[t];

                                std::vector<double> tasks = { R[index1].ID, R[index2].ID };

                                if (tasks == T[t_index].rPU) // it should be done only if kk>0 // I should add also the condition about the dropoff order
                                {

                                    if (RTV_Adj.Body[N + v].size() <= N + num_v + t_index)
                                    {

                                        // should integrate all this content in a function

                                        for (unsigned __int64 y{ RTV_Adj.Body[N + v].size() }; y < N + num_v + t_index; ++y)
                                            RTV_Adj.Body[N + v].push_back(0);

                                        RTV_Adj.Body[N + v].push_back(1);

                                        for (unsigned __int64 y{ cost_edge.Body[v].size() }; y < t_index; ++y)
                                        {
                                            cost_edge.Body[v].push_back(0);
                                            c_t_edge.Body[v].push_back(0);
                                        }
                                        c_t_edge.Body[v].push_back(c_t_1 + c_t_2);
                                        cost_edge.Body[v].push_back(cost);
                                    }
                                    else
                                    {
                                        RTV_Adj.Body[N + v][N + num_v + t_index] = 1;
                                        cost_edge.Body[v][t_index] = cost;
                                        c_t_edge.Body[v][t_index] = c_t_1 + c_t_2;
                                    }
                                    found_T2 = 1;
                                    break;
                                }
                            }
                        }
                        if (found_T2 == 1) // I can avoid this if()
                        {
                            //++pp;
                            T2_idx.Body[v].push_back(t_index);
                        }
                        else
                        {
                            if (coupling.Kind_of_link.Body[index1][index2] == 1)
                                T.push_back({ {R[index1].ID, R[index2].ID}, {R[index1].ID, R[index2].ID}, coupling.Kind_of_link.Body[index1][index2], 0, 2, coupling.Kind_of_link2.Body[index1][index2], pass, static_cast<double>(index1), static_cast<double>(index2) });
                            else
                                T.push_back({ {R[index1].ID, R[index2].ID}, {R[index2].ID, R[index1].ID}, coupling.Kind_of_link.Body[index1][index2], 0, 2, coupling.Kind_of_link2.Body[index1][index2], pass, static_cast<double>(index1), static_cast<double>(index2) });


                            T2_index.push_back(tot);
                            T2_idx.Body[v].push_back(tot);

                            ++kk;
                            ++tot;
                            //++pp; // maybe I don't need that variable in the c++ code

                            // update RTV_Adj for index1 - Trip
                            for (unsigned __int64 y{ RTV_Adj.Body[index1].size() }; y < N + num_v + tot - 1; ++y) // I use unsigned because .size() returns an unsigned int
                                RTV_Adj.Body[index1].push_back(0);

                            RTV_Adj.Body[index1].push_back(1); // should change '0' and '1' with 'NO_LINK' and 'YES_LINK' enumerations

                            // update RTV_Adj for index2 - Trip
                            for (unsigned __int64 y{ RTV_Adj.Body[index2].size() }; y < N + num_v + tot - 1; ++y)
                                RTV_Adj.Body[index2].push_back(0);

                            RTV_Adj.Body[index2].push_back(1); // should change '0' and '1' with 'NO_LINK' and 'YES_LINK' enumerations

                            // update RTV_Adj for v - Trip
                            for (unsigned __int64 y{ RTV_Adj.Body[N + v].size() }; y < N + num_v + tot - 1; ++y)
                                RTV_Adj.Body[N + v].push_back(0);

                            RTV_Adj.Body[N + v].push_back(1);


                            for (unsigned __int64 y{ cost_edge.Body[v].size() }; y < tot - 1; ++y)
                            {
                                cost_edge.Body[v].push_back(0);
                                c_t_edge.Body[v].push_back(0);
                            }
                            cost_edge.Body[v].push_back(cost);
                            c_t_edge.Body[v].push_back(c_t_1 + c_t_2);
                        }
                    }
                }
            }
        }

        idx.clear();

        // [ADD TRIPS OF SIZE 3]

        // actually I could skip the next two lines so that to not use another vector but simply using T1_idx[v]...
        std::vector<int> T1_indeces;
        std::copy(T1_idx.Body[v].begin(), T1_idx.Body[v].end(), std::back_inserter(T1_indeces));

        std::vector<int> T2_indeces;
        int n_2 = 6; // it was 6

        if (T2_idx.Body[v].size() > n_2)
            std::sample(T2_idx.Body[v].begin(), T2_idx.Body[v].end(), std::back_inserter(T2_indeces), n_2, std::mt19937{ std::random_device{}() });
        else
            std::copy(T2_idx.Body[v].begin(), T2_idx.Body[v].end(), std::back_inserter(T2_indeces));

        if (!T2_indeces.empty()) // is T2_indeces is not empty
        {
            for (int i{ 0 }; i < T1_indeces.size(); ++i)
            {
                for (int j{ 0 }; j < T2_indeces.size(); ++j)
                {
                    int id_r3{ T1_indeces[i] };
                    int id_T2{ T2_indeces[j] };
                    int id_r1{ static_cast<int>(T[id_T2].index1) };
                    int id_r2{ static_cast<int>(T[id_T2].index2) };

                    if ((id_r1 != id_r2) && (id_r2 != id_r3) && (id_r1 != id_r3))
                    {
                        int flag1{ 1 };
                        int flag2{ 1 };

                        for (int y{ 0 }; y < T2_indeces.size(); ++y)
                        {
                            if ((N + num_v + T2_indeces[y] < RTV_Adj.Body[id_r1].size()) && (N + num_v + T2_indeces[y] < RTV_Adj.Body[id_r3].size()))
                            {
                                if ((RTV_Adj.Body[id_r1][N + num_v + T2_indeces[y]] == 1) && (RTV_Adj.Body[id_r3][N + num_v + T2_indeces[y]] == 1))
                                    ++flag1;
                            }
                            if ((N + num_v + T2_indeces[y] < RTV_Adj.Body[id_r2].size()) && (N + num_v + T2_indeces[y] < RTV_Adj.Body[id_r3].size()))
                            {
                                if ((RTV_Adj.Body[id_r2][N + num_v + T2_indeces[y]] == 1) && (RTV_Adj.Body[id_r3][N + num_v + T2_indeces[y]] == 1))
                                    ++flag2;
                            }
                        }


                        if ((flag1 > 1) && (flag2 > 1))
                        {

                            int pu_order1{ 0 };
                            int pu_order2{ 0 };
                            int pu_order3{ 0 };

                            if ((coupling.Real_Adj.Body[id_r1][id_r3] == 1) && (coupling.Real_Adj.Body[id_r2][id_r3] == 1))
                                pu_order1 = 1;
                            if ((coupling.Real_Adj.Body[id_r1][id_r3] == 1) && (coupling.Real_Adj.Body[id_r3][id_r2] == 1))
                                pu_order2 = 1;
                            if ((coupling.Real_Adj.Body[id_r3][id_r1] == 1) && (coupling.Real_Adj.Body[id_r3][id_r2] == 1))
                                pu_order3 = 1;

                            int do_order1{ 0 };
                            int do_order2{ 0 };
                            int do_order3{ 0 };

                            if (coupling.Kind_of_link.Body[id_r1][id_r2] == 1)
                            {
                                if (((coupling.Kind_of_link2.Body[id_r1][id_r3] != 2) || (coupling.Kind_of_link2.Body[id_r3][id_r1] != 1)) && ((coupling.Kind_of_link2.Body[id_r2][id_r3] != 2) || (coupling.Kind_of_link2.Body[id_r3][id_r2] != 1)))
                                    do_order1 = 1;
                                if (((coupling.Kind_of_link2.Body[id_r1][id_r3] != 2) || (coupling.Kind_of_link2.Body[id_r3][id_r1] != 1)) && ((coupling.Kind_of_link2.Body[id_r3][id_r2] != 2) || (coupling.Kind_of_link2.Body[id_r2][id_r3] != 1)))
                                    do_order2 = 1;
                                if (((coupling.Kind_of_link2.Body[id_r3][id_r1] != 2) || (coupling.Kind_of_link2.Body[id_r1][id_r3] != 1)) && ((coupling.Kind_of_link2.Body[id_r3][id_r2] != 2) || (coupling.Kind_of_link2.Body[id_r2][id_r3] != 1)))
                                    do_order3 = 1;
                            }
                            else if (coupling.Kind_of_link.Body[id_r1][id_r2] == 2)
                            {
                                if (((coupling.Kind_of_link2.Body[id_r2][id_r3] != 2) || (coupling.Kind_of_link2.Body[id_r3][id_r2] != 1)) && ((coupling.Kind_of_link2.Body[id_r1][id_r3] != 2) || (coupling.Kind_of_link2.Body[id_r3][id_r1] != 1)))
                                    do_order1 = 1;
                                if (((coupling.Kind_of_link2.Body[id_r2][id_r3] != 2) || (coupling.Kind_of_link2.Body[id_r3][id_r2] != 1)) && ((coupling.Kind_of_link2.Body[id_r3][id_r1] != 2) || (coupling.Kind_of_link2.Body[id_r1][id_r3] != 1)))
                                    do_order2 = 1;
                                if (((coupling.Kind_of_link2.Body[id_r3][id_r2] != 2) || (coupling.Kind_of_link2.Body[id_r2][id_r3] != 1)) && ((coupling.Kind_of_link2.Body[id_r3][id_r1] != 2) || (coupling.Kind_of_link2.Body[id_r1][id_r3] != 1)))
                                    do_order3 = 1;
                            }

                            std::array<int, 3> task_id = { id_r1, id_r2, id_r3 };
                            std::array<int, 3> pu_order = { pu_order1, pu_order2, pu_order3 };
                            std::array<int, 3> do_order = { do_order1, do_order2, do_order3 };
                            int KoL = coupling.Kind_of_link.Body[id_r1][id_r2];

                            std::array<Request, 3> R_candidate = { R[id_r1], R[id_r2], R[id_r3] };

                            auto [value, trip_cost, trip, pass] = travel3(task_id, V[v], pu_order, do_order, KoL, R); // R to pass all the requests. Maybe it is better to pass only few requests: I will check later

                            if (value == 1)
                            {
                                int found_T3{ 0 };
                                int t_index{ 0 };

                                if (!T3_index.empty())
                                {
                                    for (int x{ 0 }; x < T3_index.size(); ++x)
                                    {
                                        t_index = T3_index[x];
                                        std::vector<double> T_tasks; // vector to contain PU and DO order of the size 3 trip
                                        T_tasks.insert(T_tasks.end(), T[t_index].rPU.begin(), T[t_index].rPU.end());
                                        T_tasks.insert(T_tasks.end(), T[t_index].rDO.begin(), T[t_index].rDO.end());

                                        if (trip == T_tasks)
                                        {
                                            if (RTV_Adj.Body[N + v].size() <= N + num_v + t_index)
                                            {

                                                for (unsigned __int64 y{ RTV_Adj.Body[N + v].size() }; y < N + num_v + t_index; ++y)
                                                    RTV_Adj.Body[N + v].push_back(0);

                                                RTV_Adj.Body[N + v].push_back(1);

                                                for (unsigned __int64 y{ cost_edge.Body[v].size() }; y < t_index; ++y) // is it right???
                                                {
                                                    cost_edge.Body[v].push_back(0);
                                                    //c_t_edge[v].push_back(0);
                                                }

                                                cost_edge.Body[v].push_back(trip_cost);
                                                //c_t_edge[v].push_back(c_t_1 + c_t_2);
                                            }
                                            else
                                            {
                                                RTV_Adj.Body[N + v][N + num_v + t_index] = 1;
                                                cost_edge.Body[v][t_index] = trip_cost;
                                                //c_t_edge[v][t_index] = c_t_1 + c_t_2;
                                            }
                                            found_T3 = 1;
                                            break;
                                        }
                                    }
                                }
                                if (found_T3 == 1) // I can evoid this if()
                                {
                                    //++ppp;
                                    T3_idx.Body[v].push_back(t_index);
                                }
                                else
                                {
                                    // I'd like to change trip[.][.]... with something else
                                    T.push_back({ {trip[0], trip[1], trip[2]}, {trip[3], trip[4], trip[5]}, 0, 0, 3, 0, pass, 0, 0 });

                                    T3_index.push_back(tot);
                                    T3_idx.Body[v].push_back(tot);

                                    ++kkk;
                                    ++tot;
                                    //++ppp; // maybe I do not need that variable

                                    // am I here actually pointing to the correct requests??? id_r1 is the task ID or its position???
                                    for (unsigned __int64 y{ RTV_Adj.Body[id_r1].size() }; y < N + num_v + tot - 1; ++y)
                                        RTV_Adj.Body[id_r1].push_back(0);

                                    RTV_Adj.Body[id_r1].push_back(1);

                                    for (unsigned __int64 y{ RTV_Adj.Body[id_r2].size() }; y < N + num_v + tot - 1; ++y)
                                        RTV_Adj.Body[id_r2].push_back(0);

                                    RTV_Adj.Body[id_r2].push_back(1);

                                    for (unsigned __int64 y{ RTV_Adj.Body[id_r3].size() }; y < N + num_v + tot - 1; ++y)
                                        RTV_Adj.Body[id_r3].push_back(0);

                                    RTV_Adj.Body[id_r3].push_back(1);

                                    for (unsigned __int64 y{ cost_edge.Body[v].size() }; y < tot - 1; ++y)
                                        cost_edge.Body[v].push_back(0);

                                    cost_edge.Body[v].push_back(trip_cost);
                                }
                            }
                        }
                    }
                }
            }
        }
        //p = 0;
        //pp = 0;
        //ppp = 0;
    } // end of for v=1:num_v

    // Population of the edges (that will be the optimization variables)
    for (int t{ 0 }; t < tot; ++t)
    {
        for (int v{ 0 }; v < num_v; ++v)
        {
            if (N + num_v + t < RTV_Adj.Body[N + v].size())
            {
                if ((T[t].dim == 3) && (RTV_Adj.Body[N + v][N + num_v + t] == 1) && (kkk != 0))
                {
                    e3.push_back({ cost_edge.Body[v][t], V[v].ID, static_cast<double>(t), T[t].rPU[0], T[t].rPU[1], T[t].rPU[2], T[t].pass, T[t].rDO[0], T[t].rDO[1], T[t].rDO[2] });
                }
                if ((T[t].dim == 2) && (RTV_Adj.Body[N + v][N + num_v + t] == 1) && (kk != 0))
                {
                    e2.push_back({ cost_edge.Body[v][t], V[v].ID, static_cast<double>(t), T[t].rPU[0], T[t].rPU[1], -1, T[t].pass, T[t].rDO[0], T[t].rDO[1], -1 });
                }
                if ((T[t].dim == 1) && (RTV_Adj.Body[N + v][N + num_v + t] == 1) && (k != 0))
                {
                    e1.push_back({ cost_edge.Body[v][t], V[v].ID, static_cast<double>(t), T[t].rPU[0], -1, -1, T[t].pass, T[t].rDO[0], -1, -1 });
                }
            }
        }
    }

    size1_exist = get_assignments_size_one(e1);

    // At this point I should call a function to read from a .txt file if there are size1 trips from previous calls and read them.
    // When at the end of the code I print the assigned trips of size1, I should also save the information on the tasks and vehicles
    // that are linked to that assignments, so that to then be able to reconsider them in the optimization problem.

    // size1_assigned.push_back({ 2.0, 3.0, 11.5, 3});

    // Here I forcibly add the already assigned trips to the set of edges of the problem, so that for
    // sure they will be considered as optimization variables.

}

std::tuple<int, double, double, double, double> Edges::travel2(Vehicle V, Request R1, Request R2, int KoL)
{

    int value{ 0 };
    double cost{ 0.0 };
    double c_t_1{ 0.0 };
    double c_t_2{ 0.0 };
    double pass = R1.pass + R2.pass; //////////////////////////////////////////////

    double Delta1{ 0.0 }; // maybe I should declare it as a double
    double Delta2{ 0.0 }; // maybe I should declare it as a double

    if (pass <= V.c)
    {

        std::array<double, 2> v_pos = { V.x, V.y };
        std::array<double, 2> R1_orgn = { R1.xo, R1.yo };
        std::array<double, 2> R1_dest = { R1.xd, R1.yd };
        std::array<double, 2> R2_orgn = { R2.xo, R2.yo };
        std::array<double, 2> R2_dest = { R2.xd, R2.yd };

        if ((R1.priority == 0) && (R2.priority == 1))
        {
            Delta1 = Delta;
            Delta2 = DeltaInf;
        }
        else if ((R1.priority == 1) && (R2.priority == 1))
        {
            Delta1 = DeltaInf;
            Delta2 = DeltaInf;
        }
        else if ((R1.priority == 1) && (R2.priority == 0))
        {
            Delta1 = DeltaInf;
            Delta2 = Delta;
        }
        else
        {
            Delta1 = Delta;
            Delta2 = Delta;
        }

        if ((R1.status == 0) || (R1.status == 1))
        {
            double d1_1{ tt(v_pos, R1_orgn) };
            double d2_1{ tt(R1_orgn, R2_orgn) };
            double d3_1{ tt(R2_orgn, R1_dest) };
            double d4_1{ tt(R1_dest, R2_dest) };
            double d1_2{ tt(v_pos, R1_orgn) };
            double d2_2{ tt(R1_orgn, R2_orgn) };
            double d3_2{ tt(R2_orgn, R2_dest) };
            double d4_2{ tt(R2_dest, R1_dest) };
            switch (KoL)
            {
            case 1: // should create a namespace with 1 as ORDER1 and 2 as ORDER2
                if (((d1_1 + d2_1 + d3_1) <= R1.arr_t + Delta1) && ((d1_1 + d2_1 + d3_1 + d4_1) <= (R2.arr_t + Delta2)) && ((d1_1 + d2_1) <= (R2.sta_t + Delta2)))// Delta1 is an int !!!
                {
                    return { value = 1, cost = (d1_1 + d2_1 + d3_1 + d4_1), c_t_1 = (d1_1 + d2_1 + d3_1 - R1.arr_t), c_t_2 = (d1_1 + d2_1 + d3_1 + d4_1 - R2.arr_t), pass };
                }
                else
                {
                    return { value = 0, cost = 0.0, c_t_1 = 0.0, c_t_2 = 0.0, pass = 0.0 };
                }
                break;
            case 2:
                if (((d1_2 + d2_2 + d3_2) <= (R2.arr_t + Delta2)) && ((d1_2 + d2_2 + d3_2 + d4_2) <= (R1.arr_t + Delta1)) && ((d1_2 + d2_2) <= (R2.sta_t + Delta2)))
                {
                    return { value = 1, cost = (d1_2 + d2_2 + d3_2 + d4_2), c_t_1 = (d1_2 + d2_2 + d3_2 + d4_2 - R1.arr_t), c_t_2 = (d1_2 + d2_2 + d3_2 - R2.arr_t), pass };
                }
                else
                {
                    return{ value = 0, cost = 0.0, c_t_1 = 0.0, c_t_2 = 0.0, pass = 0.0 };
                }
                break;
            default: // do I really need this???
                break;
            }
        }
        else if (R1.status == 2) // when of the two tasks is already picked up
        {
            double d5_1{ tt(v_pos, R2_orgn) };
            double d6_1{ tt(R2_orgn, R1_dest) };
            double d7_1{ tt(R1_dest, R2_dest) };
            double d5_2{ tt(v_pos, R2_orgn) };
            double d6_2{ tt(R2_orgn, R2_dest) };
            double d7_2{ tt(R2_dest, R1_dest) };
            switch (KoL)
            {
            case 1:
                if (((d5_1 + d6_1) <= (R1.arr_t + Delta1)) && ((d5_1 + d6_1 + d7_1) <= (R2.arr_t + Delta2)) && (d5_1 <= (R2.sta_t + Delta2)))
                {
                    return { value = 1, cost = (d5_1 + d6_1 + d7_1), c_t_1 = (d5_1 + d6_1 - R1.arr_t), c_t_2 = (d5_1 + d6_1 + d7_1 - R2.arr_t), pass };
                }
                else
                {
                    return { value = 0, cost = 0.0, c_t_1 = 0.0, c_t_2 = 0.0,  pass }; // maybe they where already assigned to 0
                }
                break;
            case 2:
                if (((d5_2 + d6_2) <= (R2.arr_t + Delta2)) && ((d5_2 + d6_2 + d7_2) <= (R1.arr_t + Delta1)) && (d5_2 <= (R2.sta_t + Delta2)))
                {
                    return { value = 1, cost = (d5_2 + d6_2 + d7_2), c_t_1 = (d5_2 + d6_2 + d7_2 - R1.sta_t), c_t_2 = (d5_2 + d6_2 - R2.sta_t), pass };
                }
                else
                {
                    return { value = 0, cost = 0.0, c_t_1 = 0.0, c_t_2 = 0.0, pass };
                }
                break;
            default:
                break;
            }

        }
    }
    else
    {
        return { value = 0, cost = 0.0, c_t_1 = 0.0, c_t_2 = 0.0, pass };
    }
}

std::tuple<int, double, std::vector<double>, double> Edges::travel3(std::array<int, 3>& task_id, Vehicle V, std::array<int, 3>& pu_order, std::array<int, 3>& do_order, int KoL, std::vector<Request>& R)
{
    int value{ 0 };
    double trip_cost{ 0.0 };
    std::vector<double> trip; // this will contain in order the tasks ///////////////////////////////////////////
    double pass{ 0 }; ///////////////////////////////////////////////////////////////

    int id_r1{ task_id[0] };
    int id_r2{ task_id[1] };
    int id_r3{ task_id[2] };

    double num_pass{ R[id_r1].pass + R[id_r2].pass + R[id_r3].pass }; /////////////////////////////////////////////
    if (num_pass <= V.c)
    {
        for (int i_t3{ 0 }; i_t3 < pu_order.size(); ++i_t3)
        {
            for (int j_t3{ 0 }; j_t3 < do_order.size(); ++j_t3)
            {
                if ((pu_order[i_t3] != 0) && (do_order[j_t3] != 0))
                {
                    int task_1{ };
                    int task_2{ };
                    int task_3{ };
                    int task_4{ };
                    int task_5{ };
                    int task_6{ };

                    switch (i_t3)
                    {
                    case 0:
                        task_1 = id_r1;
                        task_2 = id_r2;
                        task_3 = id_r3;
                        break;
                    case 1:
                        task_1 = id_r1;
                        task_2 = id_r3;
                        task_3 = id_r2;
                        break;
                    case 2:
                        task_1 = id_r3;
                        task_2 = id_r1;
                        task_3 = id_r2;
                        break;
                    default:
                        break;
                    }

                    if (KoL == 1)
                    {
                        switch (j_t3)
                        {
                        case 0:
                            task_4 = id_r1;
                            task_5 = id_r2;
                            task_6 = id_r3;
                            break;
                        case 1:
                            task_4 = id_r1;
                            task_5 = id_r3;
                            task_6 = id_r2;
                            break;
                        case 2:
                            task_4 = id_r3;
                            task_5 = id_r1;
                            task_6 = id_r2;
                            break;
                        default:
                            break;
                        }
                    }
                    else if (KoL == 2)
                    {
                        switch (j_t3)
                        {
                        case 0:
                            task_4 = id_r2;
                            task_5 = id_r1;
                            task_6 = id_r3;
                            break;
                        case 1:
                            task_4 = id_r2;
                            task_5 = id_r3;
                            task_6 = id_r1;
                            break;
                        case 2:
                            task_4 = id_r3;
                            task_5 = id_r2;
                            task_6 = id_r1;
                            break;
                        default:
                            break;
                        }
                    }

                    std::array<double, 2> v_pos = { V.x, V.y };
                    std::array<double, 2> R1_orgn = { R[task_1].xo, R[task_1].yo };
                    std::array<double, 2> R4_dest = { R[task_4].xd, R[task_4].yd };
                    std::array<double, 2> R2_orgn = { R[task_2].xo, R[task_2].yo };
                    std::array<double, 2> R5_dest = { R[task_5].xd, R[task_5].yd };
                    std::array<double, 2> R3_orgn = { R[task_3].xo, R[task_3].yo };
                    std::array<double, 2> R6_dest = { R[task_6].xd, R[task_6].yd };

                    double d1{ tt(v_pos, R1_orgn) };
                    double d2{ tt(R1_orgn, R2_orgn) };
                    double d3{ tt(R2_orgn, R3_orgn) };
                    double d4{ tt(R3_orgn, R4_dest) };
                    double d5{ tt(R4_dest, R5_dest) };
                    double d6{ tt(R5_dest, R6_dest) };

                    if (((d1 + d2) <= R[task_2].sta_t + Delta) && ((d1 + d2 + d3) <= R[task_3].sta_t + Delta) && ((d1 + d2 + d3 + d4) <= R[task_4].arr_t + Delta) && ((d1 + d2 + d3 + d4 + d5) <= R[task_5].arr_t + Delta) && ((d1 + d2 + d3 + d4 + d5 + d6) <= R[task_6].arr_t + Delta))
                    {
                        value = 1;
                        trip_cost = d1 + d2 + d3 + d4 + d5 + d6;
                        trip = { R[task_1].ID, R[task_2].ID, R[task_3].ID, R[task_4].ID, R[task_5].ID, R[task_6].ID };
                        pass = num_pass;

                        return { value, trip_cost, trip, pass };
                    }
                }
            }
        }
    }

    return { value, trip_cost, trip, pass };
}

int Edges::get_assignments_size_one(std::vector<std::vector<double>>& e1)
{

    std::ifstream File;
    File.open("C:\\Users\\Matteo\\Desktop\\C++\\Optimal_task_assignment_Classes\\Assignments_Size_One.txt");

    if (!File)
    {
        std::cerr << "Unable to open file Assignments_Size_One.txt";
        exit(1); // call system to stop
    }

    std::string str;
    std::getline(File, str); // Throw the first line

    int i{ 0 };
    std::array<double, 4> a;

    std::string line;
    while (std::getline(File, line))
    {
        std::istringstream iss(line);

        while (!iss.eof())
        {
            if (!(iss))
                break;
            else // In the case there is a line with content to save (a new size1_assignment)
            {
                for (int j{ 0 }; j < 4; ++j)
                    iss >> a[j];

                size1_assigned.push_back({ a[1],a[2] }); // update the vector that contains the size1_assignments
                        // or maybe I should use the structure assignments to add them
                int flag{ 0 };
                for (unsigned __int64 j{ 0 }; j < e1.size(); ++j) // check if the size1_assignment has been already generated by the random couplings
                {
                    if ((a[1] == e1[j][1]) && (a[2] == e1[j][3]))
                    {
                        flag = 1;
                    }
                }

                if (flag == 0)
                    e1.push_back({ a[0],a[1],-1,a[2],-1,-1,a[3],a[2],-1,-1 });

                ++i;
            }
        }
    }

    int exist{ 0 };
    if (i > 0)
        exist = 1;

    return exist;
}

#endif EDGES_H