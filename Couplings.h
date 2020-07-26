#ifndef COUPLINGS_H
#define COUPLINGS_H

#include <iostream>
#include <vector>
#include <array>
#include "Distance_function.h"
#include "V_Class.h"
#include "R_Class.h"
#include "Resizable_Vector.h"

extern double Delta;
extern double DeltaInf;

extern unsigned __int64 N;
extern unsigned __int64 num_v;

class Couplings
{

    public:
        Resizable_Vector <int> Adj;

        Resizable_Vector <int> priority_mat;

        Resizable_Vector <int> Real_Adj;

        Resizable_Vector <int> Kind_of_link;

        Resizable_Vector <int> Kind_of_link2;

        void RV_Compute(std::vector<Request>& R, std::vector<Vehicle>& V);

    private:

        // Function travel() for feasibility computation of size 1 trips
        std::tuple<int, int, double> travel(Vehicle V, Request R, std::vector<std::vector<double>>& priority_ass);

};

void Couplings::RV_Compute(std::vector<Request>& R, std::vector<Vehicle>& V)
{

    double max_cap{ get_max_cap(V) };
    double Delta1{ 0.0 };
    double Delta2{ 0.0 };

    double d1{ 0.0 };
    double d2{ 0.0 };
    double d3{ 0.0 };

    Adj.Resize_Body(N + num_v, N + num_v);

    priority_mat.Resize_Body(N + num_v, N + num_v);

    Real_Adj.Resize_Body(N, N);

    Kind_of_link.Resize_Body(N, N);

    Kind_of_link2.Resize_Body(N, N);

    Resizable_Vector <double> TripLength;
    TripLength.Resize_Body(N + num_v, N + num_v);

    double T_length = 0.0;

    std::vector<std::vector<double>> priority_ass;

    std::array<double, 2> orgn_i = { 0.0, 0.0 };
    std::array<double, 2> dest_i = { 0.0, 0.0 };
    std::array<double, 2> orgn_j = { 0.0, 0.0 };
    std::array<double, 2> dest_j = { 0.0, 0.0 };
    std::array<double, 6> b1 = {};
    std::array<double, 6> b2 = {};

    double ubd{ 0.0 };
    double lbd{ 0.0 };

    for (int i{ 0 }; i < N; ++i)
    {
        for (int j{ 0 }; j < N; ++j)
        {
            if ((R[i].pass + R[j].pass) <= max_cap)
            {
                orgn_i = { R[i].xo, R[i].yo };
                dest_i = { R[i].xd, R[i].yd };
                orgn_j = { R[j].xo, R[j].yo };
                dest_j = { R[j].xd, R[j].yd };

                if ((R[i].priority == 0) && (R[j].priority == 1))
                {
                    Delta1 = Delta;
                    Delta2 = DeltaInf;
                }
                else if ((R[i].priority) == 1 && (R[j].priority == 1))
                {
                    Delta1 = DeltaInf;
                    Delta2 = DeltaInf;
                }
                else if ((R[i].priority) == 1 && (R[j].priority == 0))
                {
                    Delta1 = DeltaInf;
                    Delta2 = Delta;
                }
                else
                {
                    Delta1 = Delta;
                    Delta2 = Delta;
                }

                d1 = tt(orgn_i, orgn_j);
                d2 = tt(orgn_j, dest_i);
                d3 = tt(dest_i, dest_j);

                b1 = { R[i].sta_t + Delta1,
                    -R[i].sta_t,
                    R[j].sta_t + Delta2 - d1,
                    -R[j].sta_t,
                    R[i].arr_t + Delta1 - d1 - d2,
                    R[j].arr_t + Delta2 - d1 - d2 - d3 };

                ubd = std::min({ b1[0], b1[2], b1[4], b1[5] });
                lbd = std::max({ b1[1], b1[3] });

                if ((ubd > lbd) && (i != j) && (ubd >= 0) && (R[j].status == 0))
                {
                    Adj.Body[i][j] = 1;
                    Adj.Body[j][i] = 1;
                    Real_Adj.Body[i][j] = 1;
                    //Adj1[i][j] = 1;
                    TripLength.Body[i][j] = d1 + d2 + d3;
                    Kind_of_link.Body[i][j] = 1;
                    Kind_of_link2.Body[i][j] = 1;
                }

                d2 = tt(orgn_j, dest_j);
                d3 = tt(dest_j, dest_i);

                b2 = { R[i].sta_t + Delta1,
                    -R[i].sta_t,
                    R[j].sta_t + Delta2 - d1,
                    -R[j].sta_t,
                    R[j].arr_t + Delta2 - d1 - d2,
                    R[i].arr_t + Delta1 - d1 - d2 - d3 };
                ubd = std::min({ b2[0], b2[2], b2[4], b2[5] });
                lbd = std::max({ b2[1], b2[3] });

                if ((ubd > lbd) && (i != j) && (ubd >= 0) && (R[j].status == 0))
                {
                    Adj.Body[i][j] = 1;
                    Adj.Body[j][i] = 1;
                    Real_Adj.Body[i][j] = 1;
                    //Adj2[i][j] = 1;
                    T_length = d1 + d2 + d3;

                    if (Kind_of_link.Body[i][j] == 1)
                        Kind_of_link2.Body[i][j] = 3;
                    else
                        Kind_of_link2.Body[i][j] = 2;

                    if ((TripLength.Body[i][j] == 0) || (T_length < TripLength.Body[i][j]))
                    {
                        Kind_of_link.Body[i][j] = 2;
                        TripLength.Body[i][j] = T_length;
                    }
                }

            }
        }
    }

    int value1{ 0 };
    int value2{ 0 };
    double value3{ 0.0 };

    // Compute feasibility of trips of size1
    for (int v{ 0 }; v < num_v; ++v)
    {
        for (int j{ 0 }; j < N; ++j)
        {
            //auto [ Adj[i+3][j], priority_mat[i+3][j], TripLength[i+3][j] ] = travel( V[i], R[j], priority_ass );
            auto [value1, value2, value3] = travel(V[v], R[j], priority_ass);

            Adj.Body[v + N][j] = value1;
            priority_mat.Body[N + v][j] = value2;
            TripLength.Body[N + v][j] = value3;

            Adj.Body[j][v + N] = Adj.Body[v + N][j]; // for graphicl purposes. I can avoid that statement
        }
    }
}

std::tuple<int, int, double> Couplings::travel(Vehicle V, Request R, std::vector<std::vector<double>>& priority_ass)
{
    std::array<double, 2> v_pos = { V.x, V.y };
    std::array<double, 2> R_orgn = { R.xo, R.yo };
    std::array<double, 2> R_dest = { R.xd, R.yd };

    double t1{ tt(v_pos, R_orgn) };
    double t2{ tt(R_orgn, R_dest) };
    int value{ 0 };
    int priority{ 0 };
    double cost_v_pick{ 0.0 };
    /*
    if (!priority_ass.empty()) // This should verify if the vector is empty or not
    {
        std::vector<double> V_R_couple = { R.ID, V.ID }; //////////////////////////////////////
        bool isPresent = std::find(priority_ass.begin(), priority_ass.end(), V_R_couple) != priority_ass.end();

        if (isPresent)
        {
            return { value = 1, priority = 2, cost_v_pick = t1 + t2 };
        }
    }
    */
    int Delta_Travel{ 0 };
    if (R.priority == 1)
        Delta_Travel = DeltaInf;
    else
        Delta_Travel = Delta; // The global Delta

    // Check for feasibility
    if ((R.pass <= V.c) && (t1 <= (R.sta_t + Delta_Travel)))
    {
        return { value = 1, priority = 1, cost_v_pick = t1 + t2 };
    }
    else
    {
        return { value = 0, priority = 0, cost_v_pick = 0 };
    }
}

#endif COUPLINGS_H