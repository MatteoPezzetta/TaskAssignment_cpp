#include <iostream>
#include <vector>
#include <ilcplex/ilocplex.h>
#include <ilconcert/iloexpression.h>

extern unsigned __int64 N;
extern unsigned __int64 num_v;

void populatebyrowMILP(IloModel model, IloNumVarArray x, IloRangeArray c,
    std::vector<std::vector<int>>& A_const1, std::vector<std::vector<int>>& A_const2,
    std::vector<int>& b_const1, std::vector<int>& b_const2, unsigned __int64 e_r, std::vector<double>& c_i_j, std::vector<int>& X_k_init)
{

    // I need: e_r, N, A_const1, b_const1, A_const2, b_const2

    //IloNumArray minArray(env, 5, 0.0, 0.0, 0.0, 0.0, 0.0);
    //IloNumArray maxArray(env, 5, 1.0, 1.0, 1.0, 1.0, 1.0);

    //std::vector<double> v1 = { 1.0, 0.64, 4.0, 5.0, 6.0 }; // cost vector c_i_j

    //model.add(IloMaximize(env, 1.00 * x[0] + 0.64 * x[1]));

    IloEnv env = model.getEnv(); // .getEnv() returns the environment

    // Generation of the vector of variables
    for (IloInt j = 0; j < e_r + N; ++j) // e_r+N
        x.add(IloBoolVar(env, 0, 1)); // 0 and 1 are lower and upper bounds

    // Cost vector
    IloNumArray cost(env);
    for (unsigned __int64 i{ 0 }; i < c_i_j.size(); ++i)
        cost.add(c_i_j[i]);

    // Construction of the cost function
    model.add(IloMinimize(env, IloScalProd(x, cost)));

    // For inequality constraints
    for (unsigned __int64 i{ 0 }; i < num_v; ++i) // I'll need here also the dimension of vector1 // num_v
    {
        IloNumArray constraint1(env);
        for (unsigned __int64 j{ 0 }; j < e_r + N; ++j) // e_r + N
            constraint1.add(A_const1[i][j]);
        c.add(IloScalProd(x, constraint1) <= b_const1[i]);
        constraint1.end();
    }

    // For equality constraints
    for (unsigned __int64 i{ 0 }; i < A_const2.size(); ++i) // N
    {
        IloNumArray constraint2(env);
        for (unsigned __int64 j{ 0 }; j < e_r + N; ++j) // e_r + N
            constraint2.add(A_const2[i][j]);
        c.add(IloScalProd(x, constraint2) == b_const2[i]);
        constraint2.end();
    }

    model.add(c);
    
    // Greedy assignmnet as initial guess
    IloNumVarArray startVar(env);
    IloNumArray startVal(env);
    for (int i{ 0 }; i < e_r; ++i)
    {
        startVar.add(x[i]);
        startVal.add(X_k_init[i]);
    }
    IloCplex cplex(model);
    cplex.addMIPStart(startVar, startVal);
    startVal.end();
    startVar.end();
    
    /*
    // VEDERE COME CREARE VETTORI IN AMBIENTE CPLEX
    // CON ELEMENTI DI VETTORI PREESISTENTI

    // VEDERE COME AGGIUNGERE ELEMENTI IN CODA A coef E constraint ARRAYS

    for (...)
    {
        IloNumArray contraint(env, { vector });
        c.add(IloScalProd(x, constraint1) <= b);
        or: c.add(IloScalProd(x, constraint2) == b);
    }

    for ( i...) // to add the constraints. i as the number of constraints -> maybe I need to divide inquality and equality
    {
        for ( j...) // to build the vector of the constraints. J as the number of variables
        {
            constraint1.add(A_const1[i][j])
            constraint2.add(A_const2[i][j])
        }

        c.add(IloScalProd(x, constraint1) <= b_constraint1[i];
        c.add(IloScalProd(x, constraint2) = b_constraint2[i];
    }
    */

    /*
    // Constraints
    IloNumArray constraint1(env, 5, 50, 31, 0, 0, 0);
    IloNumArray constraint2(env, 5, 3, -2, 0, 0, 0);
    IloNumArray b_constraint(env, 2, 50, -4);
    c.add(IloScalProd(x, constraint1) <= b_constraint[0]);
    c.add(IloScalProd(x, constraint2) >= b_constraint[1]);
    //c.add(50 * x[0] + 31 * x[1] <= 50);
    //c.add(3 * x[0] - 2 * x[1] >= -4);
    model.add(c);
    */
}

void populatebyrowLP(IloModel model_LP, IloNumVarArray x_LP, IloRangeArray c_LP,
    std::vector<std::vector<int>>& A_reb_in, std::vector<int>& A_reb_eq,
    std::vector<int>& b_reb_in, unsigned __int64 b_reb_eq, unsigned __int64 num_var, std::vector<double>& t_cost)
{
    IloEnv env_LP = model_LP.getEnv(); // .getEnv() returns the environment

    // Generation of the vector of variables
    for (IloInt j = 0; j < num_var; ++j) // e_r+N
        x_LP.add(IloNumVar(env_LP, 0, 1)); // 0 and 1 are lower and upper bounds
    // x_LP is defined in the declaration of the function

    // Cost vector
    IloNumArray cost_LP(env_LP);
    for (unsigned __int64 i{ 0 }; i < t_cost.size(); ++i)
        cost_LP.add(t_cost[i]);

    // Construction of the cost function
    model_LP.add(IloMinimize(env_LP, IloScalProd(x_LP, cost_LP)));

    // For inequality constraints
    for (unsigned __int64 i{ 0 }; i < A_reb_in.size(); ++i) // I'll need here also the dimension of vector1 // num_v
    {
        IloNumArray constraint1(env_LP);
        for (unsigned __int64 j{ 0 }; j < num_var; ++j) // e_r + N
            constraint1.add(A_reb_in[i][j]);
        c_LP.add(IloScalProd(x_LP, constraint1) <= b_reb_in[i]);
        constraint1.end();
    }

    // For equality constraints
        IloNumArray constraint2(env_LP);
        for (unsigned __int64 j{ 0 }; j < num_var; ++j) // e_r + N
            constraint2.add(A_reb_eq[j]);
        c_LP.add(IloScalProd(x_LP, constraint2) == b_reb_eq);
        constraint2.end();

    model_LP.add(c_LP);
}