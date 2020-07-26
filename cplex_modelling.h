#ifndef CPLEX_MODELLING_H
#define CPLEX_MODELLING_H

// Function to populate the cplex model so that then run the optimization
void populatebyrowMILP(IloModel model, IloNumVarArray x, IloRangeArray c,
    std::vector<std::vector<int>>& A_const1, std::vector<std::vector<int>>& A_const2,
    std::vector<int>& b_const1, std::vector<int>& b_const2, unsigned __int64 e_r, std::vector<double>& c_i_j, std::vector<int>& X_k_init);

void populatebyrowLP(IloModel model_LP, IloNumVarArray x_LP, IloRangeArray c_LP,
    std::vector<std::vector<int>>& A_reb_in, std::vector<int>& A_reb_eq,
    std::vector<int>& b_reb_in, unsigned __int64 b_reb_eq, unsigned __int64 num_var, std::vector<double>& t_cost);

#endif
