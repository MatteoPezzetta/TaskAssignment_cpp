#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

//#include <iostream>
#include <vector>
#include <fstream>
// Data structure for the requests

// Data structure for the trips
struct Trip
{

    std::vector<double> rPU; // order of tasks at pick up // was int
    std::vector<double> rDO; // order of tasks at drop off // was int
    int order;
    int status;
    int dim; // dimension of the trip
    int possible_order = 0;
    double pass; // was int
    double index1; // don't know if I am going to actually use those fields
    double index2;

};

struct FirstColumnOnlyCmp
{
    bool operator()(const std::vector<double>& lhs,
        const std::vector<double>& rhs) const
    {
        return lhs[0] > rhs[0];
    }
};

#endif
