#include <array>
#include <cmath>

// Function for l1-norm computation
double l1_norm(std::array<double, 2>& size2_array) // passing arrays to functions the information on the size is lost, so we need to specify it
{
    double l1_norm_vect{ 0.0 };

    for (int i{ 0 }; i < 2; ++i)
        l1_norm_vect += std::abs(size2_array[i]);

    return l1_norm_vect;
}

// Function tt for travel time computation
double tt(std::array<double, 2>& orgn, std::array<double, 2>& dest)
{
    double vel_k = 0.08;
    double d{};
    std::array<double,2> diff;
    diff = { 0.0, 0.0 };

    for (int i{ 0 }; i < 2; ++i)
        diff[i] = (orgn[i] - dest[i]);

    d = l1_norm(diff);
    return vel_k * d;
    //return d;
}