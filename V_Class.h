#ifndef V_CLASS_H
#define V_CLASS_H

#include <fstream>

// Data structure for the vehicles
class Vehicle
{

    public:
        double x;
        double y;
        double c;
        double ID;
        int status;

        void print_info_V(std::ofstream& File)
        {
            File << x << '\t' << y << '\t' << c << '\t' << ID << '\t' << status << '\n';
        }

};

#endif V_CLASS_H