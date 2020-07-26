#ifndef R_CLASS_H
#define R_CLASS_H

#include <fstream>

class Request
{

    public:
        double xo;
        double yo;
        double xd;
        double yd;
        double sta_t;
        double arr_t;
        double pass;
        double ID;
        int status;
        int priority;

        void print_info_R(std::ofstream& File)
        {
            File << xo << '\t' << yo << '\t' << xd << '\t' << yd << '\t' << sta_t << '\t' << arr_t << '\t'
                << pass << '\t' << ID << '\t' << status << '\t' << priority << '\n';
        }

};

#endif