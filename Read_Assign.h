#ifndef READ_ASSIGN_H
#define READ_ASSIGN_H

void Read_Assign_Requests(std::vector<Request>& R);

void Read_Assign_Vehicles(std::vector<Vehicle>& V);

double get_max_cap(std::vector<Vehicle>& V);

void write_title_Assignments_Size_One(std::ofstream & File);

void write_title_R(std::ofstream& File);

void write_title_V(std::ofstream& File);

#endif READ_ASSIGN_H
