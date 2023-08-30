#ifndef CLASS_PC_LOAD
#define CLASS_PC_LOAD

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

using std::cout;
using std::endl;

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


class ClassPcLoad
{
private:

    ros::Publisher path_puber_ ;
    ros::NodeHandle nh_;
    
public:
    ClassPcLoad(const ros::NodeHandle nh_in_);
    ~ClassPcLoad();
    double get_ram_free_percent();
};

ClassPcLoad::ClassPcLoad(const ros::NodeHandle nh_in_): nh_{nh_in_}
{
    path_puber_  = nh_.advertise<std_msgs::Float64>( "/ram_free_percent", 10);

    ros::Rate r(0.5);
    while (ros::ok())
    {
        get_ram_free_percent();
        r.sleep();
    }
    

}

ClassPcLoad::~ClassPcLoad()
{
}


double ClassPcLoad::get_ram_free_percent()
{
    try
    {
        std::string top_result = exec("top -b -n 1");

        std::string start_str = "MiB Mem :";
        std::string end_str = "buff/cache";

        std::size_t start_pos = top_result.find(start_str) + start_str.size();
        std::size_t end_pos = top_result.find(end_str) ;

        std::string ram_row = top_result.substr(start_pos, end_pos-start_pos);
        // cout << ram_row << endl;

        // Total RAM 
        std::string total_ram_end_str = "total,";

        std::size_t total_ram_start_pos = 0;
        std::size_t total_ram_end_pos = ram_row.find(total_ram_end_str) ;

        std::string total_ram_MB_str = ram_row.substr(total_ram_start_pos, total_ram_end_pos);

        // Free RAM 
        std::string free_ram_start_str = "total,";
        std::string free_ram_end_str = "free,";
        std::size_t free_ram_start_pos = ram_row.find(free_ram_start_str) + free_ram_start_str.size();
        std::size_t free_ram_end_pos = ram_row.find(free_ram_end_str) ;
        std::string free_ram_MB_str = ram_row.substr(free_ram_start_pos, free_ram_end_pos-free_ram_start_pos);

        // cout << "total_ram_MB_str  >" << total_ram_MB_str << "<" << endl;
        // cout << "free_ram_MB_str  >" << free_ram_MB_str << "<" << endl;

        double total_mb = std::stod(total_ram_MB_str);
        double free_mb = std::stod(free_ram_MB_str);
        double free_percent = free_mb/total_mb * 100.0;

        // cout << "total_ram_MB  >" << total_mb << "<" << endl;
        // cout << "free_ram_MB  >" << free_mb << "<" << endl;
        // cout << "free_percent  >" << free_percent << "<" << endl;

        std_msgs::Float64 msg;
        msg.data = free_percent;

        path_puber_.publish( msg );

        return free_percent;
        
        
    }
    catch(...)
    {
        return -1.0;
    }
    
    
}



#endif