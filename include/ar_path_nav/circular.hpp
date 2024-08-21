#include <cmath>
#include <eigen3/Eigen/Core>
#include <vector/vector.hpp>

class Circular
{
private:
    double r_;   //radious

    double x_;
    double y_;

    double radian_;

public:
    Circular();
    ~Circular();

    void set_radious(const double r){
        r_ = r;
    }

    void set_center(const double x, const double y){
        x_ = x;
        y_ = y;
    }

    void set_angle(const double radian){
        radian_ = radian;
    }

    void get_position(){
        
    }

};