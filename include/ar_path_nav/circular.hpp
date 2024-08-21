#include <cmath>
#include <eigen3/Eigen/Core>
#include <vector>

class Circular
{
private:
    double r_;   //radious

    double x_;
    double y_;

    double degree_;
    std::vector<double> pose_x_;
    std::vector<double> pose_y_;
    bool left_; 

    void calc_circular(){
        if (left_){ // right half circle
            for (double angle = -(degree_ / 2) ; angle < degree_ / 2 ; angle += 1.){
                double x = std::cos(r_) + x_;
                double y = std::sin(r_) + y_;
                pose_x_.push_back(x);
                pose_y_.push_back(y);
            }
        }

        else{   // left half circle
            for (double angle = -(degree_ / 2) ; angle < degree_ / 2 ; angle += 1.){
                double x = std::cos(r_) + x_;
                double y = std::sin(r_) + y_;
                pose_x_.push_back(x);
                pose_y_.push_back(y);
            }
        }
    }

public:
    Circular(){
        pose_x_.clear();
        pose_y_.clear();

        r_ = 0.0;
        x_ = 0.0;
        y_ = 0.0;
        
        degree_ = 0.0;
    }
    ~Circular();

    void set_radious(const double r){
        r_ = r;
    }

    void set_center(const double x, const double y){
        x_ = x;
        y_ = y;
    }

    void set_angle(const double degree){
        degree_ = degree;
    }

    void set_half(const bool left){
        left_ = left;
    }

    void set_poses(){
        calc_circular();
    }

    std::vector<double> get_x(){
        return pose_x_;
    }

    std::vector<double> get_y(){
        return pose_y_;
    }

    std::vector<double> gwt_yaw(){
        
    }
};