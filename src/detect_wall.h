#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

template<typename Func>
struct lambda_as_visitor_wrapper : Func {
    lambda_as_visitor_wrapper(const Func& f) : Func(f) {}
    template<typename S,typename I>
    void init(const S& v, I i, I j) { return Func::operator()(v,i,j); }
};

template<typename Mat, typename Func>
void visit_lambda(const Mat& m, const Func& f)
{
    lambda_as_visitor_wrapper<Func> visitor(f);
    m.visit(visitor);
}


class DetectWall{
public:
    DetectWall();
    ~DetectWall();
    Eigen::Matrix3Xf RemoveReflection(Eigen::Matrix3Xf & pt_cld);
    std::vector<int> GetWallCandidates(Eigen::Matrix3Xf & pt_cld);

    cv::Mat occupancy_mat;
    cv::Mat line_map;

    std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> 
        GetOccupancyMap(Eigen::Matrix3Xf & pt_wall);

    float occupancy_resolution;
    float occupancy_length;

    float min_height;
    float max_height;

};