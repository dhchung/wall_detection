#include "detect_wall.h"

DetectWall::DetectWall() {
    min_height = 0.0f;
    max_height = 0.2f;
    occupancy_mat = cv::Mat::zeros(400, 400, CV_8UC1);
    occupancy_length = 200.0f;
    occupancy_resolution = 0.2f;
}

DetectWall::~DetectWall() {

}

Eigen::Matrix3Xf DetectWall::RemoveReflection(Eigen::Matrix3Xf & pt_cld) {

    std::vector<int> indices;
    float th = 0.0f;
    visit_lambda(pt_cld.row(2), [&indices, &th](float v, int i , int j) {
        if(v>th) {
            // indices.push_back(std::make_pair(i,j));
            indices.push_back(j);
        }
    });
    float x_min = -7.0f;
    float x_max = 0.0f;

    float y_min = -1.5f;
    float y_max = 1.5f;

    float z_min = 0.0f;
    float z_max = 2.3f;

    std::vector<int> refined_indices;

    for(size_t i = 0; i < indices.size(); ++i) {
        bool passed = true;
        if(pt_cld(0, indices[i]) > x_min && pt_cld(0, indices[i]) < x_max) {
            if(pt_cld(1, indices[i]) > y_min && pt_cld(1, indices[i]) < y_max) {
                if(pt_cld(2, indices[i]) > z_min && pt_cld(2, indices[i]) < z_max) {
                    passed = false;
                }
            }
        }
        if(passed) {
            refined_indices.push_back(indices[i]);
        }
    }



    Eigen::Matrix3Xf return_pt;
    return_pt.resize(3, refined_indices.size());
    for(size_t i = 0; i < refined_indices.size(); ++i) {
        return_pt.col(i) = pt_cld.col(refined_indices[i]);
    }

    return return_pt;
}

std::vector<int> DetectWall::GetWallCandidates(Eigen::Matrix3Xf & pt_cld){
    std::vector<int> indices;
    float th_l = min_height;
    float th_h = max_height;

    visit_lambda(pt_cld.row(2), [&indices, &th_l, &th_h](float v, int i , int j) {
        if(v>th_l && v<th_h) {
            indices.push_back(j);
        }
    });
    return indices;
}

std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>>
    DetectWall::GetOccupancyMap(Eigen::Matrix3Xf & pt_wall){

    std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> wall_lines;


    if(pt_wall.cols() > 0) {

        int grid_length = occupancy_length/occupancy_resolution;
        bool odd;
        if(grid_length%2 == 0) {
            odd = false;
        } else {
            odd = true;
        }
        //if odd is true, the vehicle center is located at the center of a bin
        //else it is located in the coner
        // occupancy_mat.resize(grid_length, grid_length);
        occupancy_mat = cv::Mat::zeros(grid_length, grid_length, CV_8UC1);

        std::vector<std::vector<float>> max_map(grid_length, std::vector<float>(grid_length, -2000.0f));
        std::vector<std::vector<float>> min_map(grid_length, std::vector<float>(grid_length, 2000.0f));
        std::vector<std::pair<int, int>> occupied_list;

        std::vector<std::vector<int>> count_map(grid_length, std::vector<int>(grid_length, 0));

        float max_length;
        if(odd) {
            max_length = (grid_length/2)*occupancy_resolution + occupancy_resolution/2.0f;
        } else {
            max_length = (grid_length/2)*occupancy_resolution;
        }

        for(int i = 0; i < pt_wall.cols(); ++i) {
            float pt_x = -pt_wall(0,i);
            float pt_y = -pt_wall(1,i);
            if(abs(pt_x) > max_length || abs(pt_y) > max_length) {
                continue;
            }
            int bin_x;
            int bin_y;
            if(odd) {
                if(pt_x>=0) {
                    bin_x = int(grid_length/2) + int((pt_x + occupancy_resolution/2)/occupancy_resolution)-1;
                } else {
                    bin_x = int(grid_length/2) + int((pt_x - occupancy_resolution/2)/occupancy_resolution)+1;
                }
                if(pt_y>=0) {
                    bin_y = int(grid_length/2) + int((pt_y + occupancy_resolution/2)/occupancy_resolution)-1;
                } else {
                    bin_y = int(grid_length/2) + int((pt_y - occupancy_resolution/2)/occupancy_resolution)+1;
                }

            } else {
                if(pt_x>=0) {
                    bin_x = int(grid_length/2) + int(pt_x/occupancy_resolution);
                } else {
                    bin_x = int(grid_length/2) + int(pt_x/occupancy_resolution) - 1;
                }
                if(pt_y>=0) {
                    bin_y = int(grid_length/2) + int(pt_y/occupancy_resolution) - 1;
                } else {
                    bin_y = int(grid_length/2) + int(pt_y/occupancy_resolution);
                }
            }
            max_map[bin_x][bin_y] = std::max(max_map[bin_x][bin_y], pt_wall(2,i));
            min_map[bin_x][bin_y] = std::min(min_map[bin_x][bin_y], pt_wall(2,i));
            ++count_map[bin_x][bin_y];
            occupied_list.push_back(std::pair<int, int>{bin_x, bin_y});
        }

        for(size_t i = 0; i < occupied_list.size(); ++i) {
            int bin_x = occupied_list[i].first;
            int bin_y = occupied_list[i].second;
            if(min_map[bin_x][bin_y] > min_height + 0.2) {
                continue;
            } 
            if(max_map[bin_x][bin_y] - min_map[bin_x][bin_y] < 0.1) {
                continue;
            }
            if(count_map[bin_x][bin_y] < occupancy_resolution*occupancy_resolution*occupancy_resolution * 10.0f) {
                continue;
            }

            occupancy_mat.at<u_int8_t>(bin_x, bin_y) = 255;
        }

        line_map = occupancy_mat.clone();
        cv::cvtColor(line_map, line_map, cv::COLOR_GRAY2BGR);
        std::vector<cv::Vec4i> linesP;
        cv::HoughLinesP(occupancy_mat, linesP, 1, CV_PI/180, 5, 5, 3);



        for( size_t i = 0; i < linesP.size(); i++ )
        {
            cv::Vec4i l = linesP[i];
            // cv::line(line_map, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 1, cv::LINE_AA);
            std::pair<float, float> line_start;
            std::pair<float, float> line_end;

            if(odd){
                line_start.first = -(float(l[1] - int(grid_length/2)) * occupancy_resolution);
                line_start.second = -(float(l[0] - int(grid_length/2)) * occupancy_resolution);
                line_end.first = -(float(l[3] - int(grid_length/2)) * occupancy_resolution);
                line_end.second = -(float(l[2] - int(grid_length/2)) * occupancy_resolution);                
            } else {
                line_start.first = -(float(l[1] - int(grid_length/2) + 1) * occupancy_resolution - occupancy_resolution/2);
                line_start.second = -(float(l[0] - int(grid_length/2) + 1) * occupancy_resolution - occupancy_resolution/2);
                line_end.first = -(float(l[3] - int(grid_length/2) + 1) * occupancy_resolution - occupancy_resolution/2);
                line_end.second = -(float(l[2] - int(grid_length/2) + 1) * occupancy_resolution - occupancy_resolution/2);
            }
            std::pair<std::pair<float, float>, std::pair<float, float>> line_pair{line_start, line_end};
            wall_lines.push_back(line_pair);
        }


        // cv::imshow("Line Map", line_map);
        // cv::waitKey(1);


    } else {
        occupancy_mat = cv::Mat::zeros(400, 400, CV_8UC1);
    }

    return wall_lines;
}
