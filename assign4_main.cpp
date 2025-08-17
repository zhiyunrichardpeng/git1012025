#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
// {
    
//     // TODO: Implement de Casteljau's algorithm
//     // return cv::Point2f();

//     if (control_points.size() == 1){
//         control_points[0];
//     }

//     std::vector<cv::Point2f> next_level_points;
//     // if there are three points:
//     // number_of_point = len(control_points);
//     // // create a list with length: number_of_point
//     // for i in range(number_of_point){
//     //     auto &p[i] = control_points[i]
//     // }

//     // for i in range(number_of_point-1){
//     //     vector[i] = control_points[i+1] - control_points[i];
//     // }
//     // for i in range(number_of_point-1):
//     //     new_control_point_of[i] = vector[i] * t + vector[i]

//     // auto &p_0 = control_points[0];
//     // auto &p_1 = control_points[1];
//     // auto &p_2 = control_points[2];
//     // auto &p_3 = control_points[3];    

//     // vector10 = p_1 - p_0;
//     // vector21 = p_2 - p_1;
//     // vector32 = p_3 - p_2;

//     // new_control_point_of_10 = vector10 * t + vector10
//     // - wrong, + p_0, not vector10.
//     // new_control_point_of_21 = vector21 * t + vector21
//     // new_control_point_of_32 = vector32 * t + vector32


//     // std::vector<cv::Point2f> next_level_points;
//     for (size_t i = 0; i < control_points.size() - 1; i++) {
//         auto p1 = control_points[i];
//         auto p2 = control_points[i + 1];
//         next_level_points.push_back(p1 + t * (p2 - p1));
//     }

//     return recursive_bezier(next_level_points, t);

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // Implement de Casteljau's algorithm
    if (control_points.size() == 1) {
        return control_points[0];
    }

    std::vector<cv::Point2f> next_level_points;
    for (size_t i = 0; i < control_points.size() - 1; i++) {
        auto p1 = control_points[i];
        auto p2 = control_points[i + 1];
        next_level_points.push_back(p1 + t * (p2 - p1));
    }

    return recursive_bezier(next_level_points, t);
}



        // return new_control_point;
    // else:control_points
    //     new_control_point = recursive_bezier(control_points, t)

// }

// void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
// {
//     // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
//     // recursive Bezier algorithm.


//     // remaining_point_list=[];
    
//     for (double t = 0.0; t <= 1.0; t += 0.001) 
//     {
//         // call 

//         auto the_remaining_point = recursive_bezier(control_points, t);
//         // auto point = recursive_bezier(control_points, t);
//         // remaining_point_list.append(the_remaining_point)


//         // return the remaining points
//         // del below
//         // auto point = std::pow(1 - t, 3main) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
//         //          3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

//         // how to make it green?
//         window.at<cv::Vec3b>(the_remaining_point.y, the_remaining_point.x)[1] = 255;
//     }    

// }

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // Iterate through all t = 0 to t = 1 with small steps
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // Green channel
    }    
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
