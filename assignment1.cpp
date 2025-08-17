#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    // Eigen::Matrix3f j;
    model << cos(rotation_angle* MY_PI / 180.0), -sin(rotation_angle* MY_PI / 180.0), 0, 0,
    sin(rotation_angle* MY_PI / 180.0), cos(rotation_angle* MY_PI / 180.0), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    // input is 45, 1, 0.1, 50

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f perspToOrtho;
    perspToOrtho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear*zFar,
        0, 0, 1, 0;
    float abszNear = zNear;
    if (zNear < 0) {
        abszNear = -zNear;
    }
    // Convert degrees to radians
    
    float t = abszNear * std::tan(eye_fov * MY_PI / 180.0 / 2);
    float r = aspect_ratio * t;

    // t is the top coordinate of the near plane (zNear) after perspective projection.

    // It is computed as:
    
    // float t = abszNear * std::tan(eye_fov * MY_PI / 180.0 / 2);
    // float r = aspect_ratio * t;, r is the right boundary.
    Eigen::Matrix4f MorthoScale;
    MorthoScale << 2/(r-(-r)), 0, 0, 0,
        0, 2/(t-(-t)), 0, 0,
        0, 0, 2/(zNear-zFar), 0,
        0, 0, 0, 1;

    // assume b=-t, l = -r  assume symmetry i.e. l = -r, b = -t
    Eigen::Matrix4f Mtranslate;
    Mtranslate << 1, 0, 0, -(r + (-r))/2,
        0, 1, 0, -(t + (-t))/2,
        0, 0, 1, -(zNear + zFar)/2,
        0, 0, 0, 1;

    projection = MorthoScale * Mtranslate * perspToOrtho;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
