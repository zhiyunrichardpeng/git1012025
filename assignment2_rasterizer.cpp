// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // Convert point to Eigen vector
    Eigen::Vector2f p(x, y);
    
    // vertices
    Eigen::Vector2f v0 = v[0].head<2>(); // first 2 elements
    Eigen::Vector2f v1 = v[1].head<2>();
    Eigen::Vector2f v2 = v[2].head<2>();
    
    // Vectors from point to vertices
    Eigen::Vector2f pv0 = v0 - p;
    Eigen::Vector2f pv1 = v1 - p;
    Eigen::Vector2f pv2 = v2 - p;
    
    // Cross products to check winding
    float cross0 = pv0.x() * pv1.y() - pv0.y() * pv1.x();
    float cross1 = pv1.x() * pv2.y() - pv1.y() * pv2.x();
    float cross2 = pv2.x() * pv0.y() - pv2.y() * pv0.x();
    
    // Check if all cross products have same sign
    return (cross0 > 0 && cross1 > 0 && cross2 > 0) || 
           (cross0 < 0 && cross1 < 0 && cross2 < 0);    
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // t is triangle, v is the vertices of the triangle.
    
    // Find the bounding box of current triangle
    float left_min = std::numeric_limits<float>::max();
    float right_max = std::numeric_limits<float>::lowest();
    float up_max = std::numeric_limits<float>::lowest();
    float down_min = std::numeric_limits<float>::max();

    // Get screen-space vertices
    const auto& verts = t.v;
    
    // Find min/max coordinates
    for (int i = 0; i < 3; ++i) {
        left_min = std::min(left_min, verts[i].x());
        right_max = std::max(right_max, verts[i].x());
        down_min = std::min(down_min, verts[i].y());
        up_max = std::max(up_max, verts[i].y());
    }

    // Convert to pixel coordinates (clamp to screen bounds)
    // Or, I can call it the pixel box count index.
    int min_x = std::max(0, static_cast<int>(std::floor(left_min)));
    int max_x = std::min(width - 1, static_cast<int>(std::ceil(right_max)));
    // For a length 3 units segment line, there is only 0, 1, 2 box index to count them. So width -1.

    int min_y = std::max(0, static_cast<int>(std::floor(down_min)));
    int max_y = std::min(height - 1, static_cast<int>(std::ceil(up_max)));
    
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            if (insideTriangle(x + 0.5f, y + 0.5f, t.v)){
                //get barycentric coordinates
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // If so, use the following code to get the interpolated z value.
                //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                //z_interpolated *= w_reciprocal;

                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                // Depth test
                if (z_interpolated < depth_buf[get_index(x, y)]) {
                    // Update depth buffer
                    depth_buf[get_index(x, y)] = z_interpolated;
                    
                    // Set pixel color
                    Eigen::Vector3f point(x, y, 1.0f);
                    set_pixel(point, t.getColor());
                }                    

            }
            // image[x][y] = insideTriangle(tri, x + 0.5, y + 0.5)
            // + 0.5, is the pixel index count + 0.5 to be the real pixel's center's coordinates.
        }
    }
        


}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on