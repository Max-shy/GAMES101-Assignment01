#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.

    //rotation matrix:rotating around the z axis
    Eigen::Matrix4f rotate;
    float radian = rotation_angle / 180.0 * MY_PI;
    rotate << cos(radian), -1 * sin(radian), 0, 0,
        sin(radian), cos(radian), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    model = rotate * model;
    return model;
}


Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f M_persp2ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f M_ortho_scale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f M_ortho_trans = Eigen::Matrix4f::Identity();

    float angle = eye_fov * MY_PI / 180.0; // half fov angle

    auto t = -zNear * tan(angle / 2);//use fov calculate h
    auto r = t * aspect_ratio;// use aspect calculate r
    auto l = -r;
    auto b = -t;

    M_persp2ortho << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
    //scale Matrix
    M_ortho_scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    //translate Matrix
    M_ortho_trans << 1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f M_ortho = M_ortho_scale * M_ortho_trans;//orthographic Matrix
    projection = M_ortho * M_persp2ortho * projection;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    double alpha = angle / 180 * MY_PI;// define the alpha
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();//define the I
    Eigen::Matrix4f N, R;//define the N and R
    Eigen::Vector4f axis1;
    axis1 << axis.x(), axis.y(), axis.z(), 0;//need Vector4f

    //N is the Matrix behind sin(a)
    N << 0, -axis.z(), axis.y(), 0,
        axis.z(), 0, -axis.x(), 0,
        -axis.y(), axis.x(), 0, 0,
        0, 0, 0, 1;

    //Rodriguez rotation formula,R is a rotation Matrix
    R = cos(alpha) * I + (1 - cos(alpha)) * axis1 * axis1.transpose() + sin(alpha) * N;
    R(3, 3) = 1;//This is very important
    model = R * model;
    return model;
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


    rst::rasterizer r(700, 700);//width and height 
    Eigen::Vector3f eye_pos = { 0, 0, 5 };//camera position
    std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };//three vertices of triangle
    std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    int key = 0;
    int frame_count = 0;

    Eigen::Vector3f raxis(0, 0, 1);// Define default rotation axis
    double rangle = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_rotation(raxis, rangle));//model transformation
        r.set_view(get_view_matrix(eye_pos));//view transformation
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//projection projection
        //r.set_Rodrigues(get_rotation(raxis, rangle)); //set the  rotation

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//rasterize the triangle
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imwrite(filename, image);
        return 0;
    }

    //input the axis;
    std::cout << "please input the rotation axis:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z();

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_rotation(raxis, rangle));//model transformation
        r.set_view(get_view_matrix(eye_pos));//view transformation
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//projection projection(fov,aspect_ratio,n,f)
        //r.set_Rodrigues(get_rotation(raxis, rangle)); //set the  rotation

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);//rasterize the triangle

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        //change the angle in model transformation
        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'w') {//press w£¬Rotate 10¡ã counterclockwise
            rangle += 10;
        }
        else if (key == 's') {//press s,Rotate 10¡ã clockwise
            rangle -= 10;
        }
    }

    return 0;
}
