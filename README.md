# GAMES101-Assignment01
Assignment 1 requires me to finish rasterizing a triangle by MVP transformation and rotate the triangle by pressing the button.

To transform the coordinates from one space to the next coordinate space we'll use several transformation matrices of which the most important are the model, view, and projection matrix. The vertex coordinates first start in local space as local coordinates and are then further processed to world coordinates, view coordinates, clip coordinates, and eventually end up as screen coordinates. ![img](https://learnopengl.com/img/getting-started/coordinate_systems.png)

1. Model matrix: A model matrix is a transformation matrix that translates, scales, and rotates an object to the position or orientation it should be in.  This means that the coordinates of the object will be transformed from local to world space

2. View Matrix: The view space is the space as seen from the camera's point of view.   This is usually accomplished with a combination of translations and rotations to translate/rotate the scene so that certain items are transformed to the front of the camera.  These combined transformations are generally stored inside a view matrix that transforms world coordinates to view space.

   View Matrix Summary:

   ​		Transform objects together with the camera, until the camera is at the origin, up at Y, look -Z.

   
3. Projection Matrix: The process of converting coordinates within a specified range to a standardized device coordinate system is called projection.  Therefore, 3D coordinates can be easily mapped to 2D standardized device coordinates using a projection matrix.

   - Orthographic Projection: Translate the object's center to the origin, then scale(length/width/height to **2**)


   - Perspective Projection: use a Mpersp->ortho Matrix to compress the object into a cuboid, then do the orthographic projection.

    

4. Viewport transformation: Map [-1,1]^2 to the plane [0,width]*[0,height] .
   

Back to the Assignment. I need to fill in a rotation matrix and a perspective projection matrix.

**For the model transformation function:** 

```CPP
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
```

**For the projection transformation function: **

```CPP
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
```

The last part of the assignment is to rotate the triangle around any axis. Obviously, we need to use the Rodriguez rotation formula.  

**get_rotation(Vector3f axis, float angle)**

```CPP
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
    model = R*model;
    return model;
}
```

I also need to add input in the main()

```CPP
	Eigen::Vector3f raxis(0, 0, 1);// Define default rotation axis
	double rangle = 0;//Define the rotation angle
	//input the axis;
    std::cout << "please input the rotation axis:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z();
```

Replace the function **get_model_matrix()** with **function get_rotation()**

```CPP
 //r.set_model(get_model_matrix(angle));//model transformation
 r.set_model(get_rotation(raxis, rangle));//set the  rotation
```

Result:
