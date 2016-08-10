#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <iostream>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

Eigen::Vector3f unit_x = Eigen::Vector3f(1, 0, 0);
Eigen::Vector3f unit_y = Eigen::Vector3f(0, 1, 0);
Eigen::Vector3f unit_z = Eigen::Vector3f(0, 0, 1);

Eigen::Matrix4f create_rotation_matrix(float rx, float ry, float rz) {
    Eigen::Matrix4f ax =
      Eigen::Affine3f(Eigen::AngleAxisf(rx, unit_x)).matrix();
    Eigen::Matrix4f ay =
      Eigen::Affine3f(Eigen::AngleAxisf(ry, unit_y)).matrix();
    Eigen::Matrix4f az =
      Eigen::Affine3f(Eigen::AngleAxisf(rz, unit_z)).matrix();

  return az*(ay*ax);
}

Eigen::Matrix4f create_homogeneous_transform(float x, float y, float z,
                                             float rx, float ry, float rz){
    Eigen::Matrix4f ax =
      Eigen::Affine3f(Eigen::AngleAxisf(rx, unit_x)).matrix();
    Eigen::Matrix4f ay =
      Eigen::Affine3f(Eigen::AngleAxisf(ry, unit_y)).matrix();
    Eigen::Matrix4f az =
      Eigen::Affine3f(Eigen::AngleAxisf(rz, unit_z)).matrix();

    Eigen::Affine3f t(Eigen::Translation3f(Eigen::Vector3f(x,y,z)));

    return  t.matrix()*(az*(ay*ax));
}

Vector3f get_translation(Eigen::Matrix4f m, float x, float y, float z){
    Vector4f position(x, y, z, 1.0);
    return (m*position).head<3>();
}

Vector3f get_rotation(Eigen::Matrix4f m, float rx, float ry, float rz){
    Eigen::Matrix4f re = create_rotation_matrix(rx, ry, rz);
    Vector3f euler = ((m*re).block<3,3>(0,0).eulerAngles(0,1,2)) * RAD2DEG;
    return euler;
}

#endif // TRANSFORMATION_H
