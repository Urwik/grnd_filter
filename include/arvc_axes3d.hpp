#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

class axes3d
  {
  private:
    /* data */
  public:
    axes3d(){
      this->x = Eigen::Vector3f(1,0,0);
      this->y = Eigen::Vector3f(0,1,0);
      this->z = Eigen::Vector3f(0,0,1);
      this->rot_matrix = Eigen::Matrix3f::Identity();

    }
    ~axes3d(){}

    pcl::PointXYZ getPoint(Eigen::Vector3f _vector, Eigen::Vector4f _centroid){
      pcl::PointXYZ point;
      point.x = _vector(0) + _centroid(0);
      point.y = _vector(1) + _centroid(1);
      point.z = _vector(2) + _centroid(2);

      return point;
    }

    // operator << overloat to cout values
    friend std::ostream& operator<<(std::ostream& os, const arvc::axes3d& a)
    {
      // set precision for floating point output

      os << "X: [ " << a.x.x() << ", " << a.x.y() << ", " << a.z.z() << " ]" << endl;
      os << "Y: [ " << a.y.x() << ", " << a.y.y() << ", " << a.z.z() << " ]" << endl;
      os << "Z: [ " << a.z.x() << ", " << a.z.y() << ", " << a.z.z() << " ]" << endl;

      return os;
    }

    Eigen::Matrix3f getRotationMatrix(){

      this->rot_matrix << x.x(), y.x(), z.x(),
                          x.y(), y.y(), z.y(),
                          x.z(), y.z(), z.z();

      return this->rot_matrix;
    }

    Eigen::Matrix3f rot_matrix;
    Eigen::Vector3f x;
    Eigen::Vector3f y;
    Eigen::Vector3f z;
  };


  axes3d compute_eigenvectors3D(const PointCloud::Ptr& _cloud_in, const bool& force_ortogonal = false){
    axes3d _axes;
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*_cloud_in, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*_cloud_in, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();

    // Forzar a que el tercer vector sea perpendicular a los anteriores.
    if (force_ortogonal)
      eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
    
    _axes.x = eigDx.col(2).normalized();
    _axes.y = eigDx.col(1).normalized();
    _axes.z = eigDx.col(0).normalized();

    return _axes;
  }

  Eigen::Vector3f compute_eigenvalues3D(const PointCloud::Ptr& _cloud_in){

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*_cloud_in, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*_cloud_in, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);

    return eigen_solver.eigenvalues().reverse();
}