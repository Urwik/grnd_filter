#pragma once
#include <iostream>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

class plane
{
public:

    plane(){
        this->coeffs.reset(new pcl::ModelCoefficients);
        this->inliers.reset(new pcl::PointIndices);
        this->cloud.reset(new PointCloud);
        this->original_cloud.reset(new PointCloud);

        this->coeffs->values = {0,0,0,0};
        this->inliers->indices = {0};
        this->normal = Eigen::Vector3f(0,0,0);
        this->polygon = vector<Eigen::Vector3f>(5);

        this->projected_cloud.reset(new PointCloud);

        this->length = 0;
        this->width = 0;
    };

    // copy constructor
    plane(const plane& p)
    {
        this->coeffs.reset(new pcl::ModelCoefficients);
        this->inliers.reset(new pcl::PointIndices);
        this->cloud.reset(new PointCloud);
        this->original_cloud.reset(new PointCloud);

        *this->coeffs = *p.coeffs;
        *this->inliers = *p.inliers;
        *this->cloud = *p.cloud;
        *this->original_cloud = *p.original_cloud;

        this->normal = p.normal;
        this->centroid = p.centroid;
        this->tf = p.tf;
        this->eigenvectors = p.eigenvectors;
        this->eigenvalues = p.eigenvalues;
        this->polygon = p.polygon;
        this->length = p.length;
        this->width = p.width;
        this->color = p.color;
    };


    // move constructor
    plane(plane&& p)
    {
        this->coeffs.reset(new pcl::ModelCoefficients);
        this->inliers.reset(new pcl::PointIndices);
        this->cloud.reset(new PointCloud);
        this->original_cloud.reset(new PointCloud);

        *this->coeffs = *p.coeffs;
        *this->inliers = *p.inliers;
        *this->cloud = *p.cloud;
        *this->original_cloud = *p.original_cloud;

        this->normal = p.normal;
        this->centroid = p.centroid;
        this->tf = p.tf;
        this->eigenvectors = p.eigenvectors;
        this->eigenvalues = p.eigenvalues;
        this->polygon = p.polygon;
        this->length = p.length;
        this->width = p.width;
        this->color = p.color;
    };


    // copy assignment
    plane& operator=(const plane& p)
    {
        this->coeffs.reset(new pcl::ModelCoefficients);
        this->inliers.reset(new pcl::PointIndices);
        this->cloud.reset(new PointCloud);
        this->original_cloud.reset(new PointCloud);

        *this->coeffs = *p.coeffs;
        *this->inliers = *p.inliers;
        *this->cloud = *p.cloud;
        *this->original_cloud = *p.original_cloud;

        this->normal = p.normal;
        this->centroid = p.centroid;
        this->tf = p.tf;
        this->eigenvectors = p.eigenvectors;
        this->eigenvalues = p.eigenvalues;
        this->polygon = p.polygon;
        this->length = p.length;
        this->width = p.width;
        this->color = p.color;

        return *this;
    };

    // move assignment
    plane& operator=(plane&& p)
    {
        this->coeffs.reset(new pcl::ModelCoefficients);
        this->inliers.reset(new pcl::PointIndices);
        this->cloud.reset(new PointCloud);
        this->original_cloud.reset(new PointCloud);

        *this->coeffs = *p.coeffs;
        *this->inliers = *p.inliers;
        *this->cloud = *p.cloud;
        *this->original_cloud = *p.original_cloud;

        this->normal = p.normal;
        this->centroid = p.centroid;
        this->tf = p.tf;
        this->eigenvectors = p.eigenvectors;
        this->eigenvalues = p.eigenvalues;
        this->polygon = p.polygon;
        this->length = p.length;
        this->width = p.width;
        this->color = p.color;

        return *this;
    };

/*       plane(pcl::ModelCoefficientsPtr _coeffs, pcl::PointIndicesPtr _indices)
    {
    this->coeffs.reset(new pcl::ModelCoefficients);
    this->inliers.reset(new pcl::PointIndices);

    *this->coeffs = *_coeffs;
    *this->inliers = *_indices;
    
    cout << "PLANE OBJ INLIERS SIZE: " << this->inliers->indices.size() << endl;
    cout << "PLANE OBJ COEFFS: " << *this->coeffs << endl;
    cout << "-----------------------------" << endl;
    }; */

    ~plane(){
        this->coeffs->values = {0,0,0,0};
        this->inliers->indices = {0};
    };


    friend std::ostream& operator<<(std::ostream& os, const arvc::plane& p)
    {

        os << "Parameters: [ " << p.coeffs->values[0] << ", " << p.coeffs->values[1] << ", " << p.coeffs->values[2] << ", " << p.coeffs->values[3] << " ]" << endl;
        os << "Normal: [ " << p.normal.x() << ", " << p.normal.y() << ", " << p.normal.z() << " ]" << endl;
        os << "Centroid: [ " << p.centroid.x() << ", " << p.centroid.y() << ", " << p.centroid.z() << " ]" << endl;
        os << "Length: " << p.length << endl;
        os << "Width: " << p.width << endl;
        os << "Transform: " << endl << p.tf.matrix() << endl;
        os << "Eigenvalues: [ " << p.eigenvalues.x() << ", " << p.eigenvalues.y() << ", " << p.eigenvalues.z() << " ]" << endl;
        os << "Eigenvectors: " << endl << p.eigenvectors << endl;
        os << "Color: " << p.color << endl;
        return os;
    }


    void setPlane(const pcl::ModelCoefficientsPtr _coeffs, const pcl::PointIndicesPtr& _indices, const PointCloud::Ptr& _cloud_in){
        *this->coeffs = *_coeffs;
        *this->inliers = *_indices;
        *this->original_cloud = *_cloud_in;
        this->getNormal();
        this->getCloud();
        this->getEigenVectors();
        this->getEigenValues();
        this->getCentroid();
        this->getTransform();  
        this->getPolygon();
        this->color.random();
    };


    void setPlane(const pcl::ModelCoefficientsPtr _coeffs, const pcl::PointIndicesPtr& _indices, const PointCloud::Ptr& _cloud_in, const arvc::axes3d& _search_directions){
        *this->coeffs = *_coeffs;
        *this->inliers = *_indices;
        *this->original_cloud = *_cloud_in;
        this->getNormal();
        this->getCloud();
        this->getCentroid();
        this->compute_eigenDecomposition(true);
        this->forceEigenVectors(_search_directions);
        this->getTransform();  
        this->getPolygon();
        this->color.random();
    };


/*     void setPlane(Eigen::Vector3f _normal, pcl::PointIndicesPtr _indices, PointCloud::Ptr _cloud_in){
        this->coeffs->values = {_normal.x(), _normal.y(), _normal.z(), 0.0};
        *this->inliers = *_indices;
        *this->original_cloud = *_cloud_in;
        this->getCloud();
        this->getEigenVectors();
        this->getEigenValues();
        this->getCentroid();
        this->coeffs->values[3] = this->centroid.norm();
        this->getTransform();  
        this->getPolygon();
        this->color.random();
    }; */


    PointCloud::Ptr getCloud(){
        pcl::ExtractIndices<PointT> extract;
        
        extract.setInputCloud(this->original_cloud);
        extract.setIndices(this->inliers);
        extract.setNegative(false);
        extract.filter(*this->cloud);

        return this->cloud;
    };


    Eigen::Vector3f getNormal(){
        this->normal = Eigen::Vector3f(this->coeffs->values[0], this->coeffs->values[1], this->coeffs->values[2]);
        return normal;
    }


    void getEigenVectors(){
        this->eigenvectors = arvc::compute_eigenvectors3D(this->cloud, false);
    }


    void forceEigenVectors(arvc::axes3d _search_directions){
        this->eigenvectors.z = this->normal;

        this->eigenvectors.x = _search_directions.y; //TODO: CAMBIAR ESTO PARA ENCONTRAR CUAL ES LA DIRECCIÓN PERPENDICULAR A LA NORMAL CORRECTA
        this->eigenvectors.y = _search_directions.z;
    }


    void getEigenValues(){
        this->eigenvalues = arvc::compute_eigenvalues3D(this->cloud);
    }


    void projectOnPlane(){

        pcl::ProjectInliers<PointT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->cloud);
        proj.setModelCoefficients(this->coeffs);
        proj.filter(*this->projected_cloud);
        }


    void getCentroid(){
        pcl::compute3DCentroid(*this->cloud, this->centroid);
    }


    void compute_eigenDecomposition(const bool& force_ortogonal = false){
        arvc::axes3d _axes;
        pcl::compute3DCentroid(*this->cloud, this->centroid);
        pcl::computeCovarianceMatrixNormalized(*this->cloud, this->centroid, this->covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(this->covariance, Eigen::ComputeEigenvectors);

        // EIGEN VECTORS
        Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
        
        // Forzar a que el tercer vector sea perpendicular a los anteriores.
        if (force_ortogonal)
            eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
        
        this->eigenvectors.x = eigDx.col(2).normalized();
        this->eigenvectors.y = eigDx.col(1).normalized();
        this->eigenvectors.z = eigDx.col(0).normalized();
     
        // EIGEN VALUES
        this->eigenvalues = eigen_solver.eigenvalues().reverse();
    }


    void getPolygon(){
        PointCloud::Ptr relative_cloud(new PointCloud);

        this->projectOnPlane();
        pcl::transformPointCloud(*this->projected_cloud, *relative_cloud, this->tf.inverse());

        PointT max_point;
        PointT min_point;

        pcl::getMinMax3D(*relative_cloud, min_point, max_point);

        this->polygon[0] = Eigen::Vector3f(min_point.x, min_point.y, 0.0);
        this->polygon[1] = Eigen::Vector3f(min_point.x, max_point.y, 0.0);
        this->polygon[2] = Eigen::Vector3f(max_point.x, max_point.y, 0.0);
        this->polygon[3] = Eigen::Vector3f(max_point.x, min_point.y, 0.0);
        this->polygon[4] = this->polygon[0];

        this->length = abs(this->polygon[1].x() - this->polygon[2].x()); 
        this->width = abs(this->polygon[0].y() - this->polygon[1].y());

        if (this->length < this->width){
            swap(this->length, this->width);
            swap(this->eigenvalues.x(), this->eigenvalues.y());
        }


        for (int i = 0; i < this->polygon.size(); i++)
            this->polygon[i] = this->tf * this->polygon[i];
    }


    void getTransform(){
        this->tf.translation() = this->centroid.head<3>();
        this->tf.linear() = this->eigenvectors.getRotationMatrix();
    }


    pcl::ModelCoefficientsPtr coeffs;
    pcl::PointIndicesPtr inliers;
    
    PointCloud::Ptr cloud;
    PointCloud::Ptr original_cloud;
    Eigen::Vector3f normal;
    Eigen::Vector4f centroid;
    Eigen::Affine3f tf;
    vector<pcl::PointIndices> clusters;

    Eigen::Matrix3f covariance;
    arvc::axes3d eigenvectors;
    Eigen::Vector3f eigenvalues;
    
    vector<Eigen::Vector3f> polygon;
    float length;
    float width;
    
    arvc::color color;

    private:
        PointCloud::Ptr projected_cloud;
    

};
