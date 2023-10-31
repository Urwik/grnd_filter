#pragma once

// C++
#include <iostream>
#include <filesystem>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// using namespace std;


class viewer {

private:
    int cloud_count;

public:
    int v1,v2,v3,v4;
    vector<float> background_color;
    pcl::visualization::PCLVisualizer::Ptr view;
    bool save_cam_params;
    
    viewer(){
    this->view.reset(new pcl::visualization::PCLVisualizer("ARVC_VIEWER"));
    this->cloud_count = 0;
    this->v1 = 0;
    this->v2 = 0;
    this->v3 = 0;
    this->v4 = 0;

    this->background_color = {0.0, 0.0, 0.0};
    this->save_cam_params = false;
    }

    viewer(const string& name){
    this->view.reset(new pcl::visualization::PCLVisualizer(name));
    this->cloud_count = 0;
    this->v1 = 0;
    this->v2 = 0;
    this->v3 = 0;
    this->v4 = 0;

    this->background_color = {0.0, 0.0, 0.0};
    this->save_cam_params = false;

    }

    ~viewer(){}

void addCloud(const PointCloud::Ptr& cloud, const int& viewport=0){

    this->view->addPointCloud<PointT> (cloud, "cloud_" + to_string(this->cloud_count), viewport);
    this->cloud_count++;
}


void addCloud(const PointCloud::Ptr& cloud, const arvc::color& _color){
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud, _color.r, _color.g, _color.b);
    this->view->addPointCloud<PointT> (cloud, single_color, "cloud_" + to_string(this->cloud_count));
    this->cloud_count++;
}


void addOrigin(const int& _viewport= 0)
{
    this->view->addCoordinateSystem(1, "origin"+to_string(_viewport), _viewport);
    pcl::PointXYZ origin_point(0,0,0);
    this->view->addSphere(origin_point, 0.02, "sphere"+to_string(_viewport), _viewport);
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere"+to_string(_viewport), _viewport);
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere"+to_string(_viewport), _viewport);
    this->view->addText3D("origin", origin_point, 0.02, 1.0, 1.0, 1.0, "origin_text"+to_string(_viewport), _viewport);
    this->cloud_count++;
}


void addCube(const float& _range){
    this->view->addCube(-_range, _range, -_range, _range, -_range, _range, 1.0, 1.0, 0.0, "cube");
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cube");
    this->cloud_count++;

}


void addCube(const pcl::PointXYZ& _min, const pcl::PointXYZ& _max ){
    this->view->addCube(_min.x, _max.x, _min.y, _max.y, _min.z, _max.z, 1.0, 1.0, 0.0, "cube");
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cube");
    this->cloud_count++;

}


void addCoordinateSystem(const Eigen::Affine3f& tf, const int& _viewport=0){
    this->view->addCoordinateSystem(0.1, tf, "relative"+to_string(_viewport), _viewport);
    
    pcl::PointXYZ relative_origin(tf.translation().x(), tf.translation().y(), tf.translation().z());

    this->view->addText3D("relative", relative_origin, 0.02, 1.0, 1.0, 1.0, "relative_text"+to_string(_viewport), _viewport);
    this->cloud_count++;

}


void setViewports(const int& viewports){

    switch (viewports)
    {
    case 1:
    break;
    case 2:
    this->view->createViewPort(0.0, 0.0, 0.5, 1.0, this->v1);
    this->view->createViewPort(0.5, 0.0, 1.0, 1.0, this->v2);
    break;
    case 3:
    this->view->createViewPort(0.0, 0.5, 0.5, 1, this->v1);
    this->view->createViewPort(0.5, 0.5, 1.0, 1, this->v2);
    this->view->createViewPort(0.0, 0.0, 1.0, 0.5, this->v3);
    case 4:
    this->view->createViewPort(0.0, 0.5, 0.5, 1, this->v1);
    this->view->createViewPort(0.5, 0.5, 1.0, 1, this->v2);
    this->view->createViewPort(0.0, 0.0, 0.5, 0.5, this->v3);
    this->view->createViewPort(0.5, 0.0, 1.0, 0.5, this->v4);
    default:
    break;
    }
}


void addEigenVectors(const Eigen::Vector3f& _origin, const arvc::axes3d& _axis, const int& _viewport=0){

    pcl::PointXYZ origin(_origin.x(), _origin.y(), _origin.z());
    pcl::PointXYZ target_x(_axis.x.x() + _origin.x(), _axis.x.y() + _origin.y(), _axis.x.z() + _origin.z());
    pcl::PointXYZ target_y(_axis.y.x() + _origin.x(), _axis.y.y() + _origin.y(), _axis.y.z() + _origin.z());
    pcl::PointXYZ target_z(_axis.z.x() + _origin.x(), _axis.z.y() + _origin.y(), _axis.z.z() + _origin.z());

    this->view->addArrow<pcl::PointXYZ, pcl::PointXYZ>(target_x, origin, 1.0, 0.0, 0.0, false, "eigenvector_x"+to_string(this->cloud_count), _viewport);
    this->view->addArrow<pcl::PointXYZ, pcl::PointXYZ>(target_y, origin, 0.0, 1.0, 0.0, false, "eigenvector_y"+to_string(this->cloud_count), _viewport);
    this->view->addArrow<pcl::PointXYZ, pcl::PointXYZ>(target_z, origin, 0.0, 0.0, 1.0, false, "eigenvector_z"+to_string(this->cloud_count), _viewport);
    this->cloud_count++;

}


void addPolygon(const vector<Eigen::Vector3f>& _polygon, arvc::color _color = arvc::color(255,255,255), const int& _viewport=0){

    pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (auto& point : _polygon)
    {
        polygon_cloud->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
    
    polygon_cloud->push_back(pcl::PointXYZ(_polygon[0].x(), _polygon[0].y(), _polygon[0].z()));
    _color.normalized();
    this->view->addPolygon<pcl::PointXYZ>(polygon_cloud, _color.r, _color.g, _color.b, "polygon"+to_string(this->cloud_count));
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "polygon"+to_string(this->cloud_count));
    this->cloud_count++;

}

void addPlane(const arvc::plane& _plane, arvc::color _color = arvc::color(255,255,255), const int& _viewport=0){

    this->view->addPlane(*_plane.coeffs, _plane.centroid.x(), _plane.centroid.y(), _plane.centroid.z(), "plane_"+to_string(this->cloud_count), _viewport);
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, _color.r / 255, _color.g / 255, _color.b / 255, "plane_"+to_string(this->cloud_count), _viewport);
    this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "plane_"+to_string(this->cloud_count), _viewport);
    this->cloud_count++;

}

void close(){
    this->view->close();
}

void clear(){
    this->view->removeAllPointClouds();
    this->view->removeAllShapes();
    this->cloud_count = 0;
}

void show(){

    this->view->setBackgroundColor(this->background_color[0], this->background_color[1], this->background_color[2]);
    
    try
    {
        this->view->loadCameraParameters("cam_params.txt");
    }
    catch(const std::exception& e)
    {
        {}
    }

    while(!this->view->wasStopped())
    {
        if(this->save_cam_params)
            this->view->saveCameraParameters("cam_params.txt");
        
        this->view->spinOnce(100);
     
    }
}

};
