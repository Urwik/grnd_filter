#pragma once

// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <numeric>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Dense>
#include <pcl/io/obj_io.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>



namespace fs = std::filesystem;

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

typedef pcl::PointXYZL PointL;
typedef pcl::PointCloud<PointL> PointCloudL;

typedef pcl::PointXYZLNormal PointLN;
typedef pcl::PointCloud<PointLN> PointCloudLN;

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"



//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////


struct metrics
{
  float precision = 0.0;
  float recall = 0.0;
  float f1_score = 0.0;
  float accuracy = 0.0;
};

struct conf_matrix
{
  int TP = 0;
  int FP = 0;
  int TN = 0;
  int FN = 0;
};

struct gt_indices
{
  pcl::IndicesPtr ground;
  pcl::IndicesPtr truss;
};

struct cm_indices
{
  pcl::IndicesPtr tp_idx;
  pcl::IndicesPtr fp_idx;
  pcl::IndicesPtr tn_idx;
  pcl::IndicesPtr fn_idx;
};

namespace arvc
{

  /**
   * @brief Get the Ground Truth object
   * 
   */
  gt_indices
  getGroundTruthIndices(PointCloudI::Ptr &_cloud_intensity)
  {
    gt_indices _gt_indices;
    _gt_indices.ground.reset(new pcl::Indices);
    _gt_indices.truss.reset(new pcl::Indices);
    pcl::PassThrough<pcl::PointXYZI> pt;
    pt.setInputCloud(_cloud_intensity);
    pt.setFilterFieldName("intensity");
    pt.setFilterLimits(0, 0);
    pt.setNegative(false);
    pt.filter(*_gt_indices.ground);
    pt.setNegative(true);
    pt.filter(*_gt_indices.truss);

    return _gt_indices;
  }


  /**
   * @brief Get the Ground Truth object
   * 
   */
  gt_indices
  getGroundTruthIndices(PointCloudL::Ptr &_cloud_label)
  {
    gt_indices _gt_indices;
    _gt_indices.ground.reset(new pcl::Indices);
    _gt_indices.truss.reset(new pcl::Indices);
    pcl::PassThrough<pcl::PointXYZL> pt;
    pt.setInputCloud(_cloud_label);
    pt.setFilterFieldName("label");
    pt.setFilterLimits(0, 0);
    pt.setNegative(false);
    pt.filter(*_gt_indices.ground);
    pt.setNegative(true);
    pt.filter(*_gt_indices.truss);

    return _gt_indices;
  }

  /**
   * @brief Get the Ground Truth object
   * 
   */
  gt_indices
  getGroundTruthIndices(PointCloudLN::Ptr &_cloud_label)
  {
    PointCloudL::Ptr _cloud_label_tmp (new PointCloudL);
    pcl::copyPointCloud(*_cloud_label, *_cloud_label_tmp);

    return getGroundTruthIndices(_cloud_label_tmp);
  }


  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  PointCloudI::Ptr 
  readCloudWithIntensity (fs::path _path)
  {
    PointCloudI::Ptr _cloud_intensity (new PointCloudI);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud_intensity);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud_intensity);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud_intensity;
  }

  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  PointCloudL::Ptr 
  readCloudWithLabel (fs::path _path)
  {
    PointCloudL::Ptr _cloud_label (new PointCloudL);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud_label);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud_label);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud_label;
  }


  PointCloudLN::Ptr 
  readCloudWithLabelNormal (fs::path _path)
  {
    PointCloudLN::Ptr _cloud_label (new PointCloudLN);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud_label);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud_label);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud_label;
  }

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr readPointCloud(const std::string& path)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    std::map<std::string, int> ext_map = { {".pcd", 0}, {".ply", 1} };

    switch (ext_map[fs::path(path).extension().string()])
    {
        case 0:
        {
            pcl::PCDReader reader;
            reader.read(path, *cloud);
            break;
        }
        case 1:
        {
            pcl::PLYReader reader;
            reader.read(path, *cloud);
            break;
        }
        default:
        {
            std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
            break;
        }
    }

    return cloud;
}


  /**
   * @brief Lee una nube de puntos en formato .pcd o .ply
   * 
   * @param path Ruta de la nube de puntos
   * @return PointCloudI::Ptr 
   */
  PointCloud::Ptr 
  readCloud (fs::path _path)
  {
    PointCloud::Ptr _cloud (new PointCloud);
    map<string, int> ext_map = {{".pcd", 0}, {".ply", 1}};

    switch (ext_map[_path.extension().string()])
    {
      case 0: {
        pcl::PCDReader pcd_reader;
        pcd_reader.read(_path.string(), *_cloud);
        break;
      }
      case 1: {
        pcl::PLYReader ply_reader;
        ply_reader.read(_path.string(), *_cloud);
        break;
      }
      default: {
        std::cout << "Format not compatible, it should be .pcd or .ply" << std::endl;
        break;
      }
    }

    return _cloud;
  }

  float 
  get_max_length(PointCloud::Ptr &cloud)
  {

    float _max_length;

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    /// the signs are different and the box doesn't get correctly oriented in some cases.
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
                                                                                    

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    Eigen::Vector3f max_values;


    max_values.x() = std::abs(maxPoint.x - minPoint.x);
    max_values.y() = std::abs(maxPoint.y - minPoint.y);
    max_values.z() = std::abs(maxPoint.z - minPoint.z);

    return max_values.maxCoeff();
  }

  /**
   * @brief Convierte una nube de entrada con intensidad a solo coordenadas XYZ
  */
  PointCloud::Ptr
  parseToXYZ(PointCloudI::Ptr &_cloud_intensity)
  {
    PointCloud::Ptr _cloud_xyz (new PointCloud);
    pcl::copyPointCloud(*_cloud_intensity, *_cloud_xyz);

    return _cloud_xyz;
  }

  /**
   * @brief Convierte una nube de entrada con intensidad a solo coordenadas XYZ
  */
  PointCloud::Ptr
  parseToXYZ(PointCloudL::Ptr &_cloud_label)
  {
    PointCloud::Ptr _cloud_xyz (new PointCloud);
    pcl::copyPointCloud(*_cloud_label, *_cloud_xyz);

    return _cloud_xyz;
  }

    /**
   * @brief Convierte una nube de entrada con intensidad a solo coordenadas XYZ
  */
  PointCloud::Ptr
  parseToXYZ(PointCloudLN::Ptr &_cloud_label)
  {
    PointCloud::Ptr _cloud_xyz (new PointCloud);
    pcl::copyPointCloud(*_cloud_label, *_cloud_xyz);

    return _cloud_xyz;
  }

  /**
   * @brief Realiza agrupaciones de puntos en función de sus normales
   * 
   * @param cloud  Nube de entrada
   * @return std::vector<pcl::PointIndices> Vector con los indices pertenecientes 
   * a cada agrupación 
   */
  std::pair<vector<pcl::PointIndices>, int>
  regrow_segmentation (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, bool _visualize=false)
  {
    cout << "Regrow segmentation a set of indices fomr a cloud" << endl;

    auto start = std::chrono::high_resolution_clock::now();
    // Estimación de normales
    pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_cloud_in);
    ne.setInputCloud(_cloud_in);
    // ne.setIndices(_indices);   // Tiene que estar comentado para que la dimension de _cloud_normals sea igual a _cloud_in y funcione regrow
    ne.setSearchMethod(tree);
    ne.setKSearch(30);            // Por vecinos no existen normales NaN
    // ne.setRadiusSearch(0.05);  // Por radio existiran puntos cuya normal sea NaN
    ne.compute(*_cloud_normals);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // Segmentación basada en crecimiento de regiones
    vector<pcl::PointIndices> _regrow_clusters;
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize (50); //50 original
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(false);
    reg.setCurvatureTestFlag(true);
    reg.setResidualThreshold(false);
    reg.setCurvatureThreshold(1);
    reg.setNumberOfNeighbours (10); //10 original
    reg.setInputCloud (_cloud_in);
    reg.setIndices(_indices);
    reg.setInputNormals (_cloud_normals);
    reg.setSmoothnessThreshold ( pcl::deg2rad(10.0)); //10 original
    reg.extract (_regrow_clusters);

    // RESULTS VISUALIZATION 
    if(_visualize)
    {
      pcl::visualization::PCLVisualizer vis ("Regrow Visualizer");

      int v1(0);
      int v2(0);

      vis.setBackgroundColor(1,1,1);
      try
      {
        vis.loadCameraParameters("camera_params_regrow_clusters.txt");
        /* code */
      }
      catch(const std::exception& e)
      {
      }
      

      //Define ViewPorts
      vis.createViewPort(0,0,0.5,1, v1);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> green_color(_cloud_in, 10, 150, 10);
      vis.addPointCloud<PointT>(_cloud_in, green_color, "cloud", v1);
      vis.addPointCloudNormals<PointT, pcl::Normal>(_cloud_in, _cloud_normals, 5, 0.1, "normals", v1);


      vis.createViewPort(0.5,0,1,1, v2);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      color_cloud = reg.getColoredCloud();

      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(color_cloud);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      inliers->indices = *_indices;
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*color_cloud);

      vis.addPointCloud<pcl::PointXYZRGB>(color_cloud, "Regrow Segments",v2);


      while (!vis.wasStopped())
      {
        vis.saveCameraParameters("camera_params_regrow_clusters.txt");
        vis.spinOnce();
      }
    }
    return std::pair<vector<pcl::PointIndices>, int> {_regrow_clusters, duration.count()};
  }

  /**
   * @brief Realiza agrupaciones de puntos en función de sus normales
   * 
   * @param cloud  Nube de entrada
   * @return std::vector<pcl::PointIndices> Vector con los indices pertenecientes 
   * a cada agrupación 
   */
  std::pair<vector<pcl::PointIndices>, int>
  regrow_segmentation (PointCloud::Ptr &_cloud_in, bool _visualize = false)
  {
    cout << "Regrow segmentation complete cloud" << endl;
    auto start = std::chrono::high_resolution_clock::now();
    // Estimación de normales
    pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_cloud_in);
    ne.setInputCloud(_cloud_in);
    // ne.setIndices(_indices);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);            // Por vecinos no existen normales NaN
    // ne.setRadiusSearch(0.05);  // Por radio existiran puntos cuya normal sea NaN
    ne.compute(*_cloud_normals);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // Segmentación basada en crecimiento de regiones
    vector<pcl::PointIndices> _regrow_clusters;
    pcl::RegionGrowing<PointT, pcl::Normal> reg;
    reg.setMinClusterSize (50); //50 original
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(false);
    reg.setCurvatureTestFlag(true);
    reg.setResidualThreshold(false);
    reg.setCurvatureThreshold(1);
    reg.setNumberOfNeighbours (10); //10 original
    reg.setInputCloud (_cloud_in);
    reg.setInputNormals (_cloud_normals);
    reg.setSmoothnessThreshold (pcl::deg2rad(10.0)); //10 original
    reg.extract (_regrow_clusters);

    // cout << "Number of clusters: " << _regrow_clusters.size() << endl;

    if(_visualize)
    {
      pcl::visualization::PCLVisualizer vis ("Regrow Visualizer");

      int v1(0);
      int v2(0);

      //Define ViewPorts
      vis.createViewPort(0,0,0.5,1, v1);
      vis.createViewPort(0.5,0,1,1, v2);

      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn (new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields (*_cloud_in, *_cloud_normals, *cloud_pn);

      pcl::visualization::PointCloudColorHandler<pcl::PointNormal>::Ptr color_handler;
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> (cloud_pn, "curvature"));
      vis.addPointCloud<pcl::PointNormal>(cloud_pn, *color_handler, "asdf", v1);
      vis.addPointCloudNormals<PointT, pcl::Normal>(_cloud_in, _cloud_normals, 3, 0.1, "normals", v1);


      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      color_cloud = reg.getColoredCloud();
      vis.addPointCloud<pcl::PointXYZRGB>(color_cloud, "Regrow Segments",v2);

      while (!vis.wasStopped())
        vis.spinOnce();
    }

    return std::pair<vector<pcl::PointIndices>, int> {_regrow_clusters, duration.count()};
  }

  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud Nube de entrada
   * @param indices_vec Vector con los índices de los puntos que se quieren extraer
   * @param negative Si es true, se extraen los puntos que no están en indx_vec
   */
  PointCloud::Ptr
  extract_indices(PointCloud::Ptr &cloud, std::vector<pcl::PointIndices> indices_vec, bool negative = false)
  {
    pcl::PointIndices::Ptr indices (new pcl::PointIndices);
    PointCloud::Ptr _cloud_out (new PointCloud);

    for (size_t i = 0; i < indices_vec.size(); i++)
      for(auto index : indices_vec[i].indices)
        indices->indices.push_back(index);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indices);
    extract.setNegative(negative);
    extract.filter(*_cloud_out);

    return _cloud_out;
  }



  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud Nube de entrada
   * @param indices Indices de los puntos que se quieren extraer
   * @param negative Si es true, se extraen los puntos que no están en indx_vec
   */
  PointCloud::Ptr
  extract_indices (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, bool negative = false)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_cloud_in);
    extract.setIndices(_indices);
    extract.setNegative(negative);
    extract.filter(*_cloud_out);

    return _cloud_out;
  }

  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param optimizeCoefs 
   * @param distThreshold 
   * @param maxIterations 
   * @return pcl::ModelCoefficients::Ptr 
   */
  pcl::ModelCoefficientsPtr 
  compute_planar_ransac (PointCloud::Ptr &_cloud_in, const bool optimizeCoefs,
              float distThreshold = 0.03, int maxIterations = 1000)
  {
    pcl::PointIndices point_indices;
    pcl::SACSegmentation<PointT> ransac;
    pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

    ransac.setInputCloud(_cloud_in);
    ransac.setOptimizeCoefficients(optimizeCoefs);
    ransac.setModelType(pcl::SACMODEL_PLANE);
    ransac.setMethodType(pcl::SAC_RANSAC);
    ransac.setMaxIterations(maxIterations);
    ransac.setDistanceThreshold(distThreshold);
    ransac.segment(point_indices, *plane_coeffs);

    return plane_coeffs;
  }

  pcl::ModelCoefficientsPtr 
  compute_planar_ransac (PointCloud::Ptr &_cloud_in, const pcl::PointIndicesPtr& _indices, const bool optimizeCoefs=true,
              float distThreshold = 0.02, int maxIterations = 1000)
  {
    pcl::PointIndices point_indices;
    pcl::SACSegmentation<PointT> ransac;
    pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

    ransac.setInputCloud(_cloud_in);
    ransac.setIndices(_indices);
    ransac.setOptimizeCoefficients(optimizeCoefs);
    ransac.setModelType(pcl::SACMODEL_PLANE);
    ransac.setMethodType(pcl::SAC_RANSAC);
    ransac.setMaxIterations(maxIterations);
    ransac.setDistanceThreshold(distThreshold);
    ransac.segment(point_indices, *plane_coeffs);

    return plane_coeffs;
  }

  pcl::ModelCoefficientsPtr 
  compute_planar_ransac_direction (PointCloud::Ptr &_cloud_in, const pcl::PointIndicesPtr& _indices, const bool optimizeCoefs=true,
              float distThreshold = 0.02, int maxIterations = 1000, Eigen::Vector3f _direction = Eigen::Vector3f(0,0,1))
  {
    pcl::PointIndices point_indices;
    pcl::SACSegmentation<PointT> ransac;
    pcl::ModelCoefficientsPtr plane_coeffs (new pcl::ModelCoefficients);

    ransac.setInputCloud(_cloud_in);
    ransac.setIndices(_indices);
    ransac.setOptimizeCoefficients(optimizeCoefs);
    ransac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    ransac.setMethodType(pcl::SAC_RANSAC);
    ransac.setMaxIterations(maxIterations);
    ransac.setDistanceThreshold(distThreshold);
    ransac.setAxis(_direction);
    ransac.setEpsAngle (pcl::deg2rad (5.0));
    ransac.segment(point_indices, *plane_coeffs);

    return plane_coeffs;
  }



  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param coefs 
   * @param distThreshold 
   * @return pcl::PointIndices::Ptr 
   */
  pair<pcl::IndicesPtr, pcl::IndicesPtr>
  get_points_near_plane(PointCloud::Ptr &_cloud_in, pcl::ModelCoefficientsPtr &_plane_coeffs, float distThreshold = 0.5f)
  {
    Eigen::Vector4f coefficients(_plane_coeffs->values.data());
    pcl::PointXYZ point;
    pcl::IndicesPtr _plane_inliers (new pcl::Indices);
    pcl::IndicesPtr _plane_outliers (new pcl::Indices);

    for (size_t indx = 0; indx < _cloud_in->points.size(); indx++)
    {
      point = _cloud_in->points[indx];
      float distance = pcl::pointToPlaneDistance(point, coefficients);
      if (pcl::pointToPlaneDistance(point, coefficients) <= distThreshold)
        _plane_inliers->push_back(indx);
      else
        _plane_outliers->push_back(indx);
    }

    return pair<pcl::IndicesPtr, pcl::IndicesPtr> {_plane_inliers, _plane_outliers};
  }


  /**
   * @brief Filtra la nube de puntos en función de los índices pasados como parámetro
   * 
   * @param cloud 
   * @param coefs 
   * @param distThreshold 
   * @return pcl::PointIndices::Ptr 
   */
  pair<pcl::IndicesPtr, pcl::IndicesPtr>
  get_points_near_plane(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, pcl::ModelCoefficientsPtr &_plane_coeffs, float distThreshold = 0.5f)
  {
    Eigen::Vector4f coefficients(_plane_coeffs->values.data());
    pcl::PointXYZ point;
    pcl::IndicesPtr _plane_inliers (new pcl::Indices);
    pcl::IndicesPtr _plane_outliers (new pcl::Indices);
  
  
    // PointCloud::Ptr _cloud_out (new PointCloud);
    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(_cloud_in);
    // extract.setIndices(_indices);
    // extract.setNegative(negative);
    // extract.filter(*_cloud_out);

    for (int indx : *_indices)
    {
      point = _cloud_in->points[indx];
      float distance = pcl::pointToPlaneDistance(point, coefficients);
      if (pcl::pointToPlaneDistance(point, coefficients) <= distThreshold)
        _plane_inliers->push_back(indx);
      else
        _plane_outliers->push_back(indx);
    }

    return pair<pcl::IndicesPtr, pcl::IndicesPtr> {_plane_inliers, _plane_outliers};
  }

  /**
   * @brief Computes the eigenvalues of a PointCloud
   * 
   * @param cloud_in 
   * @return Eigen::Vector3f 
   */
  Eigen::Vector3f
  compute_eigenvalues(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, bool normalize = true)
  {
    Eigen::Vector4f xyz_centroid;
    PointCloud::Ptr tmp_cloud (new PointCloud);
    tmp_cloud = arvc::extract_indices(_cloud_in, _indices);
    pcl::compute3DCentroid(*tmp_cloud, xyz_centroid);

    Eigen::Matrix3f covariance_matrix;
    if (normalize)
      pcl::computeCovarianceMatrixNormalized (*tmp_cloud, xyz_centroid, covariance_matrix); 
    else
      pcl::computeCovarianceMatrix (*tmp_cloud, xyz_centroid, covariance_matrix); 

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

    return eigenValuesPCA;
  }

  struct eig_decomp
  {
    Eigen::Vector3f values;
    Eigen::Matrix3f vectors;
  };

  eig_decomp
  compute_eigen_decomposition(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, bool normalize = true)
  {
    Eigen::Vector4f xyz_centroid;
    PointCloud::Ptr tmp_cloud (new PointCloud);
    tmp_cloud = arvc::extract_indices(_cloud_in, _indices);
    pcl::compute3DCentroid(*tmp_cloud, xyz_centroid);

    Eigen::Matrix3f covariance_matrix;
    if (normalize)
      pcl::computeCovarianceMatrixNormalized (*tmp_cloud, xyz_centroid, covariance_matrix); 
    else
      pcl::computeCovarianceMatrix (*tmp_cloud, xyz_centroid, covariance_matrix); 

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);

    eig_decomp eigen_decomp;
    eigen_decomp.values = eigen_solver.eigenvalues();
    eigen_decomp.vectors = eigen_solver.eigenvectors();

    return eigen_decomp;
  }

  /**
   * @brief Filters each cluster by its eigen values
   * 
   * @return Eigen::Vector3f 
   */
  vector<int>
  validate_clusters_by_ratio(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _ratio_threshold)
  {
    // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      pcl::IndicesPtr current_cluster (new pcl::Indices);
      *current_cluster = cluster.indices;
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, true);
      float size_ratio = eig_values(1)/eig_values(2);

      if (size_ratio < _ratio_threshold){
        valid_clusters.push_back(clust_indx);
        // cout << "\tValid cluster: " << GREEN << clust_indx << RESET << " with ratio: " << size_ratio << endl;
      }
        
      clust_indx++;
    }

    return valid_clusters;
  }

  /**
   * @brief Filters each cluster by its eigen values
   * 
   * @return Eigen::Vector3f 
   */
  vector<int>
  validate_clusters_by_module(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _module_threshold)
  {
    // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();
    PointCloud::Ptr tmp_cloud (new PointCloud);
    PointCloud::Ptr current_cluster_cloud (new PointCloud);
    pcl::IndicesPtr current_cluster (new pcl::Indices);

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      *current_cluster = cluster.indices;

      current_cluster_cloud = arvc::extract_indices(_cloud_in, current_cluster);
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, false);

      // // FOR VISUALIZATION
      // remain_input_cloud = arvc::extract_indices(_cloud_in, current_cluster, true); //FOR VISUALIZTION
      // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // viewer->removeAllPointClouds();
      // viewer->addPointCloud<PointT> (remain_input_cloud, "original_cloud");
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(current_cluster_cloud, 0, 255, 0);
      // viewer->addPointCloud<PointT> (current_cluster_cloud, single_color, "current_cluster_cloud");

      // while (!viewer->wasStopped ())
      //   viewer->spinOnce(100);
      
      float max_length = get_max_length(current_cluster_cloud);

      if (max_length < _module_threshold){
        valid_clusters.push_back(clust_indx);
        // cout << "\tValid cluster: " << GREEN << clust_indx << RESET << " with  modules: " << eig_values(1) << ", " << eig_values(2) << endl;
      }
        
      clust_indx++;
    }

    // if (valid_clusters.size() == 0)
    //   cout << "\tNo valid clusters found" << endl;

    return valid_clusters;
  }

    /**
   * @brief Filters each cluster by its eigen values
   * 
   * @return Eigen::Vector3f 
   */
  vector<int>
  validate_clusters_hybrid(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _ratio_threshold, float _module_threshold)
  {
    // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();
    PointCloud::Ptr current_cluster_cloud (new PointCloud);
    PointCloud::Ptr remain_input_cloud (new PointCloud);
    pcl::IndicesPtr current_cluster (new pcl::Indices);

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      *current_cluster = cluster.indices;

      current_cluster_cloud = arvc::extract_indices(_cloud_in, current_cluster);
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, false);

      // // FOR VISUALIZATION
      // remain_input_cloud = arvc::extract_indices(_cloud_in, current_cluster, true); //FOR VISUALIZTION
      // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // viewer->removeAllPointClouds();
      // viewer->addPointCloud<PointT> (remain_input_cloud, "original_cloud");
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(current_cluster_cloud, 0, 255, 0);
      // viewer->addPointCloud<PointT> (current_cluster_cloud, single_color, "current_cluster_cloud");

      // while (!viewer->wasStopped ())
      //   viewer->spinOnce(100);
      

      float size_ratio = eig_values(1)/eig_values(2);
      float max_length = get_max_length(current_cluster_cloud);
      // cout << "Eig values: " << eig_values(1) << ", " << eig_values(2) << endl;
      // cout << "Ratio: " << size_ratio << endl;

      if (max_length < _module_threshold)
      {
        // cout << "\tValid cluster: " << GREEN << clust_indx << RESET << " With Module: " << max_length << endl;
        if (size_ratio < _ratio_threshold){
          valid_clusters.push_back(clust_indx);
          // cout << "\tValid cluster: " << GREEN << _indx << RESET << " with ratio: " << size_ratio << endl;
        }
      // else
        // cout << "\tInvalid cluster: " << RED << clust_indx << RESET << " With Module: " << max_length << endl;

      // getchar();
      }
      
      clust_indx++;

    // if (valid_clusters.size() == 0)
    //   cout << "\tNo valid clusters found" << endl;

    }
    return valid_clusters;
  }


  pair<vector<int>, bool>
  validate_clusters_hybrid_tmp(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _ratio_threshold, float _module_threshold)
  {
    // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();
    PointCloud::Ptr current_cluster_cloud (new PointCloud);
    PointCloud::Ptr remain_input_cloud (new PointCloud);
    pcl::IndicesPtr current_cluster (new pcl::Indices);
    bool raise_flag = false;

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      *current_cluster = cluster.indices;

      current_cluster_cloud = arvc::extract_indices(_cloud_in, current_cluster);
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, false);

      // // FOR VISUALIZATION
      // remain_input_cloud = arvc::extract_indices(_cloud_in, current_cluster, true); //FOR VISUALIZTION
      // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // viewer->removeAllPointClouds();
      // viewer->addPointCloud<PointT> (remain_input_cloud, "original_cloud");
      // pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(current_cluster_cloud, 0, 255, 0);
      // viewer->addPointCloud<PointT> (current_cluster_cloud, single_color, "current_cluster_cloud");

      // while (!viewer->wasStopped ())
      //   viewer->spinOnce(100);
      

      float size_ratio = eig_values(1)/eig_values(2);
      float max_length = get_max_length(current_cluster_cloud);
      // cout << "Eig values: " << eig_values(1) << ", " << eig_values(2) << endl;
      // cout << "Ratio: " << size_ratio << endl;

      if (max_length < _module_threshold)
      {
        // cout << "\tValid cluster: " << GREEN << clust_indx << RESET << " With Module: " << max_length << endl;
        if (size_ratio < _ratio_threshold){
          valid_clusters.push_back(clust_indx);
          // cout << "\tValid cluster: " << GREEN << _indx << RESET << " with ratio: " << size_ratio << endl;
        }
        else
          raise_flag = true;
      }
      // else
        // cout << "\tInvalid cluster: " << RED << clust_indx << RESET << " With Module: " << max_length << endl;

      clust_indx++;
      // getchar();
    }

    // if (valid_clusters.size() == 0)
    //   cout << "\tNo valid clusters found" << endl;

    return pair<vector<int>, bool> (valid_clusters, raise_flag);
  }


  vector<int>
  validate_clusters_hybrid_old(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _ratio_threshold, float _module_threshold)
  {
    // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();
    PointCloud::Ptr current_cluster_cloud (new PointCloud);
    PointCloud::Ptr remain_input_cloud (new PointCloud);
    pcl::IndicesPtr current_cluster (new pcl::Indices);

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      *current_cluster = cluster.indices;

      current_cluster_cloud = arvc::extract_indices(_cloud_in, current_cluster);
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, false);

      float size_ratio = eig_values(1)/eig_values(2);

      if (eig_values(1) < _module_threshold && eig_values(2) < _module_threshold){
        if (size_ratio < _ratio_threshold){
          valid_clusters.push_back(clust_indx);
        }
      }

      clust_indx++;
    }

    // if (valid_clusters.size() == 0)
    //   cout << "\tNo valid clusters found" << endl;

    return valid_clusters;
  }

  pcl::IndicesPtr
  ownInverseIndices(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices_in)
  {
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    _indices_out->resize(_cloud_in->size());
    iota(_indices_out->begin(), _indices_out->end(), 0);

    for (int indx : *_indices_in)
      _indices_out->erase(remove(_indices_out->begin(), _indices_out->end(), indx), _indices_out->end());

    // set_difference(_original_indices->begin(), _original_indices->end(), _indices_in->begin(), _indices_in->end(), inserter(*_indices_out, _indices_out->begin()));

    return _indices_out;
  }


  pcl::IndicesPtr
  inverseIndices(PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices_in)
  {
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(_cloud_in);
    extract.setIndices(_indices_in);
    extract.setNegative(true);
    extract.filter(*_indices_out);

    return _indices_out;
  }

  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param _cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  PointCloud::Ptr
  voxel_filter( PointCloud::Ptr &_cloud_in ,float leafSize = 0.1)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(_cloud_in);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*_cloud_out);

    return _cloud_out;
  }


    /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param _cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  PointCloud::Ptr
  voxel_filter( PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices, float leafSize = 0.1)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(_cloud_in);
    sor.setIndices(_indices);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*_cloud_out);

    return _cloud_out;
  }

  /**
   * @brief Returns the intersection of two vectors
   * 
   * @param v1 
   * @param v2 
   * @return vector<int> 
   */
  vector<int> 
  intersection(vector<int> v1,
               vector<int> v2){
    vector<int> v3;

    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());

    std::set_intersection(v1.begin(),v1.end(),
                          v2.begin(),v2.end(),
                          back_inserter(v3));
    return v3;
  }

  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  metrics
  compute_metrics(conf_matrix cm)
  {
    metrics _metrics;

    _metrics.precision = (float)cm.TP / ((float)cm.TP + (float)cm.FP);
    _metrics.recall = (float)cm.TP / ((float)cm.TP + (float)cm.FN);
    _metrics.f1_score = (2 * _metrics.precision * _metrics.recall) / (_metrics.precision + _metrics.recall);
    _metrics.accuracy = (float)(cm.TP + cm.TN) / (float)(cm.TP + cm.TN + cm.FP + cm.FN);

    return _metrics;
  }

  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
  metrics
  compute_metrics(cm_indices _cm_indices)
  {
    metrics _metrics;
    float TP = (float) _cm_indices.tp_idx->size();
    float FP = (float) _cm_indices.fp_idx->size();
    float TN = (float) _cm_indices.tn_idx->size();
    float FN = (float) _cm_indices.fn_idx->size();

    _metrics.precision = TP / (TP + FP);
    _metrics.recall = TP / (TP + FN);
    _metrics.f1_score = 2 * (_metrics.precision * _metrics.recall) / (_metrics.precision + _metrics.recall);
    _metrics.accuracy = (TP + TN) / (TP + TN + FP + FN);

    return _metrics;
  }


  /**
   * @brief Computes the confusion matrix from the ground truth and the detected indices
   * 
   * @param gt_truss_idx 
   * @param gt_ground_idx 
   * @param truss_idx 
   * @param ground_idx 
   * @return conf_matrix 
   */
  conf_matrix
  compute_conf_matrix(pcl::IndicesPtr &gt_truss_idx, pcl::IndicesPtr &gt_ground_idx,  pcl::IndicesPtr &truss_idx, pcl::IndicesPtr &ground_idx)
  {
    conf_matrix _conf_matrix;

    _conf_matrix.TP = arvc::intersection(*gt_truss_idx, *truss_idx).size();
    _conf_matrix.TN = arvc::intersection(*gt_ground_idx, *ground_idx).size();
    _conf_matrix.FP = gt_ground_idx->size() - _conf_matrix.TN;
    _conf_matrix.FN = gt_truss_idx->size() - _conf_matrix.TP;
    
    return _conf_matrix;
  }


  void 
  writeCloud (pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_in, fs::path entry)
  {
    pcl::PCDWriter pcd_writer;
    
    fs::path abs_file_path = fs::current_path().parent_path() / "pcd_xyzlabelnormal";
    if (!fs::exists(abs_file_path)) 
      fs::create_directory(abs_file_path);

    std::string filename = entry.stem().string() + ".pcd";

    abs_file_path = abs_file_path / filename;
    pcd_writer.write(abs_file_path.string(), *cloud_in, true);
  }


  /**
   * Visualize current working cloud
  */
  void 
  visualizeCloud ( PointCloud::Ptr &_cloud_in)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    vis.addPointCloud<PointT> (_cloud_in, "Original", v1);

    while(!vis.wasStopped())
      vis.spinOnce(100);

  }

  /**
   * Visualize current working cloud
  */
  void 
  visualizeCloud ( PointCloud::Ptr &_cloud_in, int r, int g, int b)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    vis.setBackgroundColor(1,1,1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color(_cloud_in, r, g, b);
    vis.addPointCloud<PointT> (_cloud_in, cloud_color, "cloud", v1);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
  }


  void 
  visualizeClouds (PointCloud::Ptr &original_cloud, PointCloud::Ptr &filtered_cloud)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    int v2(0);

    //Define ViewPorts
    vis.createViewPort(0,0,0.5,1, v1);
    vis.createViewPort(0.5,0,1,1, v2);

    vis.removeAllPointClouds();

    vis.addCoordinateSystem(0.50, "global", v1);
    vis.addCoordinateSystem(0.50, "global2", v2);

    Eigen::Vector3f position(0.0, 0.0, 0.0);

    vis.addCube(position, original_cloud->sensor_orientation_, 0.085, 0.085, 0.073, "cube", v1);
    vis.addPointCloud<PointT> (original_cloud, "Original", v1);
    vis.addPointCloud<PointT> (filtered_cloud, "Filtered", v2);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
  }

  void 
  visualizeClouds (PointCloud::Ptr &original_cloud, int r1, int g1, int b1, PointCloud::Ptr &filtered_cloud, int r2, int g2, int b2)
  {
    pcl::visualization::PCLVisualizer vis("PCL_Visualizer");

    int v1(0);
    int v2(0);

    //Define ViewPorts
    vis.createViewPort(0,0,0.5,1, v1);
    vis.createViewPort(0.5,0,1,1, v2);

    vis.removeAllPointClouds();
    vis.setBackgroundColor(1,1,1);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color1(original_cloud, r1, g1, b1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color2(filtered_cloud, r2, g2, b2);

    vis.addPointCloud<PointT> (original_cloud, cloud_color1, "Original", v1);
    vis.addPointCloud<PointT> (filtered_cloud, cloud_color2, "Filtered", v2);

    while(!vis.wasStopped())
      vis.spinOnce(100);

    vis.close();
  }


  /**
   * @brief Remove points with less than minNeighbors inside a give radius
   * 
   * @param radius Search sphere radius
   * @param minNeighbors Minimum num of Neighbors to consider a point an inlier
   * @return PointCloud::Ptr Return a PointCloud without low neighbor points
   */
  pcl::IndicesPtr
  radius_outlier_removal (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices_in, float radius, int minNeighbors, bool negative = false)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    pcl::RadiusOutlierRemoval<PointT> radius_removal;
    radius_removal.setInputCloud(_cloud_in);
    radius_removal.setIndices(_indices_in);
    radius_removal.setRadiusSearch(radius);
    radius_removal.setMinNeighborsInRadius(minNeighbors);
    radius_removal.setNegative(negative);
    radius_removal.filter(*_indices_out);

    return _indices_out;
  }

  pcl::IndicesPtr
  radius_outlier_removal (PointCloud::Ptr &_cloud_in, float radius, int minNeighbors, bool negative = false)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::IndicesPtr _indices_out (new pcl::Indices);
    pcl::RadiusOutlierRemoval<PointT> radius_removal;
    radius_removal.setInputCloud(_cloud_in);
    radius_removal.setRadiusSearch(radius);
    radius_removal.setMinNeighborsInRadius(minNeighbors);
    radius_removal.setNegative(negative);
    radius_removal.filter(*_indices_out);

    return _indices_out;
  }


  float
  mean(vector<float> v)
  {
    float sum = std::accumulate(v.begin(), v.end(), 0.0);
    float mean = sum / v.size();
    
    return mean;
  }

  int
  mean(vector<int> v)
  {
    int sum = std::accumulate(v.begin(), v.end(), 0.0);
    float mean = sum / v.size();
    int value = round(mean);
    
    return value;
  }


  /**
   * @brief Returns the indices of the points that are inside the given radius
   * 
   * @param _gt_truss_idx 
   * @param _gt_ground_idx 
   * @param _truss_idx 
   * @param _ground_idx
   * @return cm_indices 
   */
  cm_indices
  compute_cm_indices(pcl::IndicesPtr &_gt_truss_idx, pcl::IndicesPtr &_gt_ground_idx, pcl::IndicesPtr &_truss_idx, pcl::IndicesPtr &_ground_idx)
  {
    cm_indices _cm_indices;
    _cm_indices.tp_idx.reset(new pcl::Indices);
    _cm_indices.tn_idx.reset(new pcl::Indices);
    _cm_indices.fp_idx.reset(new pcl::Indices);
    _cm_indices.fn_idx.reset(new pcl::Indices);
    
    for (size_t i = 0; i < _truss_idx->size(); i++)
    {
      if(std::find(_gt_truss_idx->begin(), _gt_truss_idx->end(), _truss_idx->at(i)) != _gt_truss_idx->end())
        _cm_indices.tp_idx->push_back(_truss_idx->at(i));
      else
        _cm_indices.fp_idx->push_back(_truss_idx->at(i));
    }

    for (size_t i = 0; i < _ground_idx->size(); i++)
    {
      if(std::find(_gt_ground_idx->begin(), _gt_ground_idx->end(),_ground_idx->at(i)) !=_gt_ground_idx->end())
        _cm_indices.tn_idx->push_back(_ground_idx->at(i));
      else
        _cm_indices.fn_idx->push_back(_ground_idx->at(i));
    }
    
    return _cm_indices;
  }  

  pcl::PointCloud<pcl::Normal>::Ptr
  compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud_in, pcl::IndicesPtr &_indices)
  {
    auto start = std::chrono::high_resolution_clock::now();
    // Estimación de normales
    pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_cloud_in);
    ne.setInputCloud(_cloud_in);
    ne.setIndices(_indices);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);            // Por vecinos no existen normales NaN
    // ne.setRadiusSearch(0.05);  // Por radio existiran puntos cuya normal sea NaN
    ne.compute(*_cloud_normals);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Ful Normals Computation Time: " << duration.count() << " ms" << std::endl;
    return _cloud_normals;
  }

  pcl::PointCloud<pcl::Normal>::Ptr
  compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud_in)
  {
    auto start = std::chrono::high_resolution_clock::now();
    // Estimación de normales
    pcl::PointCloud<pcl::Normal>::Ptr _cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(_cloud_in);
    ne.setInputCloud(_cloud_in);
    ne.setSearchMethod(tree);
    ne.setKSearch(30);            // Por vecinos no existen normales NaN
    // ne.setRadiusSearch(0.05);  // Por radio existiran puntos cuya normal sea NaN
    ne.compute(*_cloud_normals);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Full Normals Computation Time: " << duration.count() << " ms" << std::endl;
    return _cloud_normals;
  }

  PointCloud::Ptr
  scale_cloud(PointCloud::Ptr &_cloud_in, float _scaling_factor)
  {
    PointCloud::Ptr scaledCloud(new PointCloud);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.scale(_scaling_factor);
    pcl::transformPointCloud(*_cloud_in, *scaledCloud, transform);

    return scaledCloud;
  }

  PointCloud::Ptr
  remove_origin_points(PointCloud::Ptr &_cloud_in)
  {
    PointCloud::Ptr _cloud_out (new PointCloud);
    pcl::PointIndicesPtr indices(new pcl::PointIndices);
    Eigen::Vector4f min_pt( -0.01, -0.01, -0.01, 1.0);
    Eigen::Vector4f max_pt( 0.01, 0.01, 0.01, 1.0);

    pcl::getPointsInBox(*_cloud_in, min_pt, max_pt, indices->indices);
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    extract.setInputCloud(_cloud_in);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*_cloud_out);

    return _cloud_out;
  }
  
  void
  remove_indices_from_cloud(PointCloud::Ptr &_cloud_in, pcl::PointIndicesPtr &_indices, bool _negative = true)
  {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    extract.setInputCloud(_cloud_in);
    extract.setIndices(_indices);
    extract.setNegative(_negative);
    extract.filter(*_cloud_in);
  }

  pcl::RGB
  get_random_color(int max = 255, int min = 0)
  {
    pcl::RGB color;
    color.r = (int) min + (rand() % max);
    color.g = (int) min + (rand() % max);
    color.b = (int) min + (rand() % max);

    return color;
  }

  vector<pcl::PointIndices> 
  euclideanClustering(const PointCloud::Ptr& cloud_in){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    tree->setInputCloud (cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract(cluster_indices);

    return cluster_indices;
  }

void print_vector(vector<int> _vector)
{
  cout << "[ ";
  for (auto &i : _vector)
  {
    std::cout << i << ' ';
  }
  std::cout << " ]" << std::endl;
}

void print_vector(vector<float> _vector)
{
  cout << "[ ";
  for (auto &i : _vector)
  {
    std::cout << i << ' ';
  }
  std::cout << " ]" << std::endl;
}


  float plane_to_plane_distance(const pcl::ModelCoefficients &_plane1, const pcl::ModelCoefficients &_plane2, const Eigen::Vector3f _normal)
  {
    float distance = 0.0;
    distance = abs(_plane1.values[3] - _plane2.values[3]) / _normal.norm();

    return distance;
  }


  vector<int> get_duplicates(vector<int> _vector)
  {

    vector<int>::iterator it;
    vector<int> duplicates;

    sort(_vector.begin(), _vector.end());
    it = adjacent_find(_vector.begin(), _vector.end());

    // GET DUPLICATES AS A VECTOR
    while (it != _vector.end())
    {
      duplicates.push_back(*it);
      it = adjacent_find(++it, _vector.end());
    } 

    // REMOVE DUPLICATES INSIDE THE DUPLICATE VALUES VECTOR
    it = unique(duplicates.begin(), duplicates.end());
    duplicates.resize(distance(duplicates.begin(), it));
    
    return duplicates;
  }


  pcl::PointIndices::Ptr get_unique(vector<int> _indices){
    
    pcl::PointIndices::Ptr _unique_indices (new pcl::PointIndices);

    sort(_indices.begin(), _indices.end());
    auto it = unique(_indices.begin(), _indices.end());
    _indices.resize(distance(_indices.begin(), it));

    _unique_indices->indices = _indices;

    return _unique_indices;
  }


  pcl::Indices get_cloud_indices(const PointCloud::Ptr &_cloud_in){
    pcl::Indices indices;
    indices.resize(_cloud_in->size());
    iota(indices.begin(), indices.end(), 0);
    
    return indices;
  }


  PointCloud::Ptr random_sample(const PointCloud::Ptr& _cloud_in, const float& _sample_ratio){
    
    pcl::RandomSample<PointT> rs;
    PointCloud::Ptr _cloud_out (new PointCloud);

    rs.setInputCloud(_cloud_in);
    rs.setSample(_cloud_in->size() * _sample_ratio);
    rs.setSeed(rand());
    rs.filter(*_cloud_out);

    return _cloud_out;
  }


  PointCloud::Ptr uniform_sample(const PointCloud::Ptr& _cloud_in, const float& _grid_size){
    
    pcl::UniformSampling<PointT> us;
    PointCloud::Ptr _cloud_out (new PointCloud);

    us.setInputCloud(_cloud_in);
    us.setRadiusSearch(0.01);
    us.filter(*_cloud_out);

    return _cloud_out;
  }





  // vector<int> cat_vectors(vector<vector> _vectors){
  //   vector<int> _cat_vector;

  //   for (auto _vector : _vectors)
  //     _cat_vector.insert(_cat_vector.end(), _vector.begin(), _vector.end());

  //   return _cat_vector;
  // }



#include <arvc_console.hpp>

#include <arvc_axes3d.hpp>
  /* class axes3d
  {
  private:

  public:
    axes3d(){
      this->x = Eigen::Vector3f::Zero();
      this->y = Eigen::Vector3f::Zero();
      this->z = Eigen::Vector3f::Zero();
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
  } */


#include "arvc_color.hpp"
/*   class color
  {
    public:
      color(int min = 0, int max = 255){
        this->random(min, max);
      }

      color(int _r, int _g, int _b){
        this->r = _r;
        this->g = _g;
        this->b = _b;
      }

      friend std::ostream& operator<<(std::ostream& os, const arvc::color& c)
      {
        os << "Color: [ " << c.r << ", " << c.g << ", " << c.b << " ]" << endl;
        return os;
      }

      void random(int min = 0, int max = 255){
        this->r = (int) min + (rand() % max);
        this->g = (int) min + (rand() % max);
        this->b = (int) min + (rand() % max);
      }

      void normalized(){
        this->r = (int) this->r / 255;
        this->g = (int) this->g / 255;
        this->b = (int) this->b / 255;
      }

      static const color RED_COLOR;
      static const color BLUE_COLOR;
      static const color GREEN_COLOR;
      static const color YELLOW_COLOR;
      static const color WHITE_COLOR;
      int r;
      int g;
      int b;
  };
  
  const arvc::color arvc::color::RED_COLOR = arvc::color(255,0,0);
  const arvc::color arvc::color::BLUE_COLOR = arvc::color(0,0,255);
  const arvc::color arvc::color::GREEN_COLOR = arvc::color(0,255,0);
  const arvc::color arvc::color::YELLOW_COLOR = arvc::color(255,255,0);
  const arvc::color arvc::color::WHITE_COLOR = arvc::color(255,255,255); */


  class direction

  {
  public:
    direction(){}

    ~direction(){}    
    Eigen::Vector3f vector;
    arvc::color color;
  };


#include "arvc_plane.hpp"
/*   class plane
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
        this->polygon = vector<Eigen::Vector3f>(4);
      };

      // plane(pcl::ModelCoefficientsPtr _coeffs, pcl::PointIndicesPtr _indices)
      // {
      //   this->coeffs.reset(new pcl::ModelCoefficients);
      //   this->inliers.reset(new pcl::PointIndices);

      //   *this->coeffs = *_coeffs;
      //   *this->inliers = *_indices;
        
      //   cout << "PLANE OBJ INLIERS SIZE: " << this->inliers->indices.size() << endl;
      //   cout << "PLANE OBJ COEFFS: " << *this->coeffs << endl;
      //   cout << "-----------------------------" << endl;
      // }; 

      ~plane(){
        this->coeffs->values = {0,0,0,0};
        this->inliers->indices = {0};
      };

      friend std::ostream& operator<<(std::ostream& os, const arvc::plane& p)
      {
        os << "Plane parameters: [ " << p.coeffs->values[0] << ", " << p.coeffs->values[1] << ", " << p.coeffs->values[2] << ", " << p.coeffs->values[3] << " ]" << endl;
        os << "Plane inliers: " << p.inliers->indices.size() << endl;

        return os;
      }

      void setPlane(pcl::ModelCoefficientsPtr _coeffs, pcl::PointIndicesPtr _indices, PointCloud::Ptr _cloud_in){
        *this->coeffs = *_coeffs;
        *this->inliers = *_indices;
        *this->original_cloud = *_cloud_in;
        this->getNormal();
        this->getCloud();
        this->getEigenVectors();
        this->getEigenValues();
        this->getCentroid();  
        this->color.random();
      };

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

      void getEigenValues(){
        this->eigenvalues = arvc::compute_eigenvalues3D(this->cloud);
        cout << this->eigenvalues << endl;
      }

      void projectOnPlane(){
        pcl::ProjectInliers<PointT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(this->cloud);
        proj.setModelCoefficients(this->coeffs);
        proj.filter(*this->cloud);
      }

      void getCentroid(){
        pcl::compute3DCentroid(*this->cloud, this->centroid);
      }


      void getPolygon(){
        PointT max_point;
        PointT min_point;

        pcl::getMinMax3D(*this->cloud, min_point, max_point);

        this->polygon[0] = Eigen::Vector3f(min_point.x, min_point.y, 0.0);
        this->polygon[1] = Eigen::Vector3f(min_point.x, max_point.y, 0.0);
        this->polygon[2] = Eigen::Vector3f(max_point.x, max_point.y, 0.0);
        this->polygon[3] = Eigen::Vector3f(max_point.x, min_point.y, 0.0);
      }



      pcl::ModelCoefficientsPtr coeffs;
      pcl::PointIndicesPtr inliers;
      Eigen::Vector3f normal;
      Eigen::Vector4f centroid;
      PointCloud::Ptr cloud;
      PointCloud::Ptr original_cloud;
      arvc::color color;
      arvc::axes3d eigenvectors;
      Eigen::Vector3f eigenvalues;
      vector<Eigen::Vector3f> polygon;


  }; */


#include <arvc_viewer.hpp>
  /* class viewer {
    private:
      pcl::visualization::PCLVisualizer::Ptr view;
      int cloud_count;

    public:
      int v1,v2,v3,v4;
      
      viewer(){
        this->view.reset(new pcl::visualization::PCLVisualizer("ARVC_VIEWER"));
        this->cloud_count = 0;
        this->v1 = 0;
        this->v2 = 0;
        this->v3 = 0;
        this->v4 = 0;

      }

      viewer(const string& name){
        this->view.reset(new pcl::visualization::PCLVisualizer(name));
        this->cloud_count = 0;
        this->v1 = 0;
        this->v2 = 0;
        this->v3 = 0;
        this->v4 = 0;

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
      this->view->addCoordinateSystem(0.1, "origin"+to_string(_viewport), _viewport);
      pcl::PointXYZ origin_point(0,0,0);
      this->view->addSphere(origin_point, 0.02, "sphere"+to_string(_viewport), _viewport);
      this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "sphere"+to_string(_viewport), _viewport);
      this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "sphere"+to_string(_viewport), _viewport);
      this->view->addText3D("origin", origin_point, 0.02, 1.0, 1.0, 1.0, "origin_text"+to_string(_viewport), _viewport);
    }


    void addCube(const float& _range){
      this->view->addCube(-_range, _range, -_range, _range, -_range, _range, 1.0, 1.0, 0.0, "cube");
      this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
      this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cube");
    }

    void addCube(const pcl::PointXYZ& _min, const pcl::PointXYZ& _max ){
      this->view->addCube(_min.x, _max.x, _min.y, _max.y, _min.z, _max.z, 1.0, 1.0, 0.0, "cube");
      this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
      this->view->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "cube");
    }

    void addCoordinateSystem(const Eigen::Affine3f& tf, const int& _viewport=0){
      this->view->addCoordinateSystem(0.1, tf, "relative"+to_string(_viewport), _viewport);
      
      pcl::PointXYZ relative_origin(tf.translation().x(), tf.translation().y(), tf.translation().z());

      this->view->addText3D("relative", relative_origin, 0.02, 1.0, 1.0, 1.0, "relative_text"+to_string(_viewport), _viewport);

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

      this->view->addArrow<pcl::PointXYZ, pcl::PointXYZ>(target_x, origin, 1.0, 0.0, 0.0, false, "eigenvector_x", _viewport);
      this->view->addArrow<pcl::PointXYZ, pcl::PointXYZ>(target_y, origin, 0.0, 1.0, 0.0, false, "eigenvector_y", _viewport);
      this->view->addArrow<pcl::PointXYZ, pcl::PointXYZ>(target_z, origin, 0.0, 0.0, 1.0, false, "eigenvector_z", _viewport);

    }

    void addPolygon(const vector<eigen){

    }

    void show(){
      while(!this->view->wasStopped())
        this->view->spinOnce(100);
    }

  }; */
}