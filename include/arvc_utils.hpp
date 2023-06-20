#pragma once

// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <numeric>


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
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Dense>
#include <pcl/io/obj_io.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"  
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

namespace fs = std::filesystem;

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PointXYZL> PointCloudL;

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
   * @brief Realiza agrupaciones de puntos en función de sus normales
   * 
   * @param cloud  Nube de entrada
   * @return std::vector<pcl::PointIndices> Vector con los indices pertenecientes 
   * a cada agrupación 
   */
  std::pair<vector<pcl::PointIndices>, int>
  regrow_segmentation (PointCloud::Ptr &_cloud_in, pcl::IndicesPtr &_indices)
  {
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
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(false);
    reg.setCurvatureTestFlag(true);
    reg.setResidualThreshold(false);
    reg.setCurvatureThreshold(1);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (_cloud_in);
    reg.setIndices(_indices);
    reg.setInputNormals (_cloud_normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.extract (_regrow_clusters);

    // cout << "Number of clusters: " << _regrow_clusters.size() << endl;

    // // Uncomment to visualize cloud
    // pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // color_cloud = reg.getColoredCloud();
    // vis.addPointCloud<pcl::PointXYZRGB>(color_cloud);

    // while (!vis.wasStopped())
    //   vis.spinOnce();

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
  regrow_segmentation (PointCloud::Ptr &_cloud_in)
  {
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
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (25000);
    reg.setSearchMethod (tree);
    reg.setSmoothModeFlag(false);
    reg.setCurvatureTestFlag(true);
    reg.setResidualThreshold(false);
    reg.setCurvatureThreshold(1);
    reg.setNumberOfNeighbours (10);
    reg.setInputCloud (_cloud_in);
    reg.setInputNormals (_cloud_normals);
    reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
    reg.extract (_regrow_clusters);

    // cout << "Number of clusters: " << _regrow_clusters.size() << endl;

    // Uncomment to visualize cloud
    // pcl::visualization::PCLVisualizer vis ("PCL Visualizer");
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // color_cloud = reg.getColoredCloud();
    // vis.addPointCloud<pcl::PointXYZRGB>(color_cloud);

    // while (!vis.wasStopped())
    //   vis.spinOnce();

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

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      pcl::IndicesPtr current_cluster (new pcl::Indices);
      *current_cluster = cluster.indices;
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, false);
      // cout << "Eig values: " << eig_values(1) << ", " << eig_values(2) << endl;

      if (eig_values(1) < _module_threshold && eig_values(2) < _module_threshold){
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

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      pcl::IndicesPtr current_cluster (new pcl::Indices);
      *current_cluster = cluster.indices;
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, false);
      float size_ratio = eig_values(1)/eig_values(2);
      // cout << "Eig values: " << eig_values(1) << ", " << eig_values(2) << endl;
      // cout << "Ratio: " << size_ratio << endl;

      if (eig_values(1) < _module_threshold && eig_values(2) < _module_threshold){
        if (size_ratio < _ratio_threshold){
          valid_clusters.push_back(clust_indx);
          // cout << "\tValid cluster: " << GREEN << clust_indx << RESET << " with ratio: " << size_ratio << endl;
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
  computeMetrics(conf_matrix cm)
  {
    metrics _metrics;

    _metrics.precision = (float)cm.TP / ((float)cm.TP + (float)cm.FP);
    _metrics.recall = (float)cm.TP / ((float)cm.TP + (float)cm.FN);
    _metrics.f1_score = (2 * _metrics.precision * _metrics.recall) / (_metrics.precision + _metrics.recall);
    _metrics.accuracy = (float)(cm.TP + cm.TN) / (float)(cm.TP + cm.TN + cm.FP + cm.FN);

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
  computeConfusionMatrix(pcl::IndicesPtr &gt_truss_idx, pcl::IndicesPtr &gt_ground_idx,  pcl::IndicesPtr &truss_idx, pcl::IndicesPtr &ground_idx)
  {
    conf_matrix _conf_matrix;

    _conf_matrix.TP = arvc::intersection(*gt_truss_idx, *truss_idx).size();
    _conf_matrix.TN = arvc::intersection(*gt_ground_idx, *ground_idx).size();
    _conf_matrix.FP = gt_ground_idx->size() - _conf_matrix.TN;
    _conf_matrix.FN = gt_truss_idx->size() - _conf_matrix.TP;
    
    return _conf_matrix;
  }


  /**
   * @brief Returns a Voxelized PointCloud
   * 
   * @param cloud_in 
   * @return pcl::PointCloud<pcl::PointXYZ>::Ptr
   */
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

    vis.close();
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


  void
  addLinetoViewer()
  {
  //   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //   viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  //   viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  //   viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");
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

}