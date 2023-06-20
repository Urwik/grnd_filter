// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>

#include "arvc_utils.hpp"
#include "tqdm.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

namespace fs = std::filesystem;
using namespace std;

class remove_ground
{
private:
  /* data */
public:
  fs::path path;
  PointCloud::Ptr cloud_in;
  PointCloud::Ptr cloud_out;
  pcl::IndicesPtr truss_idx;
  pcl::IndicesPtr ground_idx;
  pcl::IndicesPtr gt_truss_idx;
  pcl::IndicesPtr gt_ground_idx;
  pcl::IndicesPtr low_density_idx;
  pcl::IndicesPtr wrong_idx;
  pcl::IndicesPtr tp_idx;
  pcl::IndicesPtr fp_idx;
  pcl::IndicesPtr fn_idx;
  pcl::IndicesPtr tn_idx;

  metrics metricas;
  conf_matrix cm;

  int normals_time;
  int metrics_time;

  bool visualize;
  bool compute_metrics;

  remove_ground(fs::path _path)
  {
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);
    this->truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->low_density_idx = pcl::IndicesPtr (new pcl::Indices);
    this->wrong_idx = pcl::IndicesPtr (new pcl::Indices);
    this->tp_idx = pcl::IndicesPtr (new pcl::Indices);
    this->fp_idx = pcl::IndicesPtr (new pcl::Indices);
    this->fn_idx = pcl::IndicesPtr (new pcl::Indices);
    this->tn_idx = pcl::IndicesPtr (new pcl::Indices);

    this->visualize = false;
    this->compute_metrics = false;
    this->normals_time = 0;
    this->metrics_time = 0;
  }

  ~remove_ground()
  {
    this->path.clear();
    this->cloud_in->clear();
    this->cloud_out->clear();
    this->visualize = false;
    this->normals_time = 0;
    this->metrics_time = 0;
  }

  void
  getConfMatrixIndexes()
  {
    for (size_t i = 0; i < this->truss_idx->size(); i++)
    {
      if(std::find(this->gt_truss_idx->begin(), this->gt_truss_idx->end(), this->truss_idx->at(i)) != this->gt_truss_idx->end())
        this->tp_idx->push_back(this->truss_idx->at(i));
      else
        this->fp_idx->push_back(this->truss_idx->at(i));
    }

    for (size_t i = 0; i < this->ground_idx->size(); i++)
    {
      if(std::find(this->gt_ground_idx->begin(), this->gt_ground_idx->end(), this->ground_idx->at(i)) != this->gt_ground_idx->end())
        this->tn_idx->push_back(this->ground_idx->at(i));
      else
        this->fn_idx->push_back(this->ground_idx->at(i));
    }
    
  }  

  int
  run()
  {
    PointCloudI::Ptr cloud_in_intensity (new PointCloudI);
    PointCloud::Ptr cloud_in_xyz (new PointCloud);
    PointCloud::Ptr cloud_out_xyz (new PointCloud);
    PointCloud::Ptr tmp_cloud (new PointCloud);
    PointCloud::Ptr uncluster_cloud (new PointCloud);

    pcl::IndicesPtr coarse_ground_indices (new pcl::Indices);
    pcl::IndicesPtr coarse_truss_indices (new pcl::Indices);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;
    ground_truth_indices.ground = pcl::IndicesPtr (new pcl::Indices);
    ground_truth_indices.truss = pcl::IndicesPtr (new pcl::Indices);

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;

    // Read pointcloud
    cloud_in_intensity = arvc::readCloudWithIntensity(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    *this->gt_ground_idx = *ground_truth_indices.ground;
    *this->gt_truss_idx = *ground_truth_indices.truss;
    this->cloud_in = arvc::parseToXYZ(cloud_in_intensity);

    // ***********************************************************************//
    // COARSE SEGMENTATITION
    // EXTRACT BIGGEST PLANE
    // This is temporal, only for get the correct biggest plane
    tmp_cloud = arvc::voxel_filter(this->cloud_in, 0.05f);

    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true, 0.5f, 1000);
    auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, tmp_plane_coefss, 0.5f);
    coarse_ground_indices = coarse_indices.first;
    coarse_truss_indices = coarse_indices.second;
    


    // CHECK CLOUDS
    if(this->visualize)
    {
      PointCloud::Ptr coarse_ground_cloud (new PointCloud);
      PointCloud::Ptr coarse_truss_cloud (new PointCloud);

      coarse_ground_cloud = arvc::extract_indices(this->cloud_in, coarse_ground_indices, false);
      coarse_truss_cloud = arvc::extract_indices(this->cloud_in, coarse_truss_indices, false);
      arvc::visualizeClouds(coarse_truss_cloud, coarse_ground_cloud);
    }

    // // ***********************************************************************//
    // // FINE SEGMENTATION
    // // FILTER CLUSTERS BY EIGEN VALUES
    // std::pair<vector<pcl::PointIndices>, int> regrow_output = arvc::regrow_segmentation(this->cloud_in, coarse_ground_indices);
    // // std::pair<vector<pcl::PointIndices>, int> regrow_output = arvc::regrow_segmentation(this->cloud_in);

    // regrow_clusters = regrow_output.first;
    // this->normals_time = regrow_output.second;

    // // VALIDATE CLUSTERS FROM THEIR EIGENVALUES
    // valid_clusters = arvc::validate_clusters_by_ratio(this->cloud_in, regrow_clusters, 0.3f);
    // // valid_clusters = arvc::validate_clusters_by_module(this->cloud_in, regrow_clusters, 1000.0f);
    // // valid_clusters = arvc::validate_clusters_hybrid(this->cloud_in, regrow_clusters, 0.3f, 1000.0f);

    // for(int clus_indx : valid_clusters)
    //   coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());




    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);

    // FILTER CLOUD BY DENISTY
    this->truss_idx = arvc::radius_outlier_removal(this->cloud_in, this->truss_idx, 0.1f, 5, false);
    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);



    // FINAL CLOUD
    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);
    // tmp_cloud = arvc::extract_indices(this->cloud_in, this->ground_idx, false);
    // (this->visualize) ?  arvc::visualizeClouds(this->cloud_out,0,0,170, tmp_cloud,0,0,170) : void();



    if(this->visualize){
      this->getConfMatrixIndexes();
      PointCloud::Ptr error_cloud (new PointCloud);
      PointCloud::Ptr truss_cloud (new PointCloud);
      PointCloud::Ptr ground_cloud (new PointCloud);
      pcl::IndicesPtr error_idx (new pcl::Indices);

      error_idx->insert(error_idx->end(), this->fp_idx->begin(), this->fp_idx->end());
      error_idx->insert(error_idx->end(), this->fn_idx->begin(), this->fn_idx->end());

      truss_cloud = arvc::extract_indices(cloud_in, this->tp_idx, false);
      ground_cloud = arvc::extract_indices(cloud_in,this->tn_idx, false);
      error_cloud = arvc::extract_indices(cloud_in, error_idx, false);

      pcl::visualization::PCLVisualizer my_vis;
      my_vis.setBackgroundColor(1,1,1);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0,255,0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color (ground_cloud, 100,100,100);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color (error_cloud, 255,0,0);

      my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
      my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
      my_vis.addPointCloud(error_cloud, error_color, "error_cloud");

      my_vis.addCoordinateSystem(0.8, "sensor_origin");
      auto pos = cloud_in->sensor_origin_;
      auto ori = cloud_in->sensor_orientation_;
      
      Eigen::Vector3f position(pos[0], pos[1], pos[2]);
      my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");

      while (!my_vis.wasStopped())
      {
        my_vis.spinOnce(100);
      }
    }

    auto start = std::chrono::high_resolution_clock::now();
    // Compute metrics
    if(this->compute_metrics)
    {
      this->cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
      this->metricas = arvc::computeMetrics(cm);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    this->metrics_time = duration.count();

    return 0;
  }

  int 
  run2(){
    PointCloudI::Ptr cloud_in_intensity (new PointCloudI);
    PointCloud::Ptr cloud_in_xyz (new PointCloud);
    PointCloud::Ptr cloud_out_xyz (new PointCloud);
    PointCloud::Ptr tmp_cloud (new PointCloud);
    PointCloud::Ptr uncluster_cloud (new PointCloud);

    pcl::IndicesPtr coarse_ground_indices (new pcl::Indices);
    pcl::IndicesPtr coarse_truss_indices (new pcl::Indices);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;
    ground_truth_indices.ground = pcl::IndicesPtr (new pcl::Indices);
    ground_truth_indices.truss = pcl::IndicesPtr (new pcl::Indices);

    pcl::IndicesPtr idx_high_density (new pcl::Indices);
    pcl::IndicesPtr idx_low_density (new pcl::Indices);

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;

    // Read pointcloud
    cloud_in_intensity = arvc::readCloudWithIntensity(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    *this->gt_ground_idx = *ground_truth_indices.ground;
    *this->gt_truss_idx = *ground_truth_indices.truss;
    this->cloud_in = arvc::parseToXYZ(cloud_in_intensity);

    // APPLY DENSITY FILTER
    idx_high_density = arvc::radius_outlier_removal(this->cloud_in, 0.1f, 5, false);
    idx_low_density = arvc::inverseIndices(this->cloud_in, idx_high_density);


    // ***********************************************************************//
    // COARSE SEGMENTATITION
    // EXTRACT BIGGEST PLANE
    // This is temporal, only for get the correct biggest plane
    tmp_cloud = arvc::voxel_filter(this->cloud_in, idx_high_density, 0.05f);

    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true, 0.5f, 1000);
    auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, idx_high_density, tmp_plane_coefss, 0.5f);
    coarse_ground_indices = coarse_indices.first;
    coarse_truss_indices = coarse_indices.second;
    

    // CHECK CLOUDS
    if(this->visualize)
    {
      PointCloud::Ptr coarse_ground_cloud (new PointCloud);
      PointCloud::Ptr coarse_truss_cloud (new PointCloud);

      coarse_ground_cloud = arvc::extract_indices(this->cloud_in, coarse_ground_indices, false);
      coarse_truss_cloud = arvc::extract_indices(this->cloud_in, coarse_truss_indices, false);
      arvc::visualizeClouds(coarse_truss_cloud, coarse_ground_cloud);
    }

    // ***********************************************************************//
    // FINE SEGMENTATION
    // FILTER CLUSTERS BY EIGEN VALUES
    std::pair<vector<pcl::PointIndices>, int> regrow_output = arvc::regrow_segmentation(this->cloud_in, coarse_ground_indices);
    // std::pair<vector<pcl::PointIndices>, int> regrow_output = arvc::regrow_segmentation(this->cloud_in, idx_high_density);

    regrow_clusters = regrow_output.first;
    this->normals_time = regrow_output.second;

    // VALIDATE CLUSTERS FROM THEIR EIGENVALUES
    // valid_clusters = arvc::validate_clusters_by_ratio(this->cloud_in, regrow_clusters, 0.3f);
    // valid_clusters = arvc::validate_clusters_by_module(this->cloud_in, regrow_clusters, 1000.0f);
    valid_clusters = arvc::validate_clusters_hybrid(this->cloud_in, regrow_clusters, 0.3f, 1000.0f);

    for(int clus_indx : valid_clusters)
      coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());


    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);

    // FINAL CLOUD
    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);
    // tmp_cloud = arvc::extract_indices(this->cloud_in, this->ground_idx, false);
    // (this->visualize) ?  arvc::visualizeClouds(this->cloud_out,0,0,170, tmp_cloud,0,0,170) : void();



    if(this->visualize){
      this->getConfMatrixIndexes();
      PointCloud::Ptr error_cloud (new PointCloud);
      PointCloud::Ptr truss_cloud (new PointCloud);
      PointCloud::Ptr ground_cloud (new PointCloud);
      pcl::IndicesPtr error_idx (new pcl::Indices);

      error_idx->insert(error_idx->end(), this->fp_idx->begin(), this->fp_idx->end());
      error_idx->insert(error_idx->end(), this->fn_idx->begin(), this->fn_idx->end());

      truss_cloud = arvc::extract_indices(cloud_in, this->tp_idx, false);
      ground_cloud = arvc::extract_indices(cloud_in,this->tn_idx, false);
      error_cloud = arvc::extract_indices(cloud_in, error_idx, false);

      pcl::visualization::PCLVisualizer my_vis;
      my_vis.setBackgroundColor(1,1,1);

      pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 0,255,0);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color (ground_cloud, 100,100,100);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color (error_cloud, 255,0,0);

      my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
      my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
      my_vis.addPointCloud(error_cloud, error_color, "error_cloud");

      my_vis.addCoordinateSystem(0.8, "sensor_origin");
      auto pos = cloud_in->sensor_origin_;
      auto ori = cloud_in->sensor_orientation_;
      
      Eigen::Vector3f position(pos[0], pos[1], pos[2]);
      my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");

      while (!my_vis.wasStopped())
      {
        my_vis.spinOnce(100);
      }
    }

    auto start = std::chrono::high_resolution_clock::now();
    // Compute metrics
    if(this->compute_metrics)
    {
      this->cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
      this->metricas = arvc::computeMetrics(cm);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    this->metrics_time = duration.count();

    return 0;
  }
};

/*
class segment_planes
{
private:

public:
  fs::path path;
  PointCloud::Ptr cloud_in;
  PointCloud::Ptr cloud_out;
  pcl::IndicesPtr truss_idx;
  pcl::IndicesPtr ground_idx;
  pcl::IndicesPtr gt_truss_idx;
  pcl::IndicesPtr gt_ground_idx;
  pcl::IndicesPtr low_density_idx;

  metrics metricas;
  conf_matrix cm;

  bool visualize;
  bool compute_metrics;

  // Constructor
  segment_planes(fs::path _path){
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);
    this->truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_truss_idx = pcl::IndicesPtr (new pcl::Indices);
    this->gt_ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->low_density_idx = pcl::IndicesPtr (new pcl::Indices);

    this->visualize = false;
    this->compute_metrics = false;
  }

  // Destructor
  ~segment_planes(){
    this->path.clear();
    this->cloud_in->clear();
    this->cloud_out->clear();
    this->visualize = false;
    this->compute_metrics = false;
  }

  int
  run()
  {
    PointCloudI::Ptr cloud_in_intensity (new PointCloudI);
    PointCloud::Ptr cloud_in_xyz (new PointCloud);
    PointCloud::Ptr cloud_out_xyz (new PointCloud);
    PointCloud::Ptr tmp_cloud (new PointCloud);
    pcl::IndicesPtr coarse_ground_indices (new pcl::Indices);
    pcl::IndicesPtr coarse_truss_indices (new pcl::Indices);

    pcl::IndicesPtr current_indices (new pcl::Indices);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    gt_indices ground_truth_indices;

    vector<pcl::PointIndices> regrow_clusters;
    vector<int> valid_clusters;

    cloud_in_intensity = arvc::readCloud(this->path);
    ground_truth_indices = arvc::getGroundTruthIndices(cloud_in_intensity);
    cloud_in_xyz = arvc::parseToXYZ(cloud_in_intensity);
    *this->cloud_in = *cloud_in_xyz;

    regrow_clusters = arvc::regrow_segmentation(this->cloud_in);
    cout << "Regrow clusters size: " << regrow_clusters.size() << endl;

    valid_clusters = arvc::validate_clusters_by_ratio(tmp_cloud, regrow_clusters, 0.3f);
    // valid_clusters = arvc::validate_clusters_by_module(tmp_cloud, regrow_clusters, 0.3f);
    // valid_clusters = arvc::validate_clusters_hybrid(this->cloud_in, regrow_clusters, 0.3f, 0.3f);


    for(int clus_indx : valid_clusters)
      coarse_truss_indices->insert(coarse_truss_indices->end(), regrow_clusters[clus_indx].indices.begin(), regrow_clusters[clus_indx].indices.end());

    // cout << "Coarse truss indices size: " << coarse_truss_indices->size() << endl;
    *this->truss_idx = *coarse_truss_indices;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);
    // cout << "Showing cloud after valid clusters" << endl;
    // (this->visualize) ?  arvc::visualizeCloud(this->cloud_out) : void();



    this->truss_idx = arvc::radius_outlier_removal(this->cloud_in, this->truss_idx, 0.1f, 5, false);
    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);

    this->cloud_out = arvc::extract_indices(this->cloud_in, this->truss_idx, false);

    (this->visualize) ?  arvc::visualizeClouds(this->cloud_in, this->cloud_out) : void();

    // Compute metrics
    if(this->compute_metrics)
    {
      this->cm = arvc::computeConfusionMatrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
      this->metricas = arvc::computeMetrics(cm);
    }

    return 0;
  }
};
*/

int main(int argc, char **argv)
{
  std::cout << YELLOW << "Running your code..." << RESET << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  vector<float> precision{};
  vector<float> recall{};
  vector<float> f1_score{};
  vector<float> accuracy{};
  vector<int> tp_vector{};
  vector<int> tn_vector{};
  vector<int> fp_vector{};
  vector<int> fn_vector{};

  bool visualize = false;
  bool compute_metrics = true;

  int normals_time = 0;
  int metrics_time = 0;

  std::vector<fs::path> path_vector;
  // EVERY CLOUD IN THE CURRENT FOLDER
  if(argc < 2)
  {
    fs::path current_dir = fs::current_path();
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    for(const fs::path &entry : tq::tqdm(path_vector))
    {
      remove_ground rg(entry);
      rg.visualize = visualize;
      rg.compute_metrics = compute_metrics;
      // rg.run();
      rg.run();
      normals_time += rg.normals_time;
      metrics_time += rg.metrics_time;

      if(rg.compute_metrics)
      {
        accuracy.push_back(rg.metricas.accuracy);
        precision.push_back(rg.metricas.precision);
        recall.push_back(rg.metricas.recall);
        f1_score.push_back(rg.metricas.f1_score);
        tp_vector.push_back(rg.cm.TP);
        tn_vector.push_back(rg.cm.TN);
        fp_vector.push_back(rg.cm.FP);
        fn_vector.push_back(rg.cm.FN);
      }
    } 
  }


  // ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else
  {
    // Get the input data
    fs::path entry = argv[1];
    remove_ground rg(entry);
  
    // segment_planes rg(entry);
    rg.visualize = visualize;
    rg.compute_metrics = compute_metrics;
    // rg.run();
    rg.run2();
    normals_time += rg.normals_time;
    metrics_time += rg.metrics_time;

    if(rg.compute_metrics){
      accuracy.push_back(rg.metricas.accuracy);
      precision.push_back(rg.metricas.precision);
      recall.push_back(rg.metricas.recall);
      f1_score.push_back(rg.metricas.f1_score);
      tp_vector.push_back(rg.cm.TP);
      tn_vector.push_back(rg.cm.TN);
      fp_vector.push_back(rg.cm.FP);
      fn_vector.push_back(rg.cm.FN);
    }


  }

  if(compute_metrics){
    cout << endl;
    cout << "Accuracy: " << arvc::mean(accuracy) << endl;
    cout << "Precision: " << arvc::mean(precision) << endl;
    cout << "Recall: " << arvc::mean(recall) << endl;
    cout << "F1 Score: " << arvc::mean(f1_score) << endl;
    cout << "TP: " << arvc::mean(tp_vector) << endl;
    cout << "TN: " << arvc::mean(tn_vector) << endl;
    cout << "FP: " << arvc::mean(fp_vector) << endl;
    cout << "FN: " << arvc::mean(fn_vector) << endl;
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << YELLOW << "Code end!!" << RESET << std::endl;
  std::cout << "Num of files: " << path_vector.size() << std::endl;
  std::cout << "Total Computation Time: " << duration.count() << " ms" << std::endl;
  std::cout << "Computation time without normal estimation: " << duration.count() - normals_time << " ms" << std::endl;
  cout << "Average Computation Time: " << duration.count()/path_vector.size() << " ms" << endl;
  cout << "Average Computation Time Without normal estimation and metrics: " << (duration.count()-normals_time-metrics_time)/path_vector.size() << " ms" << endl;

  return 0;
}