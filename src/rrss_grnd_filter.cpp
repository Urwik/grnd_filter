// C++
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>

#include "arvc_utils_v2.hpp"
#include "tqdm.hpp"

#include "arvc_metrics.hpp"
// #include "arvc_console.hpp"
// #include "arvc_viewer.hpp"


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//****************************************************************************//
// TYPE DEFINITIONS ////////////////////////////////////////////////////////////

namespace fs = std::filesystem;
using namespace std;

arvc::console cons;


class remove_ground
{
private:
  /* data */
public:
  fs::path path;

  PointCloud::Ptr cloud_in;
  PointCloud::Ptr cloud_out;

  pcl::IndicesPtr coarse_ground_idx;
  pcl::IndicesPtr coarse_truss_idx;

  vector<pcl::PointIndices> regrow_clusters;
  vector<int> valid_clusters;

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
  bool enable_metrics;

  string mode;

  remove_ground(fs::path _path)
  {
    this->path = _path;
    this->cloud_in = PointCloud::Ptr (new PointCloud);
    this->cloud_out = PointCloud::Ptr (new PointCloud);

    this->coarse_ground_idx = pcl::IndicesPtr (new pcl::Indices);
    this->coarse_truss_idx = pcl::IndicesPtr (new pcl::Indices);

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
    this->enable_metrics = false;
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
  


  void read_cloud(){
    cons.debug("Reading cloud");
    PointCloudLN::Ptr cloud_in_labelled (new PointCloudLN);
    
    cloud_in_labelled = arvc::readCloudWithLabelNormal(this->path);

    this->get_ground_truth_indices(cloud_in_labelled);
    this->cloud_in = arvc::parseToXYZ(cloud_in_labelled);
  }


  void get_ground_truth_indices(PointCloudLN::Ptr& _cloud){
    gt_indices ground_truth_indices;
    ground_truth_indices.ground.reset(new pcl::Indices); 
    ground_truth_indices.truss.reset(new pcl::Indices); 
    ground_truth_indices = arvc::getGroundTruthIndices(_cloud);
    *this->gt_ground_idx = *ground_truth_indices.ground;
    *this->gt_truss_idx = *ground_truth_indices.truss;
  }


  void coarse_segmentation(){
    cons.debug("Coarse segmentation");
    PointCloud::Ptr tmp_cloud (new PointCloud);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    tmp_cloud = arvc::voxel_filter(this->cloud_in, 0.05f);
    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true, 0.5f, 1000);
    auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, tmp_plane_coefss, 0.5f);
    this->coarse_ground_idx = coarse_indices.first;
    this->coarse_truss_idx = coarse_indices.second;
  }


  void view_indices(pcl::IndicesPtr& _indices){
    PointCloud::Ptr cloud (new PointCloud);
    cloud = arvc::extract_indices(this->cloud_in, _indices, false);
    arvc::visualizeCloud(cloud);
  }


  void fine_segmentation(){
    cons.debug("Fine segmentation");

    std::pair<vector<pcl::PointIndices>, int> regrow_output = arvc::regrow_segmentation(this->cloud_in, this->coarse_ground_idx);

    this->regrow_clusters = regrow_output.first;
    this->normals_time = regrow_output.second;

    this->validate_clusters();

    // Append valid clusters to truss indices
    for(int clus_indx : this->valid_clusters)
      this->coarse_truss_idx->insert(this->coarse_truss_idx->end(), this->regrow_clusters[clus_indx].indices.begin(), this->regrow_clusters[clus_indx].indices.end());

  }


  void validate_clusters(){
    cons.debug("Validating clusters");

    map<string, int> mode_dict;
    mode_dict["ratio"] = 0;
    mode_dict["module"] = 1;
    mode_dict["hybrid"] = 2;


    switch (mode_dict[this->mode])
    {
    case 0:
      this->valid_clusters = this->validate_clusters_by_ratio(this->cloud_in, this->regrow_clusters, 0.3f);
      break;
    
    case 1:
      this->valid_clusters = this->validate_clusters_by_module(this->cloud_in, this->regrow_clusters, 1000.0f);
      break;

    case 2:
      this->valid_clusters = this->validate_clusters_hybrid(this->cloud_in, this->regrow_clusters, 0.3f, 1000.0f);
      break;

    default:
      break;
    }
  }


  void density_filter(){
    cons.debug("Density filter");

    this->truss_idx = arvc::radius_outlier_removal(this->cloud_in, this->truss_idx, 0.1f, 5, false);
    this->ground_idx = arvc::inverseIndices(this->cloud_in, this->truss_idx);
  }


  void update_segmentation(){
    // Update truss and ground indices for final segmentation
    *this->truss_idx = *this->coarse_truss_idx;
    *this->ground_idx = *arvc::inverseIndices(this->cloud_in, this->truss_idx);
  }


  void view_final_segmentation(){
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
      
      // Eigen::Vector3f position(pos[0], pos[1], pos[2]);
      // my_vis.addCube(position, ori, 0.3, 0.3, 0.3, "sensor_origin");

      while (!my_vis.wasStopped())
      {
        my_vis.spinOnce(100);
      }
  }

  
  void compute_metrics(){
    auto start = std::chrono::high_resolution_clock::now();
    
    this->cm = arvc::compute_conf_matrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);
    this->metricas = arvc::compute_metrics(cm);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    this->metrics_time = duration.count();
  }


  vector<int>
  validate_clusters_by_ratio(PointCloud::Ptr &_cloud_in, vector<pcl::PointIndices> &clusters, float _ratio_threshold)
  {
    vector<int> valid_clusters;
    valid_clusters.clear();

    int clust_indx = 0;
    for(auto cluster : clusters)
    {
      pcl::IndicesPtr current_cluster (new pcl::Indices);
      *current_cluster = cluster.indices;
      auto eig_values = arvc::compute_eigenvalues(_cloud_in, current_cluster, true);
      float size_ratio = eig_values(1)/eig_values(2);

      if (size_ratio <= _ratio_threshold){
        valid_clusters.push_back(clust_indx);
      }
        
      clust_indx++;
    }

    return valid_clusters;
  }


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
      
      float max_length = arvc::get_max_length(current_cluster_cloud);

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
      float max_length = arvc::get_max_length(current_cluster_cloud);
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


  int compute()
  {
    this->read_cloud();
    this->coarse_segmentation();
    this->fine_segmentation();
    this->update_segmentation();
    this->density_filter();

    if (this->visualize)
      this->view_final_segmentation();

    if(this->enable_metrics)
      this->compute_metrics();

    return 0;
  }
};





int main(int argc, char **argv)
{
  std::cout << YELLOW << "Running your code..." << RESET << std::endl;
  auto start = std::chrono::high_resolution_clock::now();

  cons.enable_debug = false;

  // CONFIGURATION PARAMS
  bool en_visual = false;
  bool en_metric = true;
  int normals_time = 0, metrics_time = 0;
  

  // VARIABLES UTILITIES
  arvc::metrics metricas;
  std::vector<fs::path> path_vector;
  


  // COMPUTE THE ALGORITHM FOR EVERY CLOUD IN THE CURRENT FOLDER

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
      rg.visualize = en_visual;
      rg.enable_metrics = en_metric;
      rg.mode = "module";
      rg.compute();

      normals_time += rg.normals_time;
      metrics_time += rg.metrics_time;

      if(en_metric)
      {
        metricas.accuracy.push_back(rg.metricas.accuracy);
        metricas.precision.push_back(rg.metricas.precision);
        metricas.recall.push_back(rg.metricas.recall);
        metricas.f1_score.push_back(rg.metricas.f1_score);
        metricas.tp_vector.push_back(rg.cm.TP);
        metricas.tn_vector.push_back(rg.cm.TN);
        metricas.fp_vector.push_back(rg.cm.FP);
        metricas.fn_vector.push_back(rg.cm.FN);
      }
    } 
  }



  // COMPUTE THE ALGORITHM ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER

  else {

    fs::path entry = argv[1];

    remove_ground rg(entry);
  
    rg.visualize = en_visual;
    rg.enable_metrics = en_metric;
    rg.mode = argv[2];

    rg.compute();

    normals_time += rg.normals_time;
    metrics_time += rg.metrics_time;

    if(en_metric){
      metricas.accuracy.push_back(rg.metricas.accuracy);
      metricas.precision.push_back(rg.metricas.precision);
      metricas.recall.push_back(rg.metricas.recall);
      metricas.f1_score.push_back(rg.metricas.f1_score);
      metricas.tp_vector.push_back(rg.cm.TP);
      metricas.tn_vector.push_back(rg.cm.TN);
      metricas.fp_vector.push_back(rg.cm.FP);
      metricas.fn_vector.push_back(rg.cm.FN);
    }
  }



  // PLOT METRICS
  
  if(en_metric) metricas.show(); 
    


  
  // PRINT COMPUTATION TIME

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