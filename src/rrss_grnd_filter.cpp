
// C++
// PREVIOUS VERSION TO THE CHANGE OF THE ARVC UTILS LIBRARY
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <chrono>

#include "tqdm.hpp"
#include "arvc_utils/arvc_utils.hpp"
#include "arvc_utils/arvc_metrics.hpp"
#include "arvc_utils/arvc_console.hpp"
#include "arvc_utils/arvc_viewer.hpp"

// #include "arvc_utils_v2. hpp"
// #include "arvc_console.hpp"
// #include "arvc_viewer.hpp"


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

  arvc::Metrics metricas;
  // metrics metricas;
  conf_matrix cm;

  int normals_time;
  int metrics_time;

  bool visualize;
  bool enable_metrics;
  bool debug_msgs;

  float module_threshold;
  float ratio_threshold;
  float ransac_threshold;

  string mode;
  string cloud_id;
  arvc::console cons;


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
    this->ratio_threshold = 0.3f;
    this->module_threshold = 1000.0f;
    this->ransac_threshold = 0.5f;
    this->mode = "hybrid";
    this->cloud_id = this->path.stem().string();

    this->debug_msgs = true;
    this->cons.enable = this->debug_msgs;
    this->cons.enable_error = this->debug_msgs;
    this->cons.enable_debug = this->debug_msgs;

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
    // For the index points classified as truss, check if this point is in the ground truth truss. If it is, it is a true positive, otherwise it is a false positive.
    for (size_t i = 0; i < this->truss_idx->size(); i++)
    {
      if(std::find(this->gt_truss_idx->begin(), this->gt_truss_idx->end(), this->truss_idx->at(i)) != this->gt_truss_idx->end())
        this->tp_idx->push_back(this->truss_idx->at(i));
      else
        this->fp_idx->push_back(this->truss_idx->at(i));
    }

    // For the index points classified as ground, check if this point is in the ground truth ground. If it is, it is a true negative, otherwise it is a false negative.
    for (size_t i = 0; i < this->ground_idx->size(); i++)
    {
      if(std::find(this->gt_ground_idx->begin(), this->gt_ground_idx->end(), this->ground_idx->at(i)) != this->gt_ground_idx->end())
        this->tn_idx->push_back(this->ground_idx->at(i));
      else
        this->fn_idx->push_back(this->ground_idx->at(i));
    }
    
  }  
  


  void read_cloud(){
    this->cons.debug("Reading cloud...", "YELLOW");
    PointCloudLN::Ptr cloud_in_labelled (new PointCloudLN);
    
    cloud_in_labelled = arvc::readPointCloud<PointLN>(this->path);

    this->get_ground_truth_indices(cloud_in_labelled);
    this->cloud_in = arvc::parseToXYZ(cloud_in_labelled);

    this->cons.debug("Cloud in size: " + to_string(this->cloud_in->size()), "YELLOW");
  }



  void get_ground_truth_indices(PointCloudLN::Ptr& _cloud){
    gt_indices ground_truth_indices;
    ground_truth_indices.ground.reset(new pcl::Indices); 
    ground_truth_indices.truss.reset(new pcl::Indices); 
    // ground_truth_indices = arvc::getGroundTruthIndices(_cloud);
    ground_truth_indices = arvc::getGroundTruthIndicesV2(_cloud);
    *this->gt_ground_idx = *ground_truth_indices.ground;
    *this->gt_truss_idx = *ground_truth_indices.truss;
  }



  void coarse_segmentation(){
    this->cons.debug("Coarse segmentation");
    PointCloud::Ptr tmp_cloud (new PointCloud);
    pcl::ModelCoefficientsPtr tmp_plane_coefss (new pcl::ModelCoefficients);

    tmp_cloud = arvc::voxel_filter(this->cloud_in, 0.05f);
    tmp_plane_coefss = arvc::compute_planar_ransac(tmp_cloud, true, this->ransac_threshold, 1000);
    auto coarse_indices = arvc::get_points_near_plane(this->cloud_in, tmp_plane_coefss, this->ransac_threshold);
    this->coarse_ground_idx = coarse_indices.first;
    this->coarse_truss_idx = coarse_indices.second;
  }



  void fine_segmentation(){
    this->cons.debug("Fine segmentation");

    std::pair<vector<pcl::PointIndices>, int> regrow_output;
    
    if(this->coarse_ground_idx->size() > 0)
      regrow_output = arvc::regrow_segmentation(this->cloud_in, this->coarse_ground_idx, false);
    else
      regrow_output = arvc::regrow_segmentation(this->cloud_in, false);  

    this->regrow_clusters = regrow_output.first;
    this->normals_time = regrow_output.second;

    this->validate_clusters();

    // Append valid clusters to truss indices
    for(int clus_indx : this->valid_clusters)
      this->coarse_truss_idx->insert(this->coarse_truss_idx->end(), this->regrow_clusters[clus_indx].indices.begin(), this->regrow_clusters[clus_indx].indices.end());

  }



  void validate_clusters(){
    this->cons.debug("Validating clusters");

    map<string, int> mode_dict;
    mode_dict["ratio"] = 0;
    mode_dict["module"] = 1;
    mode_dict["hybrid"] = 2;


    switch (mode_dict[this->mode])
    {
    case 0:
      this->valid_clusters = this->validate_clusters_by_ratio();
      break;
    
    case 1:
      this->valid_clusters = this->validate_clusters_by_module();
      break;

    case 2:
      this->valid_clusters = this->validate_clusters_hybrid();
      break;

    default:
      break;
    }
  }



  void density_filter(){
    this->cons.debug("Density filter");

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

      fs::path camera_params_path = "/home/arvc/workSpaces/rrss_grnd_filter/examples/minkunet_predictions/" + this->cloud_id + "_camera_params.txt";

      try
      {
        my_vis.loadCameraParameters(camera_params_path.string());
        // my_vis.loadCameraParameters("/home/arvc/workSpaces/code_ws/build/" + this->cloud_id + "_camera_params.txt");

      }
      catch(const std::exception& e)
      {
        
      }
      

      pcl::visualization::PointCloudColorHandlerCustom<PointT> truss_color (truss_cloud, 50,190,50);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> ground_color (ground_cloud, 100,100,100);
      pcl::visualization::PointCloudColorHandlerCustom<PointT> error_color (error_cloud, 200,10,10);

      my_vis.addPointCloud(truss_cloud, truss_color, "truss_cloud");
      my_vis.addPointCloud(ground_cloud, ground_color, "wrong_cloud");
      my_vis.addPointCloud(error_cloud, error_color, "error_cloud");

      my_vis.addCoordinateSystem(0.8, "sensor_origin");
      auto pos = cloud_in->sensor_origin_;
      auto ori = cloud_in->sensor_orientation_;
      

      while (!my_vis.wasStopped())
      {
        my_vis.saveCameraParameters(camera_params_path.string());
        my_vis.spinOnce(100);
      }
  }


  
  void compute_metrics(){
    auto start = std::chrono::high_resolution_clock::now();
    
    this->cm = arvc::compute_conf_matrix(this->gt_truss_idx, this->gt_ground_idx, this->truss_idx, this->ground_idx);

    this->metricas.computeMetricsFromConfusionMatrix(this->cm.TP, this->cm.FP, this->cm.FN, this->cm.TN);
    // this->metricas = arvc::compute_metrics(cm);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    this->metrics_time = duration.count();
  }


  bool
  valid_ratio(pcl::IndicesPtr& _cluster_indices)
  {
      auto eig_decomp = arvc::compute_eigen_decomposition(this->cloud_in, _cluster_indices, true);
      float size_ratio = eig_decomp.values(1)/eig_decomp.values(2);

      if (size_ratio <= this->ratio_threshold){
        return true;
      }
      else
        return false;
  }



  bool
  valid_module(pcl::IndicesPtr& _cluster_indices){

    // GET THE CLOUD REPRESENTING THE CLUSTER
    PointCloud::Ptr cluster_cloud (new PointCloud);
    cluster_cloud = arvc::extract_indices(this->cloud_in, _cluster_indices);
    
    // COMPUTE CLOUD CENTROID
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloud, centroid);

    // COMPUTE EIGEN DECOMPOSITION
    arvc::eig_decomp eigen_decomp = arvc::compute_eigen_decomposition(this->cloud_in, _cluster_indices, false);

    // Compute transform between the original cloud and the eigen vectors of the cluster
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigen_decomp.vectors.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());
    
    // Transform the origin to the centroid of the cluster and to its eigen vectors
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cluster_cloud, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

    Eigen::Vector3f max_values;
    max_values.x() = std::abs(maxPoint.x - minPoint.x);
    max_values.y() = std::abs(maxPoint.y - minPoint.y);
    max_values.z() = std::abs(maxPoint.z - minPoint.z);

    if (max_values.maxCoeff() < this->module_threshold)
      return true;
    else
      return false;
  }



  vector<int>
  validate_clusters_by_ratio()
  {
    vector<int> valid_clusters;
    valid_clusters.clear();
    pcl::IndicesPtr current_cluster (new pcl::Indices);
    
    int clust_indx = 0;
    for(auto cluster : this->regrow_clusters)
    {
      *current_cluster = cluster.indices;

      if (this->valid_ratio(current_cluster))
        valid_clusters.push_back(clust_indx);
        
      clust_indx++;
    }

    return valid_clusters;
  }



  vector<int>
  validate_clusters_by_module()
  {
    vector<int> valid_clusters;
    valid_clusters.clear();

    PointCloud::Ptr tmp_cloud (new PointCloud);
    PointCloud::Ptr current_cluster_cloud (new PointCloud);
    pcl::IndicesPtr current_cluster (new pcl::Indices);

    int clust_indx = 0;
    for(auto cluster : this->regrow_clusters)
    {
      *current_cluster = cluster.indices;

      if (this->valid_module(current_cluster))
        valid_clusters.push_back(clust_indx);
        
      clust_indx++;
    }

    return valid_clusters;
  }



  vector<int>
  validate_clusters_hybrid()
  {
    // cout << GREEN <<"Checking regrow clusters..." << RESET << endl;
    vector<int> valid_clusters;
    valid_clusters.clear();
    PointCloud::Ptr current_cluster_cloud (new PointCloud);
    PointCloud::Ptr remain_input_cloud (new PointCloud);
    pcl::IndicesPtr current_cluster (new pcl::Indices);

    int clust_indx = 0;
    for(auto cluster : this->regrow_clusters)
    {
      *current_cluster = cluster.indices;

      if (this->valid_module(current_cluster) && this->valid_ratio(current_cluster))
        valid_clusters.push_back(clust_indx);
      
      clust_indx++;
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


  int computeWOfine()
  {
    this->read_cloud();
    this->coarse_segmentation();
    // this->fine_segmentation();
    this->update_segmentation();
    this->density_filter();

    if (this->visualize)
      this->view_final_segmentation();

    if(this->enable_metrics)
      this->compute_metrics();

    return 0;
  }


  int computeWoCoarse()
  {
    this->read_cloud();
    // this->coarse_segmentation();
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

  bool enable_debug = true;

  // CONFIGURATION PARAMS
  bool en_visual = true;
  bool en_metric = false;

  float node_lngth = 2.0f;
  float node_width = 0.2f;
  float rsac_thrsh = 0.5f;

  float ratio_threshold = node_width / rsac_thrsh;    // original
  float module_threshold = rsac_thrsh;                // original
  
  int normals_time = 0, metrics_time = 0;
  

  // VARIABLES UTILITIES
  arvc::Metrics metricas;
  std::vector<fs::path> path_vector;
  
  int count = 5;

  vector<float> by_module_f1;
  vector<float> by_hybrid_f1;

  // COMPUTE THE ALGORITHM FOR EVERY CLOUD IN THE CURRENT FOLDER
  if(argc == 1)
  {
    fs::path current_dir = fs::current_path();
    for(const auto &entry : fs::directory_iterator(current_dir))
    {
      if(entry.path().extension() == ".pcd" || entry.path().extension() == ".ply")
        path_vector.push_back(entry.path());
    }

    // for(const fs::path &entry : tq::tqdm(path_vector))
    for(const fs::path &entry : path_vector)

    {
      remove_ground rg(entry);
      rg.visualize = en_visual;
      rg.enable_metrics = en_metric;
      rg.ratio_threshold = node_width / rsac_thrsh;    // original
      rg.module_threshold = rsac_thrsh;                // original
      rg.debug_msgs = enable_debug;

      rg.mode = "hybrid"; // DEFAULT MODE
      rg.compute();

      // // GET IMPORTANT CLOUDS
      // {
      //   float recall_hybrid = rg.metricas.values.recall;
      //   rg.computeWOfine();
      //   float recall_WOfine = rg.metricas.values.recall;

      //   if (recall_hybrid > recall_WOfine){
      //     cout << GREEN << rg.path.stem() << RESET << endl;

      //   }
      // }

      normals_time += rg.normals_time;
      metrics_time += rg.metrics_time;

      if(en_metric)
      {
        metricas.accuracy.push_back(rg.metricas.values.accuracy);
        metricas.precision.push_back(rg.metricas.values.precision);
        metricas.recall.push_back(rg.metricas.values.recall);   
        metricas.f1_score.push_back(rg.metricas.values.f1_score);
        metricas.tp.push_back(rg.metricas.values.tp);
        metricas.tn.push_back(rg.metricas.values.tn);
        metricas.fp.push_back(rg.metricas.values.fp);
        metricas.fn.push_back(rg.metricas.values.fn);
      }
    } 
  }

  
  // COMPUTE THE ALGORITHM ONLY ONE CLOUD PASSED AS ARGUMENT IN CURRENT FOLDER
  else if(argc == 3){

    fs::path entry = argv[1];
    remove_ground rg(entry);

    const string MODE = argv[2];
  
    rg.visualize = en_visual;
    rg.enable_metrics = en_metric;
    // rg.ratio_threshold = ratio_threshold;
    // rg.module_threshold = module_threshold;
    rg.ransac_threshold = rsac_thrsh;
    rg.debug_msgs = enable_debug;
    
    if (MODE == "wofine")
    {
      cout << "Entra en modo wofine" << endl;
      rg.ratio_threshold = node_width / rsac_thrsh;    // original
      rg.module_threshold = rsac_thrsh;     
      rg.computeWOfine();
    }
  
    else if (MODE == "wocoarse") // DEFAULT METHOD HYBRID
    {
      cout << "Entra en modo wocoarse" << endl;
      // UPATE THRESHOLDS FOR COMLETE FINE SEGMENTATION
      rg.ratio_threshold = node_width / node_lngth;   // wo coarse 
      rg.module_threshold = node_lngth;               // wo coarse
      rg.mode = "hybrid";
      rg.computeWoCoarse();
    }
  
    else if (MODE == "ratio" || MODE == "module" || MODE == "hybrid")
    {
      cout << "Entra en modo normal" << endl;
      rg.ratio_threshold = node_width / rsac_thrsh;    // original
      rg.module_threshold = rsac_thrsh;                // original
      rg.mode = MODE;
      rg.compute();
    }

    else{
      cout << "NO ENTRA EN NINGUN MODODEFAULT" << endl;
      return 1;
    }

    normals_time += rg.normals_time;
    metrics_time += rg.metrics_time;

    if(en_metric){
        metricas.accuracy.push_back(rg.metricas.values.accuracy);
        metricas.precision.push_back(rg.metricas.values.precision);
        metricas.recall.push_back(rg.metricas.values.recall);   
        metricas.f1_score.push_back(rg.metricas.values.f1_score);
        metricas.tp.push_back(rg.metricas.values.tp);
        metricas.tn.push_back(rg.metricas.values.tn);
        metricas.fp.push_back(rg.metricas.values.fp);
        metricas.fn.push_back(rg.metricas.values.fn);
    }

  }

  else{
    std::cout << "\tNo mode selected." << std::endl;
    std::cout << "\tUsage: ./rrss_grnd_filter <path_to_cloud> <mode>{ratio, module, hybrid, wofine, wocoarse}" << std::endl;
    return 1;
  }


  // PLOT METRICS
  
  if(en_metric) 
    metricas.plotMetrics(); 
    

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