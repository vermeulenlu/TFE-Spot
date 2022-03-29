#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <istream>
#include <ostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <fstream>
#include <ctime>
#include "include/a_star.hpp"
#include "include/d_star_lite.hpp"
#include "include/jump_point_search.hpp"
#include "include/lpa_star.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/core.hpp> 
// Create a variable to save the position value in track 
int blurAmount=15; 
struct MouseParams
{
    cv::Mat img;
    cv::Point begin; 
    cv::Point pt;

};

//Mouse callback 
static void onMouse(int event, int x, int y, int, void* userInput) 
{ 
  if(event !=cv::EVENT_LBUTTONDOWN) 
          return; 

  // Get the pointer input image 
  MouseParams* mp = (MouseParams*)userInput;
  mp->pt = cv::Point(x,y);

  cv::Mat *img= &(mp->img); 
  // Draw circle
  cv::Mat temp = (*img).clone();
  circle(temp, cv::Point(x, y), 10, cv::Scalar(0, 255, 0), 3);
  circle(temp, (*mp).begin, 10, cv::Scalar(0, 0, 255), 3);

  // Call on change to get blurred image
  cv::imshow("Lena", temp); 
}  
using namespace std::chrono_literals;

void create_img(int width, int height, const std::vector<std::vector<int>>&   map,  cv::Mat &mat_img ){
cv::Mat mat_img_1(width, height, CV_8UC3, cv::Scalar(0, 0, 0));

for(int i=0;i<width;i++){
    for(int j=0;j<height;j++){
      cv::Vec3b & color = mat_img_1.at<cv::Vec3b>(i,j);
      if (map[i][j]==3)
        color = cv::Vec3b(255,255,0);      
      else if (map[i][j]==1)
          color = cv::Vec3b(255,255,255);
    }
}
	cv::resize(mat_img_1, mat_img, cv::Size(width*4, height*4), cv::INTER_LINEAR);
    
}
void visualization(int width, int height, std::vector<std::vector<int>>&   map ){
  cv::Mat mat_img;
create_img(width, height, map, mat_img);
cv::imshow("hello world", mat_img);
    int k = cv::waitKey(0); // Wait for a keystroke in the window
}
void choseInput(int width, int height, std::vector<std::vector<int>>&   map, const cv::Point &point_input, cv::Point &point_dest)
{

  cv::   namedWindow("Lena"); 
  cv::Mat mat_img;
  create_img(width, height, map, mat_img);
  std::cout << mat_img.at<cv::Vec3b>(0,0);
  MouseParams mp{mat_img,cv::Point(point_input.x*4,point_input.x*4), cv::Point(0,0)};
    circle(mat_img, (mp).begin, 10, cv::Scalar(0, 0, 255), 3);

  cv::imshow("Lena", mat_img); 
  cv::setMouseCallback("Lena", onMouse, &mp); 
  cv::waitKey(0); 
  cv:: destroyWindow("Lena"); 
  std::cout <<(mp.pt).x;
  point_dest.x = (mp.pt).y/4;
  point_dest.y = (mp.pt).x/4;

}
pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

Eigen::Matrix3f get_matrix_from_angles(Eigen::Vector3f angles) {

  Eigen::Matrix3f rot;

  float c1 = std::cos(angles(0));
  float c2 = std::cos(angles(1));
  float c3 = std::cos(angles(2));

  float s1 = std::sin(angles(0));
  float s2 = std::sin(angles(1));
  float s3 = std::sin(angles(2));

  rot(0, 0) = c1 * c2;
  rot(0, 1) = c1 * s2 * s3 - c3 * s1;
  rot(0, 2) = s1 * s3 + c1 * c3 * s2;

  rot(1, 0) = c2 * s1;
  rot(1, 1) = c1 * c3 + s1 * s2 * s3;
  rot(1, 2) = c3 * s1 * s2 - c1 * s3;

  rot(2, 0) = -s2;
  rot(2, 1) = c2 * s3;
  rot(2, 2) = c2 * c3;

  return rot;

}

Eigen::Matrix4f get_matrix_from_transform(Eigen::VectorXf transform) {

  Eigen::Matrix4f m;
  m = Eigen::Matrix4f::Identity();

  Eigen::Vector3f angles(transform(0), transform(1), transform(2));

  m.block<3, 3>(0, 0) = get_matrix_from_angles(angles);
  m(0, 3) = transform(3);
  m(1, 3) = transform(4);
  m(2, 3) = transform(5);

  return m;
}

Eigen::Vector3f get_angles_from_matrix(Eigen::Matrix3f rot) {

  float alpha = std::atan2(rot(1, 0), rot(0, 0));
  float beta = std::atan2(-rot(2, 0), std::sqrt(1 - rot(2, 0) * rot(2, 0)));
  float gamma = std::atan2(rot(2, 1), rot(2, 2));

  Eigen::Vector3f angles;
  angles << alpha, beta, gamma;

  return angles;
}

void save_slam_odometries(std::vector<Eigen::Matrix4f>& slam_odometries, std::string folder_name) {
  std::ostringstream oss;
  oss << folder_name + std::string("/save/slam_odometries");
  std::ofstream f(oss.str());

  for (int i = 0; i < slam_odometries.size(); i++) {
    Eigen::Vector3f angles = get_angles_from_matrix(slam_odometries[i].block<3, 3>(0, 0));
    f << angles(0) << " ";
    f << angles(1) << " ";
    f << angles(2) << " ";
    f << slam_odometries[i](0, 3) << " ";
    f << slam_odometries[i](1, 3) << " ";
    f << slam_odometries[i](2, 3) << " ";
    f << "\n";
  }
}

void save_slam_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& raw_clouds, std::string folder_name) {
  for (int i = 0; i < raw_clouds.size(); i++) {
    std::ostringstream oss;
    oss << folder_name + std::string("/save/cloud_") << i << std::string(".pcd");
    pcl::io::savePCDFileASCII(oss.str(), *(raw_clouds[i]));
  }
}

void get_slam_odometries(std::vector<Eigen::Matrix4f>& slam_odometries, std::string folder_name, int n) {
  std::ostringstream oss;
  oss << folder_name + std::string("/save/slam_odometries");
  std::ifstream f(oss.str());

  std::cout << "Start get slam odometries" << std::endl;

  Eigen::VectorXf v(6);
  v(0) = 0.0;
  v(1) = 0.0;
  v(2) = 0.0;
  v(3) = 0.0;
  v(4) = 0.0;
  v(5) = 0.0;

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 6; j++) {
      float value;
      f >> value;
      v(j) = value;
    }
    slam_odometries.push_back(get_matrix_from_transform(v));
    std::cout << "Got slam odometry : " << i << std::endl;
  }
}

void get_slam_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& raw_clouds, std::string folder_name, int n) {
  for (int i = 0; i < n; i++) {
    std::ostringstream oss;
    oss << folder_name + std::string("/save/cloud_") << i << std::string(".pcd");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (oss.str(), *cloud);

    raw_clouds.push_back(cloud);
    std::cout << "Got slam cloud : " << i << std::endl;
  }
}

void vizualize_saved_slam(pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr, pcl::visualization::PCLVisualizer::Ptr viewer, std::string folder_name) {
  std::vector<Eigen::Matrix4f> slam_odometries;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> raw_clouds;
  get_slam_odometries(slam_odometries, folder_name, 288);
  get_slam_clouds(raw_clouds, folder_name, 288);

  std::cout << "Start viz" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 
  for (int i = 0; i < slam_odometries.size(); i++) {

    std::cout << "Start viz : " << i << std::endl;

    float ratio = ((float) i) / (slam_odometries.size() - 1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB point;
    std::uint8_t r(255);
    std::uint8_t g(255);
    std::uint8_t b(15);

    if (ratio < 0.5) {
        r = (1.0 - ratio / 0.5) * (255 - 15) + 15;
        g = 15;
        b = (ratio / 0.5) * (255 - 15) + 15;
    } else {
        g = ((ratio - 0.5) / 0.5) * (255 - 15) + 15;
        b = (1.0 - (ratio - 0.5) / 0.5) * (255 - 15) + 15;
        r = 15;
    }

    for (int j = 0; j < raw_clouds[i]->points.size(); j++) {

      point.x = raw_clouds[i]->points[j].x;
      point.y = raw_clouds[i]->points[j].y;
      point.z = raw_clouds[i]->points[j].z;

      float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      if (dist < 1 || dist > 10) {
        continue;
      }

      std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 | static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      cloud_colored->push_back(point);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud (*cloud_colored, *cloud_colored_transformed, slam_odometries[i]);

    *all_cloud += *cloud_colored_transformed; 
  }

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud_ds(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
  downSizeFilter.setInputCloud(all_cloud);
  downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
  downSizeFilter.filter(*basic_cloud_ptr);

  viewer->addPointCloud<pcl::PointXYZRGB>(basic_cloud_ptr, "map");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
}

void Print_Map(std::vector<std::vector<int>>& map, int dim, std::string folder_name){
  // Writing into a file to visualise
  ofstream myfile;
  myfile.open (folder_name);

  for(int i=0;i<dim;i++)
  {
    for(int j=0;j<dim;j++)
    {
      myfile<<map[i][j];
    }
    myfile<<endl;
  }
  myfile.close();
}


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("./save/cloud_100.pcd", *basic_cloud_ptr) == -1) //* load the file
  {
     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
     return (-1);
  }

  // Filter to keep only a part of the point cloud and save it into another one
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtred_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  basic_cloud_ptr->width = basic_cloud_ptr->size ();
  basic_cloud_ptr->height = 1;
  std::cout << "Width :" << basic_cloud_ptr->width << std::endl;
  for(int i=0; i<basic_cloud_ptr->width*basic_cloud_ptr->height ; i++){
    if(basic_cloud_ptr->points[i].z < 0.6 && basic_cloud_ptr->points[i].z > 0.4){
      pcl::PointXYZ adding_point;
      adding_point.x = basic_cloud_ptr->points[i].x;
      adding_point.y = basic_cloud_ptr->points[i].y;
      adding_point.z = 0.0;
      filtred_cloud_ptr->points.push_back(adding_point);
    }
  }
  //TODO : Voir le temps que ça te prend pour créer la map et pour créer ton path.
  //TODO : créer une itnerface
  // Add informations about the pointcloud
  filtred_cloud_ptr->width = filtred_cloud_ptr->size ();
  filtred_cloud_ptr->height = 1;
  pcl::PointXYZ MinPt, MaxPt;
  pcl::getMinMax3D (*filtred_cloud_ptr, MinPt, MaxPt);

  // Creation of the grid 
  double margin = 1.0;
  double resolution = 0.15;
  int width = (MaxPt.x - MinPt.x)+margin;
  int height = (MaxPt.y - MinPt.y)+margin;
  double dimension = int(std::max(width,height)/resolution);
  int width_map = int(dimension);
  int height_map = int(dimension);
  int radius_inflate = int(0.3/resolution);
  int k;
  int q;

  // My grid on which I will perform Path P
  std::vector<std::vector<int>> map(width_map, std::vector<int>(height_map, 0));
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(filtred_cloud_ptr);
  pcl::PointXYZ searchPoint;
  
  // Writing into to map
  for(int i=0;i<width_map;i++)
  {
    for(int j=0;j<height_map;j++)
    {
      searchPoint.x=(MinPt.x-margin/2) + (i*resolution);
      searchPoint.y=(MinPt.y-margin/2) + (j*resolution);
      searchPoint.z=0;
    
      int K=1;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
      if(kdtree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance)>0)
      {
        if(sqrt(pointNKNSquaredDistance[0])<(resolution/2))
        {
          for(k=i-radius_inflate; k<i+radius_inflate; k++){
            if(k<0 || k>=dimension){
              continue;
            }
            for(q=j-radius_inflate; q<j+radius_inflate; q++){
              if(q<0 || q>=dimension){
                continue;
              }
              map[k][q] = 1;
            }
          }
        }
        else{
          if(map[i][j] == 0){
            map[i][j] = 0;
          }
        }
      }
    }
  }
  cv::Point final = cv::Point(0,0);
  cv::Point begin = cv::Point(width_map/2,height_map/2);

  choseInput(width_map, height_map,   map,begin, final);
  // Creating goal and start position
  int n = dimension;
  // Normalement l'input on le prend via l'odométrie. 

  Node start(begin.x, begin.y, 0, 0, 0, 0);
  Node goal(final.x, final.y, 0, 0, 0, 0);
  //Node goal(dimension/2, dimension/2, 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  map[start.x_][start.y_] = 0;
  map[goal.x_][goal.y_] = 0;

  std::vector<std::vector<int>> main_grid = map;
  clock_t time_req;

  time_req = clock();
  std::cout << "--------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: A* ---------------------" << '\n'
            << "--------------------------------------------------------" << '\n';
  AStar a_star(map);
  {
    map = main_grid;
    const auto [path_found, path_vector] = a_star.Plan(start, goal);
    PrintPath(path_vector, start, goal, map);
    Print_Map(map, dimension, "../a_star.txt");
    visualization(dimension, dimension,map);
  }
  time_req = clock() - time_req;
	cout << "A_star " << (float)time_req/CLOCKS_PER_SEC << " seconds" << endl;

  time_req = clock();
  std::cout << "-----------------------------------------------------------------------" << '\n';
  std::cout << "--------------------- ALGORITHM: Jump Point Search ---------------------" << '\n';
  std::cout << "-----------------------------------------------------------------------" << '\n';
  JumpPointSearch jump_point_search(map);
  {
    map = main_grid;
    const auto [path_found, path_vector] = jump_point_search.Plan(start, goal);
    PrintPath(path_vector, start, goal, map);
    Print_Map(map, dimension, "../jump_point_search.txt");
    visualization(dimension, dimension,map);

  }
  time_req = clock() - time_req;
	cout << "JPS " << (float)time_req/CLOCKS_PER_SEC << " seconds" << endl;

  time_req = clock();
  std::cout << "--------------------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: Lifelong Planning A* ---------------------" << '\n'
            << "--------------------------------------------------------------------------" << '\n';
  LPAStar lpa_star(map);
  {
    map = main_grid;
    const auto [path_found, path_vector] = lpa_star.Plan(start, goal);
    PrintPath(path_vector, start, goal, map);
    Print_Map(map, dimension, "../lpa_star.txt");
    visualization(dimension, dimension,map);

  }
  time_req = clock() - time_req;
	cout << "LPA " << (float)time_req/CLOCKS_PER_SEC << " seconds" << endl;

  time_req = clock();
  std::cout << "-------------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: D* Lite ---------------------" << '\n'
            << "-------------------------------------------------------------" << '\n';
  DStarLite d_star_lite(map);
  {
    map = main_grid;
    const auto [path_found, path_vector] = d_star_lite.Plan(start, goal);
    PrintPath(path_vector, start, goal, map);
    Print_Map(map, dimension, "../d_star_lite.txt");
        visualization(dimension, dimension,map);

  }
  time_req = clock() - time_req;
	cout << "D_star " << (float)time_req/CLOCKS_PER_SEC << " seconds" << endl;

  // //Visualisation
  // pcl::visualization::PCLVisualizer::Ptr viewer;
  // viewer = simpleVis(filtred_cloud_ptr);

  // //--------------------
  // // -----Main loop-----
  // //--------------------
  // while (!viewer->wasStopped ())
  // {
  //   viewer->spinOnce (100);
  //   std::this_thread::sleep_for(100ms);
  // }

    return 0;
}
