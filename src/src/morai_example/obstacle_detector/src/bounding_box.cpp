#include "header.h"
#include "dbscan.h"

using namespace std;

int minPoints = 3; //10          //Core Point 기준 필요 인접점 최소 개수
double epsilon = 0.5; //0.3        //Core Point 기준 주변 탐색 반경 

int minClusterSize = 10;
int maxClusterSize = 1000; //10000  //Cluster 최대 사이즈

double xMinROI = 0;
double xMaxROI = 7; //7 
double yMinROI = -3; //-3
double yMaxROI = 3;//3 
double zMinROI = -1; 
double zMaxROI = 3;

double xMinBoundingBox = 0.1;
double xMaxBoundingBox = 1;
double yMinBoundingBox = 0.1;
double yMaxBoundingBox = 1;
double zMinBoundingBox = 0.5;
double zMaxBoundingBox = 2; // BoundingBox 크기 범위 지정 변수 


typedef pcl::PointXYZ PointT;
vector<float> obstacle;
vector< vector<float> > obstacle_vec;
ros::Publisher roiPub; //ROI Publishser
ros::Publisher clusterPub; //Cluster Publishser
ros::Publisher boundingBoxPub; //Bounding Box Visualization Publisher
ros::Publisher DynamicObsPub;
ros::Publisher DynamicXPub;
ros::Publisher DynamicYPub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& inputcloud) {
  //ROS message 변환
  //PointXYZI가 아닌 PointXYZ로 선언하는 이유 -> 각각의 Cluster를 다른 색으로 표현해주기 위해서. Clustering 이후 각각 구별되는 intensity value를 넣어줄 예정.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*inputcloud, *cloud);

  std_msgs::Bool DynamicObsDetected;
  DynamicObsDetected.data = false;
  std_msgs::Float64 DynamicX;
  DynamicX.data = 0;
  std_msgs::Float64 DynamicY;
  DynamicY.data = 0;
  
  //Visualizing에 필요한 Marker 선언
  visualization_msgs::Marker boundingBox;
  visualization_msgs::MarkerArray boundingBoxArray;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> xfilter;
  xfilter.setInputCloud(cloud);
  xfilter.setFilterFieldName("x"); //x축을 자를 거다!
  xfilter.setFilterLimits(xMinROI, xMaxROI); 
  xfilter.setFilterLimitsNegative(false);
  xfilter.filter(*cloud_xf); // 결과적으로 cloud_xf타입에다가 필터한 내용을 넣어줘! could -> cloud_xf

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> yfilter;
  yfilter.setInputCloud(cloud_xf);
  yfilter.setFilterFieldName("y");
  yfilter.setFilterLimits(yMinROI, yMaxROI);
  yfilter.setFilterLimitsNegative(false);
  yfilter.filter(*cloud_xyf);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyzf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> zfilter;
  zfilter.setInputCloud(cloud_xyf);
  zfilter.setFilterFieldName("z");
  zfilter.setFilterLimits(zMinROI, zMaxROI); // -0.62, 0.0
  zfilter.setFilterLimitsNegative(false);
  zfilter.filter(*cloud_xyzf);

  // //Voxel Grid를 이용한 DownSampling
  // pcl::VoxelGrid<pcl::PointXYZ> vg;    // VoxelGrid 선언
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //Filtering 된 Data를 담을 PointCloud 선언
  // vg.setInputCloud(cloud);             // Raw Data 입력
  // vg.setLeafSize(0.5f, 0.5f, 0.5f); // 사이즈를 너무 작게 하면 샘플링 에러 발생
  // vg.filter(*cloud);          // Filtering 된 Data를 cloud PointCloud에 삽입

  // 효과 미미하다

  //KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  if (cloud_xyzf->size() > 0) {
    tree->setInputCloud(cloud_xyzf);
  }
  //Segmentation
  vector<pcl::PointIndices> cluster_indices; // 클러스터링 된 pointcloud가 들어갈 곳.
  
  //DBSCAN with Kdtree for accelerating
  DBSCANKdtreeCluster<pcl::PointXYZ> dc;
  dc.setCorePointMinPts(minPoints);   //Set minimum number of neighbor points
  dc.setClusterTolerance(epsilon); //Set Epsilon 
  dc.setMinClusterSize(minClusterSize);            

  dc.setMaxClusterSize(maxClusterSize);
  dc.setSearchMethod(tree);
  dc.setInputCloud(cloud_xyzf); //input data를 선언
  dc.extract(cluster_indices); //결과가 저장됨

  pcl::PointCloud<pcl::PointXYZI> totalcloud_clustered;
  int cluster_id = 0;

  //각 Cluster 접근
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {
    pcl::PointCloud<pcl::PointXYZI> eachcloud_clustered;
    float cluster_counts = cluster_indices.size();
    //각 Cluster내 각 Point 접근
    for(vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
        pcl::PointXYZI tmp;
        tmp.x = cloud_xyzf->points[*pit].x;
        tmp.y = cloud_xyzf->points[*pit].y;
        tmp.z = cloud_xyzf->points[*pit].z;
        tmp.intensity = cluster_id%8;
        eachcloud_clustered.push_back(tmp); // 총 2개로 복붙하는 중. bounding box를 칠려고 포인트 값을 일일이 땃다
        totalcloud_clustered.push_back(tmp);
    }

    //minPoint와 maxPoint 받아오기
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

    float x_len = abs(maxPoint.x - minPoint.x);   //직육면체 x 모서리 크기
    float y_len = abs(maxPoint.y - minPoint.y);   //직육면체 y 모서리 크기
    float z_len = abs(maxPoint.z - minPoint.z);   //직육면체 z 모서리 크기 
    float volume = x_len * y_len * z_len;         //직육면체 부피 (쓸일이 없다)

    float center_x = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
    float center_y = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
    float center_z = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표 

    float distance = sqrt(center_x * center_x + center_y * center_y); //장애물 <-> 차량 거리(라이다는 0,0,0 이다.)

    // visualzation하는 과정이다.
    if ( (xMinBoundingBox < x_len && x_len < xMaxBoundingBox) && (yMinBoundingBox < y_len && y_len < yMaxBoundingBox) && (zMinBoundingBox < z_len && z_len < zMaxBoundingBox) ) {
      boundingBox.header.frame_id = "velodyne"; //좌표계설정
      boundingBox.header.stamp = ros::Time(); // 식별자. 시간을 넣어줌
      boundingBox.ns = cluster_counts; //ns = namespace
      boundingBox.id = cluster_id; 
      boundingBox.type = visualization_msgs::Marker::CYLINDER; //직육면체로 표시
      boundingBox.action = visualization_msgs::Marker::ADD;

      boundingBox.pose.position.x = center_x; 
      boundingBox.pose.position.y = center_y;
      boundingBox.pose.position.z = center_z;

      boundingBox.pose.orientation.x = 0.0; //무시해도 됨
      boundingBox.pose.orientation.y = 0.0;
      boundingBox.pose.orientation.z = 0.0;
      boundingBox.pose.orientation.w = 1.0;

      boundingBox.scale.x = x_len;
      boundingBox.scale.y = y_len;
      boundingBox.scale.z = z_len;

      boundingBox.color.a = 0.8; //직육면체 투명도, a = alpha
      boundingBox.color.r = 1; //직육면체 색상 RGB값
      boundingBox.color.g = 0;
      boundingBox.color.b = 0;

      boundingBox.lifetime = ros::Duration(0.1); //box 지속시간(길면은 bounding box 길이가 계속 머무른다. 짧으면 실시간성이 높아진다)
      boundingBoxArray.markers.emplace_back(boundingBox); // push_back 보다 속도가 빠르다.
    }

    if (boundingBoxArray.markers.size() > 0) {
      for (int i = 0; i < boundingBoxArray.markers.size(); i++){
        vector<float>().swap(obstacle);

        obstacle.emplace_back(boundingBoxArray.markers[i].pose.position.x);
        obstacle.emplace_back(boundingBoxArray.markers[i].pose.position.y);
        obstacle.emplace_back(boundingBoxArray.markers[i].pose.position.z);
        obstacle.emplace_back(distance);
        obstacle_vec.emplace_back(obstacle);
      }

      sort(obstacle_vec.begin(), obstacle_vec.end());
      // if (3.5 <= obstacle_vec[0][0] && obstacle_vec[0][0] < 8) {
      if (0 <= obstacle_vec[0][3] && obstacle_vec[0][3] < 6) { //4 // 6
        DynamicObsDetected.data = true;
        DynamicX.data = obstacle_vec[0][0];
        DynamicY.data = obstacle_vec[0][1];
      }
      else {
        DynamicObsDetected.data = false;
      }
    }

    vector< vector<float> >().swap(obstacle_vec);  



    //사람도 bounding box를 시각화 할 수 있다.
    cluster_id++; //intensity 증가 
    //bounding box마다 다른 id값을 줘야만 한다. 그래서 증가식을 넣어주자!

  }

  //debuging을 위해서 cloud를 새로 다 선언하면서 내려왔다!!

  //Convert To ROS data type
  pcl::PCLPointCloud2 cloud_p;
  pcl::toPCLPointCloud2(totalcloud_clustered, cloud_p);

  sensor_msgs::PointCloud2 cluster;
  pcl_conversions::fromPCL(cloud_p, cluster); //clustering 된 채로 그냥 남아있어서 그냥 이거를 rviz 상에서 볼 수 있다.
  cluster.header.frame_id = "velodyne";


  pcl::PCLPointCloud2 cloud_cropbox;
  pcl::toPCLPointCloud2(*cloud_xyzf, cloud_cropbox);

  sensor_msgs::PointCloud2 ROI;
  pcl_conversions::fromPCL(cloud_cropbox, ROI); //ROI 된 채로 남아있는 상태를 rviz상에서 볼 수 있다.
  ROI.header.frame_id = "velodyne";


  roiPub.publish(ROI);
  clusterPub.publish(cluster);
  boundingBoxPub.publish(boundingBoxArray);
  DynamicObsPub.publish(DynamicObsDetected);
  DynamicXPub.publish(DynamicX);
  DynamicYPub.publish(DynamicY);

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "bounding_box");
  ros::NodeHandle nh;

  ros::Subscriber rawDataSub = nh.subscribe("/velodyne_points", 1, cloud_cb);  // velodyne_points 토픽 구독. velodyne_points = 라이다 raw data

  roiPub = nh.advertise<sensor_msgs::PointCloud2>("/roi", 0.001); 
  clusterPub = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 0.001);                  
  boundingBoxPub = nh.advertise<visualization_msgs::MarkerArray>("/boundingBox", 0.001);
  DynamicObsPub = nh.advertise<std_msgs::Bool>("/dynamic_obs", 0.001);
  DynamicXPub = nh.advertise<std_msgs::Float64>("/dynamic_x", 0.001);
  DynamicYPub = nh.advertise<std_msgs::Float64>("/dynamic_y", 0.001);
  //값 받아오기 

  ros::spin();

  return 0;
}
