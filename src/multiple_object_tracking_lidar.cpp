#include "multiple_object_tracking_lidar.hpp"

namespace multiple_object_tracking_lidar
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("multiple_object_tracking_lidar");

MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(
  const rclcpp::NodeOptions& options
): MultipleObjectTrackingLidar("", options)
{}

MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("MultipleObjectTrackingLidar", name_space, options)
{
  RCLCPP_INFO(this->get_logger(),"MultipleObjectTrackingLidar init complete!");

  this->declare_parameter("stateDim", 4);
  this->declare_parameter("measDim", 2);
  this->declare_parameter("ctrlDim", 0);
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("filtered_cloud", "filtered_cloud");

  this->get_parameter("stateDim", stateDim);
  this->get_parameter("measDim", measDim);
  this->get_parameter("ctrlDim", ctrlDim);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("filtered_cloud", filtered_cloud);

  // Store clock
  clock_ = this->get_clock();

  std::cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> sub_callback = std::bind(&MultipleObjectTrackingLidar::cloud_cb,
      this, std::placeholders::_1);
  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        filtered_cloud,
        1,
        sub_callback);
  // Create a ROS publisher for the output point cloud
  pub_cluster0 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_0", 1);
  pub_cluster1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_1", 1);
  pub_cluster2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_2", 1);
  pub_cluster3 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_3", 1);
  pub_cluster4 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_4", 1);
  pub_cluster5 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_5", 1);
  // Subscribe to the clustered pointclouds
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2> c1=this->create_subscription<sensor_msgs::msg::PointCloud2>("ccs", 100, kft);
  objID_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("obj_id", 1);
  /* Point cloud clustering
   */

  // cc_pos=this->create_publisher<std_msgs::msg::Float32MultiArray>("ccs", 100);//clusterCenter1
  //markerPub = this->create_publisher<visualization_msgs::msg::Marker>("viz", 1);

  /* Point cloud clustering
   */
}
// calculate euclidean distance of two points
double MultipleObjectTrackingLidar::euclidean_distance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}
/*
//Count unique object IDs. just to make sure same ID has not been assigned to
two KF_Trackers. int countIDs(vector<int> v)
{
    transform(v.begin(), v.end(), v.begin(), abs); // O(n) where n =
distance(v.end(), v.begin()) sort(v.begin(), v.end()); // Average case O(n log
n), worst case O(n^2) (usually implemented as quicksort.
    // To guarantee worst case O(n log n) replace with make_heap, then
sort_heap.

    // Unique will take a sorted range, and move things around to get duplicated
    // items to the back and returns an iterator to the end of the unique
section of the range auto unique_end = unique(v.begin(), v.end()); // Again n
comparisons return distance(unique_end, v.begin()); // Constant time for random
access iterators (like vector's)
}
*/

/*

objID: vector containing the IDs of the clusters that should be associated with
each KF_Tracker objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/

std::pair<int, int> MultipleObjectTrackingLidar::findIndexOfMin(std::vector<std::vector<float>> distMat) {
  std::cout << "findIndexOfMin CALLED\n";
  std::pair<int, int> minIndex;
  float minEl = std::numeric_limits<float>::max();
  std::cout << "minEl=" << minEl << "\n";
  for (int i = 0; i < distMat.size(); i++)
    for (int j = 0; j < distMat.at(0).size(); j++) {
      if (distMat[i][j] < minEl) {
        minEl = distMat[i][j];
        minIndex = std::make_pair(i, j);
      }
    }
  std::cout << "minIndex=" << minIndex.first << "," << minIndex.second << "\n";
  return minIndex;
}

void MultipleObjectTrackingLidar::kft(const std_msgs::msg::Float32MultiArray ccs) {

  // First predict, to update the internal statePre variable

  std::vector<cv::Mat> pred{KF0.predict(), KF1.predict(), KF2.predict(),
                            KF3.predict(), KF4.predict(), KF5.predict()};
  // std::cout<<"Pred successfull\n";

  // cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
  // std::cout<<"Prediction 1
  // ="<<prediction.at<float>(0)<<","<<prediction.at<float>(1)<<"\n";

  // Get measurements
  // Extract the position of the clusters forom the multiArray. To check if the
  // data coming in, check the .z (every third) coordinate and that will be 0.0
  std::vector<geometry_msgs::msg::Point> clusterCenters; // clusterCenters

  int i = 0;
  for (std::vector<float>::const_iterator it = ccs.data.begin();
       it != ccs.data.end(); it += 3) {
    geometry_msgs::msg::Point pt;
    pt.x = *it;
    pt.y = *(it + 1);
    pt.z = *(it + 2);

    clusterCenters.push_back(pt);
  }

  //  std::cout<<"CLusterCenters Obtained"<<"\n";
  std::vector<geometry_msgs::msg::Point> KFpredictions;
  i = 0;
  for (auto it = pred.begin(); it != pred.end(); it++) {
    geometry_msgs::msg::Point pt;
    pt.x = (*it).at<float>(0);
    pt.y = (*it).at<float>(1);
    pt.z = (*it).at<float>(2);

    KFpredictions.push_back(pt);
  }
  // std::cout<<"Got predictions"<<"\n";

  // Find the cluster that is more probable to be belonging to a given KF.
  objID.clear();   // Clear the objID vector
  objID.resize(6); // Allocate default elements so that [i] doesnt segfault.
                   // Should be done better
  // Copy clusterCentres for modifying it and preventing multiple assignments of
  // the same ID
  std::vector<geometry_msgs::msg::Point> copyOfClusterCenters(clusterCenters);
  std::vector<std::vector<float>> distMat;

  for (int filterN = 0; filterN < 6; filterN++) {
    std::vector<float> distVec;
    for (int n = 0; n < 6; n++) {
      distVec.push_back(
          euclidean_distance(KFpredictions[filterN], copyOfClusterCenters[n]));
    }

    distMat.push_back(distVec);
    /*// Based on distVec instead of distMat (global min). Has problems with the
    person's leg going out of scope int
    ID=std::distance(distVec.begin(),min_element(distVec.begin(),distVec.end()));
     //cout<<"finterlN="<<filterN<<"   minID="<<ID
     objID.push_back(ID);
    // Prevent assignment of the same object ID to multiple clusters
     copyOfClusterCenters[ID].x=100000;// A large value so that this center is
    not assigned to another cluster copyOfClusterCenters[ID].y=10000;
     copyOfClusterCenters[ID].z=10000;
    */
    std::cout << "filterN=" << filterN << "\n";
  }

  std::cout << "distMat.size()" << distMat.size() << "\n";
  std::cout << "distMat[0].size()" << distMat.at(0).size() << "\n";
  // DEBUG: print the distMat
  for (const auto &row : distMat) {
    for (const auto &s : row)
      std::cout << s << ' ';
    std::cout << std::endl;
  }

  for (int clusterCount = 0; clusterCount < 6; clusterCount++) {
    // 1. Find min(distMax)==> (i,j);
    std::pair<int, int> minIndex(findIndexOfMin(distMat));
    std::cout << "Received minIndex=" << minIndex.first << "," << minIndex.second
         << "\n";
    // 2. objID[i]=clusterCenters[j]; counter++
    objID[minIndex.first] = minIndex.second;

    // 3. distMat[i,:]=10000; distMat[:,j]=10000
    distMat[minIndex.first] =
        std::vector<float>(6, 10000.0); // Set the row to a high number.
    for (int row = 0; row < distMat.size();
         row++) // set the column to a high number
    {
      distMat[row][minIndex.second] = 10000.0;
    }
    // 4. if(counter<6) got to 1.
    std::cout << "clusterCount=" << clusterCount << "\n";
  }

  // std::cout<<"Got object IDs"<<"\n";
  // countIDs(objID);// for verif/corner cases

  // display objIDs
  /* DEBUG
    std::cout<<"objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        std::cout<<*it<<" ,";
    std::cout<<"\n";
    */

  //TODO: fix MarkerArray
  //visualization_msgs::MarkerArray clusterMarkers;

  for (int i = 0; i < 6; i++) {
    visualization_msgs::msg::Marker m;

    m.id = i;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.header.frame_id = frame_id;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = i % 2 ? 1 : 0;
    m.color.g = i % 3 ? 1 : 0;
    m.color.b = i % 4 ? 1 : 0;

    // geometry_msgs::msg::Point clusterC(clusterCenters.at(objID[i]));
    geometry_msgs::msg::Point clusterC(KFpredictions[i]);
    m.pose.position.x = clusterC.x;
    m.pose.position.y = clusterC.y;
    m.pose.position.z = clusterC.z;

    //TODO: fix MarkerArray
    //clusterMarkers.markers.push_back(m);
  }

  prevClusterCenters = clusterCenters;

  //TODO: fix MarkerArray
  //markerPub.publish(clusterMarkers);

  std_msgs::msg::Int32MultiArray obj_id;
  for (auto it = objID.begin(); it != objID.end(); it++)
    obj_id.data.push_back(*it);
  // Publish the object IDs
  objID_pub->publish(obj_id);
  // convert clusterCenters from geometry_msgs::msg::Point to floats
  std::vector<std::vector<float>> cc;
  for (int i = 0; i < 6; i++) {
    std::vector<float> pt;
    pt.push_back(clusterCenters[objID[i]].x);
    pt.push_back(clusterCenters[objID[i]].y);
    pt.push_back(clusterCenters[objID[i]].z);

    cc.push_back(pt);
  }
  // std::cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";
  float meas0[2] = {cc[0].at(0), cc[0].at(1)};
  float meas1[2] = {cc[1].at(0), cc[1].at(1)};
  float meas2[2] = {cc[2].at(0), cc[2].at(1)};
  float meas3[2] = {cc[3].at(0), cc[3].at(1)};
  float meas4[2] = {cc[4].at(0), cc[4].at(1)};
  float meas5[2] = {cc[5].at(0), cc[5].at(1)};

  // The update phase
  cv::Mat meas0Mat = cv::Mat(2, 1, CV_32F, meas0);
  cv::Mat meas1Mat = cv::Mat(2, 1, CV_32F, meas1);
  cv::Mat meas2Mat = cv::Mat(2, 1, CV_32F, meas2);
  cv::Mat meas3Mat = cv::Mat(2, 1, CV_32F, meas3);
  cv::Mat meas4Mat = cv::Mat(2, 1, CV_32F, meas4);
  cv::Mat meas5Mat = cv::Mat(2, 1, CV_32F, meas5);

  // std::cout<<"meas0Mat"<<meas0Mat<<"\n";
  if (!(meas0Mat.at<float>(0, 0) == 0.0f || meas0Mat.at<float>(1, 0) == 0.0f))
    cv::Mat estimated0 = KF0.correct(meas0Mat);
  if (!(meas1[0] == 0.0f || meas1[1] == 0.0f))
    cv::Mat estimated1 = KF1.correct(meas1Mat);
  if (!(meas2[0] == 0.0f || meas2[1] == 0.0f))
    cv::Mat estimated2 = KF2.correct(meas2Mat);
  if (!(meas3[0] == 0.0f || meas3[1] == 0.0f))
    cv::Mat estimated3 = KF3.correct(meas3Mat);
  if (!(meas4[0] == 0.0f || meas4[1] == 0.0f))
    cv::Mat estimated4 = KF4.correct(meas4Mat);
  if (!(meas5[0] == 0.0f || meas5[1] == 0.0f))
    cv::Mat estimated5 = KF5.correct(meas5Mat);

  // Publish the point clouds belonging to each clusters

  // std::cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
  // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
  // std::cout<<"DONE KF_TRACKER\n";
}

void MultipleObjectTrackingLidar::publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  auto clustermsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cluster, *clustermsg);
  clustermsg->header.frame_id = frame_id;
  clustermsg->header.stamp = clock_->now();
  pub->publish(*clustermsg);
}

void MultipleObjectTrackingLidar::cloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &input)
{
  // std::cout<<"IF firstFrame="<<firstFrame<<"\n";
  // If this is the first frame, initialize kalman filters for the clustered
  // objects
  if (firstFrame) {
    // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
    // Could be made generic by creating a Kalman Filter only when a new object
    // is detected

    float dvx = 0.01f; // 1.0
    float dvy = 0.01f; // 1.0
    float dx = 1.0f;
    float dy = 1.0f;
    KF0.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);
    KF1.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);
    KF2.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);
    KF3.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);
    KF4.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);
    KF5.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0,
                            dvx, 0, 0, 0, 0, dvy);

    cv::setIdentity(KF0.measurementMatrix);
    cv::setIdentity(KF1.measurementMatrix);
    cv::setIdentity(KF2.measurementMatrix);
    cv::setIdentity(KF3.measurementMatrix);
    cv::setIdentity(KF4.measurementMatrix);
    cv::setIdentity(KF5.measurementMatrix);
    // Process Noise Covariance Matrix Q
    // [ Ex 0  0    0 0    0 ]
    // [ 0  Ey 0    0 0    0 ]
    // [ 0  0  Ev_x 0 0    0 ]
    // [ 0  0  0    1 Ev_y 0 ]
    //// [ 0  0  0    0 1    Ew ]
    //// [ 0  0  0    0 0    Eh ]
    float sigmaP = 0.01;
    float sigmaQ = 0.1;
    setIdentity(KF0.processNoiseCov, cv::Scalar::all(sigmaP));
    setIdentity(KF1.processNoiseCov, cv::Scalar::all(sigmaP));
    setIdentity(KF2.processNoiseCov, cv::Scalar::all(sigmaP));
    setIdentity(KF3.processNoiseCov, cv::Scalar::all(sigmaP));
    setIdentity(KF4.processNoiseCov, cv::Scalar::all(sigmaP));
    setIdentity(KF5.processNoiseCov, cv::Scalar::all(sigmaP));
    // Meas noise cov matrix R
    cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(sigmaQ)); // 1e-1
    cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(sigmaQ));

    // Process the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      float x = 0.0;
      float y = 0.0;
      int numPts = 0;
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);
        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        numPts++;

        // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
        //                                          origin);
        // mindist_this_cluster = std::min(dist_this_point,
        // mindist_this_cluster);
      }

      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;

      cluster_vec.push_back(cloud_cluster);

      // Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
    }

    // Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < 6) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      clusterCentroids.push_back(centroid);
    }

    // Set initial state
    KF0.statePre.at<float>(0) = clusterCentroids.at(0).x;
    KF0.statePre.at<float>(1) = clusterCentroids.at(0).y;
    KF0.statePre.at<float>(2) = 0; // initial v_x
    KF0.statePre.at<float>(3) = 0; // initial v_y

    // Set initial state
    KF1.statePre.at<float>(0) = clusterCentroids.at(1).x;
    KF1.statePre.at<float>(1) = clusterCentroids.at(1).y;
    KF1.statePre.at<float>(2) = 0; // initial v_x
    KF1.statePre.at<float>(3) = 0; // initial v_y

    // Set initial state
    KF2.statePre.at<float>(0) = clusterCentroids.at(2).x;
    KF2.statePre.at<float>(1) = clusterCentroids.at(2).y;
    KF2.statePre.at<float>(2) = 0; // initial v_x
    KF2.statePre.at<float>(3) = 0; // initial v_y

    // Set initial state
    KF3.statePre.at<float>(0) = clusterCentroids.at(3).x;
    KF3.statePre.at<float>(1) = clusterCentroids.at(3).y;
    KF3.statePre.at<float>(2) = 0; // initial v_x
    KF3.statePre.at<float>(3) = 0; // initial v_y

    // Set initial state
    KF4.statePre.at<float>(0) = clusterCentroids.at(4).x;
    KF4.statePre.at<float>(1) = clusterCentroids.at(4).y;
    KF4.statePre.at<float>(2) = 0; // initial v_x
    KF4.statePre.at<float>(3) = 0; // initial v_y

    // Set initial state
    KF5.statePre.at<float>(0) = clusterCentroids.at(5).x;
    KF5.statePre.at<float>(1) = clusterCentroids.at(5).y;
    KF5.statePre.at<float>(2) = 0; // initial v_x
    KF5.statePre.at<float>(3) = 0; // initial v_y

    firstFrame = false;

    for (int i = 0; i < 6; i++) {
      geometry_msgs::msg::Point pt;
      pt.x = clusterCentroids.at(i).x;
      pt.y = clusterCentroids.at(i).y;
      prevClusterCenters.push_back(pt);
    }
    /*  // Print the initial state of the kalman filter for debugging
      std::cout<<"KF0.satePre="<<KF0.statePre.at<float>(0)<<","<<KF0.statePre.at<float>(1)<<"\n";
      std::cout<<"KF1.satePre="<<KF1.statePre.at<float>(0)<<","<<KF1.statePre.at<float>(1)<<"\n";
      std::cout<<"KF2.satePre="<<KF2.statePre.at<float>(0)<<","<<KF2.statePre.at<float>(1)<<"\n";
      std::cout<<"KF3.satePre="<<KF3.statePre.at<float>(0)<<","<<KF3.statePre.at<float>(1)<<"\n";
      std::cout<<"KF4.satePre="<<KF4.statePre.at<float>(0)<<","<<KF4.statePre.at<float>(1)<<"\n";
      std::cout<<"KF5.satePre="<<KF5.statePre.at<float>(0)<<","<<KF5.statePre.at<float>(1)<<"\n";

      //cin.ignore();// To be able to see the printed initial state of the
      KalmanFilter
      */
  }

  else {
    // std::cout<<"ELSE firstFrame="<<firstFrame<<"\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    /* Creating the KdTree from input point cloud*/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);

    tree->setInputCloud(input_cloud);

    /* Here we are creating a vector of PointIndices, which contains the actual
     * index information in a vector<int>. The indices of each detected cluster
     * are saved here. Cluster_indices is a vector containing one instance of
     * PointIndices for each detected cluster. Cluster_indices[0] contain all
     * indices of the first cluster in input point cloud.
     */
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    // std::cout<<"PCL init successfull\n";
    /* Extract the clusters out of pc and save indices in cluster_indices.*/
    ec.extract(cluster_indices);
    // std::cout<<"PCL extract successfull\n";
    /* To separate each cluster out of the vector<PointIndices> we have to
     * iterate through cluster_indices, create a new PointCloud for each
     * entry and write all points of the current cluster in the PointCloud.
     */
    // pcl::PointXYZ origin (0,0,0);
    // float mindist_this_cluster = 1000;
    // float dist_this_point = 1000;

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    // Vector of cluster pointclouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

    // Cluster centroids
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      float x = 0.0;
      float y = 0.0;
      int numPts = 0;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {

        cloud_cluster->points.push_back(input_cloud->points[*pit]);

        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        numPts++;

        // dist_this_point = pcl::geometry::distance(input_cloud->points[*pit],
        //                                          origin);
        // mindist_this_cluster = std::min(dist_this_point,
        // mindist_this_cluster);
      }

      pcl::PointXYZ centroid;
      centroid.x = x / numPts;
      centroid.y = y / numPts;
      centroid.z = 0.0;

      cluster_vec.push_back(cloud_cluster);

      // Get the centroid of the cluster
      clusterCentroids.push_back(centroid);
    }
    // std::cout<<"cluster_vec got some clusters\n";

    // Ensure at least 6 clusters exist to publish (later clusters may be empty)
    while (cluster_vec.size() < 6) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(
          new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < 6) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;

      clusterCentroids.push_back(centroid);
    }

    std_msgs::msg::Float32MultiArray cc;
    for (int i = 0; i < 6; i++) {
      cc.data.push_back(clusterCentroids.at(i).x);
      cc.data.push_back(clusterCentroids.at(i).y);
      cc.data.push_back(clusterCentroids.at(i).z);
    }
    // std::cout<<"6 clusters initialized\n";

    // cc_pos.publish(cc);// Publish cluster mid-points.
    kft(cc);
    int i = 0;
    bool publishedCluster[6];
    for (auto it = objID.begin(); it != objID.end();
         it++) { // std::cout<<"Inside the for loop\n";

      switch (i) {
        std::cout << "Inside the switch case\n";
      case 0: {
        publish_cloud(pub_cluster0, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 1: {
        publish_cloud(pub_cluster1, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 2: {
        publish_cloud(pub_cluster2, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 3: {
        publish_cloud(pub_cluster3, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 4: {
        publish_cloud(pub_cluster4, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }

      case 5: {
        publish_cloud(pub_cluster5, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      default:
        break;
      }
    }
  }
}

}
