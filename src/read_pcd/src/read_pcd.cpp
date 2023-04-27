#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <string>     
#include <iostream>
#include <fstream>
using namespace std;
 
sensor_msgs::PointCloud2 read_pcd_path(string path, string num)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  sensor_msgs::PointCloud2 output;
  //string path_final = path + to_string(num) + ".pcd";
  string path_final = path + num + ".pcd";
  cout << "Read file:" << path_final << endl; 
  pcl::io::loadPCDFile (path_final, cloud); //修改自己pcd文件所在路径
  //Convert the cloud to ROS message
  
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "rslidar";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer    
//！！！这一步需要注意，是后面rviz的 fixed_frame  !!!敲黑板，画重点。
  return output;
}

bool find_file_under_dir(const string &AbsFilePath,vector<string> &FileName)
{
	DIR *dir;
	struct dirent *ptr;
	if(!(dir = opendir(AbsFilePath.c_str())))
	{
		cout << "current dir isn't exit" << endl;
		return false;
	}
	while((ptr = readdir(dir)) != 0)
	{
		if(strcmp(ptr->d_name,".") == 0 || strcmp(ptr->d_name,"..") == 0)
		{
			continue;
		}
		FileName.push_back(string(ptr->d_name));
	}
	closedir(dir);
	return true;
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "UandBdetect");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 10);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  sensor_msgs::PointCloud2 output;
  //string path = "/home/hao/catkin_ws/pcd/";
  string path;
  ros::param::get("pcd_path", path);
  cout<<"work,"<<path<<endl;
  string bagname = "/home/hao/catkin_ws/4-26.bag";
  //string bagname;
  //ros::param::get("bag_name", bagname);

  ofstream fileout(bagname,ios::trunc);//ios::trunc是清除原文件内容,可不写,默认就是它
  if(!fileout){
      cout << "Create file failure...\n";
      exit(0);
  }
  fileout.close();

  rosbag::Bag bag;
  double time = ros::Time::now().toSec();
  bag.open(bagname,rosbag::bagmode::Write);
  vector<string> FileName;
  if(find_file_under_dir("/home/hao/catkin_ws/pcd",FileName))
  {
   	cout<<"loading pcd ..."<<endl;
	  cout<<"pcd num is:"<<FileName.size()<<endl;
  }

  for (float num = 0 ; num <= FileName.size()-1 ; num++)
  {
    //cout<<FileName[num]<<endl;
    int slice_len=4;
    int len = FileName[num].length();
    int start_pos = len - slice_len;
    string filename = FileName[num].substr(0,start_pos);
    //cout<<filename<<endl;
    output = read_pcd_path(path,filename);
    output.header.frame_id = "rslidar" ;
    //output.header.stamp = ros::Time().fromSec(float(num/10)+time);
    float timestamp = std::stof(filename);
    ros::Time times = ros::Time(timestamp);
    output.header.stamp = times;
    // cout << "scan time:" << float(num/10)+time << endl; 
    cout << "scan time:" << output.header.stamp << endl; 
    bag.write("/rslidar_points",output.header.stamp,output);
  }
  bag.close();
  cout<<"Bag file write finish"<<endl;

  // ros::Rate loop_rate(1);
  // while (ros::ok())
  // {
  //   pcl_pub.publish(output);
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  return 0;
}


