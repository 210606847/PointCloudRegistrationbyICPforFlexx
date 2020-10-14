#include <vector>
#include <iostream>
#include <string>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Core>
#include <pcl/common/transforms.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
float voxelgridsize=0.01;

PointCloudT::Ptr cloud_src (new PointCloudT);
PointCloudT::Ptr cloud_tgt (new PointCloudT);

PointCloudT::Ptr save (new PointCloudT);
PointCloudT::Ptr save1 (new PointCloudT);
PointCloudT::Ptr save2 (new PointCloudT);

float timeFlexxPointCloudLast=0;
bool newFlexxPointCloudLast = false;
int counter=0;
float zmin=0;
float zmax=0;
struct pointscoor{
  float x;
  float y;
  float z;
};
bool cmp(float a,float b){
  return a<b;
}

PointCloudT::Ptr FlexxPointCloudLast (new PointCloudT);

PointCloudT::Ptr read_cloud_point (std::string const &file_path){
    PointCloudT::Ptr cloud (new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(file_path,*cloud)==-1){
        std::cout<<"Couldn't read the pcd file"<<std::endl;
        return nullptr;
    }
    return cloud;
}

void FlexxPointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& FlexxPointCloudLast2)
{
    timeFlexxPointCloudLast = FlexxPointCloudLast2->header.stamp.toSec();
    FlexxPointCloudLast->clear();
    pcl::fromROSMsg(*FlexxPointCloudLast2, *FlexxPointCloudLast);
    newFlexxPointCloudLast = true;
}

void visualize_pcd(PointCloudT::Ptr pcd_src,PointCloudT::Ptr pcd_tgt,PointCloudT::Ptr pcd_final)
{
   //int vp_1, vp_2;
   // Create a PCLVisualizer object
   pcl::visualization::PCLVisualizer viewer("registration Viewer");
   //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
   pcl::visualization::PointCloudColorHandlerCustom<PointT> src_h (pcd_src, 0, 255, 0);
   pcl::visualization::PointCloudColorHandlerCustom<PointT> tgt_h (pcd_tgt, 255, 0, 0);
   pcl::visualization::PointCloudColorHandlerCustom<PointT> final_h (pcd_final, 0, 0, 255);
   viewer.addPointCloud (pcd_src, src_h, "source cloud");
   viewer.addPointCloud (pcd_tgt, tgt_h, "tgt cloud");
   viewer.addPointCloud (pcd_final, final_h, "final cloud");
   //viewer.addCoordinateSystem(1.0);
   while (!viewer.wasStopped())
   {
       viewer.spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloudmatch");
    ros::NodeHandle nh;

    std::string FilePath;
    ros::param::get("~filepath",FilePath);
    std::cout <<FilePath.c_str()<<std::endl;
    double voxelgridsize;
    ros::param::get("~voxelgridsize",voxelgridsize);

    cloud_tgt=read_cloud_point(FilePath);
    int sourcesize=cloud_tgt->size();
    std::cout << "Loaded " << sourcesize <<" data points from cloud_tgt.pcd" << std::endl;
    //去除目标点云中的NAN点
    std::vector<int> indices_src;
    pcl::removeNaNFromPointCloud(*cloud_tgt,*cloud_tgt,indices_src);
    std::cout <<"After removed NAN from pointcloud: "<<cloud_tgt->size()<< std::endl;
    //目标点云下采样滤波
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(voxelgridsize,voxelgridsize,voxelgridsize);//
    voxel_grid.setInputCloud(cloud_tgt);
    PointCloudT::Ptr cloud_tgt_voxelgrid (new PointCloudT);
    voxel_grid.filter(*cloud_tgt_voxelgrid);
    std::cout<<"down size *cloud_in_surf from "<<cloud_tgt->size()<<" to "<<cloud_tgt_voxelgrid->size()<<endl;
    //
    int cloudsize1=cloud_tgt_voxelgrid->size();
	double cx1=0,cy1=0,cz1=0,cx2=0,cy2=0,cz2=0;
	for(int i=0; i<cloudsize1; i++){
		cx1 += cloud_tgt_voxelgrid->points[i].x;
		cy1 += cloud_tgt_voxelgrid->points[i].y;
		cz1 += cloud_tgt_voxelgrid->points[i].z;
	}
	cx1 /=cloudsize1;
	cy1 /=cloudsize1;
	cz1 /=cloudsize1;
    float trans[3]={0};
    //
    ros::Subscriber subFlexxPointCloud = nh.subscribe<sensor_msgs::PointCloud2>
        ("/royale_camera_driver/point_cloud", 10, FlexxPointCloudHandler);
    ros::Publisher pubFlexxPointCloudAfterfilted = nh.advertise<sensor_msgs::PointCloud2>
        ("/Filted",10);
    ros::Publisher pubFlexxPointCloudAfterMatched = nh.advertise<sensor_msgs::PointCloud2>
        ("/Matched",10);
    ros::Publisher pubShipModelPointCloud = nh.advertise<sensor_msgs::PointCloud2>
        ("/ShipModelPointCloud",10);

    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();

        if (newFlexxPointCloudLast){
            clock_t start=clock();
            newFlexxPointCloudLast = false;
            //截取船模部分有效点云，使用条件滤波
            //pointscoor coor;
            std::vector<float> data;
            for (int i=0;i<FlexxPointCloudLast->size();i++){
                data.push_back(FlexxPointCloudLast->points[i].z);
            }
            std::sort(data.begin(),data.end(),cmp);
            float zmin = data[100]-0.1;
	        float zmax = data[100]+0.1;
            std::cout << FlexxPointCloudLast->size()<<std::endl;
            std::cout << data[100]<<std::endl;
            pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>()); 
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr 
                (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::GT,zmin)));
            range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr 
                (new pcl::FieldComparison<pcl::PointXYZI>("z",pcl::ComparisonOps::LT,zmax)));
            pcl::ConditionalRemoval<pcl::PointXYZI> range_filt;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filt (new pcl::PointCloud<pcl::PointXYZI>());
            range_filt.setCondition (range_cond);
            range_filt.setInputCloud(FlexxPointCloudLast);
            range_filt.setKeepOrganized(false);//直接删除z<=0.1的点，若为true则将这些点重置为0，0，0仍然保存索引
            range_filt.filter(*cloud_filt);
            std::cout <<"CloudSize Filt form: "<<FlexxPointCloudLast->size()<<" to "<<cloud_filt->size()<<std::endl;
            //去除NAN点
            //std::vector<int> indices_src1;
            //clock_t start1=clock();

            //源点云下采样滤波
            pcl::VoxelGrid<PointT> voxel_grid1;
            voxel_grid1.setLeafSize(voxelgridsize,voxelgridsize,voxelgridsize);//200
            voxel_grid1.setInputCloud(cloud_filt);
            PointCloudT::Ptr cloud_src_voxelgrid (new PointCloudT);
            voxel_grid1.filter(*cloud_src_voxelgrid);
            std::cout<<"down size *cloud_filt from "<<cloud_filt->size()<<" to "<<cloud_src_voxelgrid->size()<<endl;
            //
            int cloudsize2=cloud_src_voxelgrid->size();
            for(int i=0; i<cloudsize2; i++){
                cx2 += cloud_src_voxelgrid->points[i].x;
                cy2 += cloud_src_voxelgrid->points[i].y;
                cz2 += cloud_src_voxelgrid->points[i].z;
            }
            cx2 /=cloudsize2;
            cy2 /=cloudsize2;
            cz2 /=cloudsize2;
            trans[0]=cx1-cx2;
            trans[1]=cy1-cy2;
            trans[2]=cz1-cz2;
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block<3,1>(0,3) = Eigen::Vector3f(trans[0],trans[1],trans[2]);
            std::cout<< T <<std::endl;
            pcl::transformPointCloud(*cloud_src_voxelgrid,*cloud_src_voxelgrid,T);
            //icp匹配
            PointCloudT::Ptr icp_result (new PointCloudT);
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setInputSource(cloud_src_voxelgrid);
            icp.setInputTarget(cloud_tgt_voxelgrid);
            icp.setMaximumIterations (50);
            icp.setTransformationEpsilon (0.001);
            icp.align(*icp_result);
            std::cout << "ICP has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
            Eigen::Matrix4f icp_trans;
            icp_trans=icp.getFinalTransformation();
            std::cout << icp_trans << std::endl;
            pcl::transformPointCloud(*cloud_filt, *icp_result, icp_trans);

            clock_t end=clock();
            std::cout<<"icp time: "<<(double)(end-start)/(double)CLOCKS_PER_SEC<<std::endl;
               // " 去除NAN点: "<<(double)(start1-start)/(double)CLOCKS_PER_SEC<<
               // " 下采样滤波: "<<(double)(start2-start1)/(double)CLOCKS_PER_SEC<<
               // " icp: "<<(double)(end-start2)/(double)CLOCKS_PER_SEC<<std::endl;
            
            //visualize_pcd(cloud_in_surf1,icp_result,cloud_in_surf);

            sensor_msgs::PointCloud2 FlexxPointCloudAfterfilted;
            pcl::toROSMsg(*cloud_filt,FlexxPointCloudAfterfilted);
            FlexxPointCloudAfterfilted.header.stamp = ros::Time().fromSec(timeFlexxPointCloudLast);
            FlexxPointCloudAfterfilted.header.frame_id = "/ship";
            pubFlexxPointCloudAfterfilted.publish(FlexxPointCloudAfterfilted);

            sensor_msgs::PointCloud2 FlexxPointCloudAfterMatched;
            pcl::toROSMsg(*icp_result,FlexxPointCloudAfterMatched);
            FlexxPointCloudAfterMatched.header.stamp = ros::Time().fromSec(timeFlexxPointCloudLast);
            FlexxPointCloudAfterMatched.header.frame_id = "/ship";
            pubFlexxPointCloudAfterMatched.publish(FlexxPointCloudAfterfilted);

            sensor_msgs::PointCloud2 ShipModelPointCloud;
            pcl::toROSMsg(*cloud_tgt,ShipModelPointCloud);
            ShipModelPointCloud.header.stamp = ros::Time().fromSec(timeFlexxPointCloudLast);
            ShipModelPointCloud.header.frame_id = "/ship";
            pubShipModelPointCloud.publish(ShipModelPointCloud);

            //*save=(*cloud_filt);
            //*save1=(*icp_result);
            //*save2=(*cloud_tgt);

        }

        rate.sleep();
    }
    //visualize_pcd(save2,save1,save);


    return 0;
}
