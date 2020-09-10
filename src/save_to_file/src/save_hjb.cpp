#include<iostream>
#include<ros/ros.h>
#include <tf/transform_listener.h>


#include<signal.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include<std_msgs/String.h>






std::vector<double> current_jts;
std::vector<double> current_pose;
using namespace std;
void signal_callback_handler(int signum)
{
    cout << "Caught Signal" << signum << endl;

    exit(0);
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    current_jts.clear();

    for(int i=0;i<msg->position.size();i++)
    {

        current_jts.push_back(msg->position[i]);
    }


    std::cout<<current_jts[0]<< "," << current_jts[1] << "," << current_jts[2]<< "," <<current_jts[3]<< "," << current_jts[4] << "," << current_jts[5]<< "," <<  std::endl;


    return;
}

void waitForEnter() {
    static std::string line;
    std::getline(std::cin, line);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"save_data");
    ros::NodeHandle nh;



    std::ofstream myfile_pose_base,myfile_pose_world,myfile_joint;
    std::ofstream myfile_orientation;

    myfile_pose_base.open ("/home/ravi/ral_ws/src/save_to_file/datac/record_pose_base.txt");
    myfile_pose_world.open ("/home/ravi/ral_ws/src/save_to_file/datac/circle.txt");
    myfile_orientation.open("/home/ravi/ral_ws/src/save_to_file/orientation.txt");

    myfile_joint.open ("/home/ravi/ral_ws/src/save_to_file/record_jts.txt");

    tf::TransformListener transform_listener;

    transform_listener.waitForTransform("/world", "/ee_link", ros::Time(0), ros::Duration(2));

    ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);
    std::cout<<"Press [Enter] to start displaying joint positions and pose \n"<<std::endl;
    int i;
    for(i=0; i<2000;i++)
    {
        //        std::cout<<" --------------------"<<i<<"-----------------------------\n";

        signal(SIGINT, signal_callback_handler);

        ros::spinOnce();
        //            tf::StampedTransform transform_ee_base;
        tf::StampedTransform transform_ee_world;



        //            transform_listener.waitForTransform("/base", "/ee_link", ros::Time(0), ros::Duration(1));
        //            transform_listener.lookupTransform("/base" , "/ee_link", ros::Time(0), transform_ee_base);
        usleep(40000);
        transform_listener.lookupTransform("/world" , "/ee_link", ros::Time(0), transform_ee_world);

        //            std::cout<<"base : "<<transform_ee_base.getOrigin().x()<<","<<transform_ee_base.getOrigin().y()<<","<<transform_ee_base.getOrigin().z()<<","<<std::endl;
        std::cout<<"world : "<<transform_ee_world.getOrigin().x()<<","<<transform_ee_world.getOrigin().y()<<","<<transform_ee_world.getOrigin().z()<<","<<std::endl;




        //            myfile_pose_base<<transform_ee_base.getOrigin().x()<< "," << transform_ee_base.getOrigin().y()<< "," << transform_ee_base.getOrigin().z()<< "," << std::endl;
        myfile_pose_world<<transform_ee_world.getOrigin().x()<< "," << transform_ee_world.getOrigin().y()<< "," << transform_ee_world.getOrigin().z()<< "," << std::endl;
        //myfile_orientation<<transform_ee_world.getRotation().getX()<<","<<transform_ee_world.getRotation().getY()<<","
        //                 <<transform_ee_world.getRotation().getZ()<<","
        //                <<transform_ee_world.getRotation().getW()<<","<< std::endl;


        //            myfile_joint <<current_jts[0]<< "," << current_jts[1] << "," << current_jts[2]<< "," <<current_jts[3]<< "," << current_jts[4] << "," << current_jts[5]<< "," <<  std::endl;

        //            std::cout <<current_pose[0]<< "\t" << current_pose[1] << "\t" << current_pose[2] <<'\n';

    }

    //    myfile_pose_base.close();
    myfile_pose_world.close();

    //    myfile_joint.close();

    //        ros::MultiThreadedSpinner spinner(4);

    //        spinner.spin();


    return 0;
}
