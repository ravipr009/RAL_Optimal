/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <signal.h>
#include<visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>
#include <sstream>
#define NO_GOALS 12
#define NO_OBS 1
#define NO_STARTS 1

void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << "\n";

    exit(0);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    signal(SIGINT, signal_callback_handler);

    const char* file_name_goal = "/home/ravi/ral_ws/src/save_to_file/data/goal.txt";
    const char* file_name_obs = "/home/ravi/ral_ws/src/save_to_file/data/obs.txt";
    const char* file_name_start= "/home/ravi/ral_ws/src/save_to_file/data/start.txt";


    Eigen::MatrixXd goal(NO_GOALS,3);

    std::ifstream file_goal;

    file_goal.open(file_name_goal,std::ios_base::in);

    for(int i=0;i<NO_GOALS;i++)
        for(int j=0;j<3;j++)
        {
            std::string line;
            getline(file_goal, line, ',' );
            goal(i,j) = atof(line.c_str());

        }
    file_goal.close();

    Eigen::MatrixXd obs(NO_OBS,7);

    std::ifstream file_obs;

    file_obs.open(file_name_obs,std::ios_base::in);

    for(int i=0;i<NO_OBS;i++)
        for(int j=0;j<7;j++)
        {
            std::string line;
            getline(file_obs, line, ',' );
            obs(i,j) = atof(line.c_str());

        }
    file_obs.close();

    Eigen::MatrixXd start(NO_STARTS,3);

    std::ifstream file_start;

    file_start.open(file_name_start,std::ios_base::in);

    for(int i=0;i<NO_STARTS;i++)
        for(int j=0;j<3;j++)
        {
            std::string line;
            getline(file_start, line, ',' );
            start(i,j) = atof(line.c_str());

        }
    file_start.close();

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(2.0);


    moveit::planning_interface::MoveGroup group("manipulator");


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    ros::Publisher pub_traj = node_handle.advertise<visualization_msgs::Marker>("/traj_n1", 1000);


    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    group.setPlannerId("RRTConnectkConfigDefault");


    std::vector<double> startj1{-1.44178,-1.1341,-2.42841,-1.16632,1.60212,0.299298};
    std::vector<double> startj2{0.0955179,-1.25224,-2.45897,-1.00729,1.60484,0.593181};




int goal_index=0;
    geometry_msgs::Pose target_pose;

    for (int s=0;s<NO_STARTS;s++)
    {
        for (int iter=0; iter<NO_GOALS;iter++)
        {





            visualization_msgs::Marker points;
            points.header.frame_id ="/world";
            points.header.stamp = ros::Time::now();
            points.ns ="points_and_lines";
            points.action =  visualization_msgs::Marker::ADD;
            points.pose.orientation.w =1.0;



            points.id = 0;

            points.type = visualization_msgs::Marker::POINTS;

            // POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.05;
            points.scale.y = 0.05;


            // Points are green
            points.color.b = 1.0f;
            points.color.a = 1.0;


            // Create the vertices for the points and lines



            geometry_msgs::Point m;
            m.x = goal(goal_index,0);
            m.y = goal(goal_index,1);
            m.z = goal(goal_index,2);

            points.points.push_back(m);



            while (pub_traj.getNumSubscribers() < 1)
            {
                if (!ros::ok())
                {
                    return 0;
                }
                ROS_WARN_ONCE("Please create a subscriber to the marker");
                sleep(1);
            }
            pub_traj.publish(points);
            group.setStartState(*group.getCurrentState());



            if(s==0){
                std::cout<<"MOving to start : "<<s<<"\n";

            group.setJointValueTarget(startj1);}

            if(s==1){                std::cout<<"MOving to start : "<<s<<"\n";
             group.setJointValueTarget(startj2);}



            moveit::planning_interface::MoveGroup::Plan joint_plan;


            bool success = group.plan(joint_plan);

            if(success)
            {
                char pp;
                pp ='p';

                ROS_INFO ("Press pp to run");
                std::cin>>pp;
                sleep(1);
                group.execute(joint_plan);
            }


            sleep(4);


std::cout<<"GOAL ID :"<<goal_index<<"\n";
            geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
            target_pose.position.x=goal(goal_index,0);
            target_pose.position.y=goal(goal_index,1);

            target_pose.position.z=goal(goal_index,2);

            target_pose.orientation=current_pose.pose.orientation;
            //    target_pose.orientation.w = 1.0;



            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(3);


            primitive.dimensions[0] = 0.05;

            std::vector<moveit_msgs::CollisionObject> collision_objects;


            ///////////////////////////////////// OBS1/////////////////////////////////////////////////////
            moveit_msgs::CollisionObject collision_object1;
            collision_object1.header.frame_id = group.getPlanningFrame();

            collision_object1.id = "sphere1";

            geometry_msgs::Pose sphere_pose1;
            sphere_pose1.position.x = (goal(goal_index,0)+start(s,0))/2;
            sphere_pose1.position.y = (goal(goal_index,1)+start(s,1))/2;
            sphere_pose1.position.z = (goal(goal_index,2)+start(s,2))/2;
            sphere_pose1.orientation.w = 1.0;

            collision_object1.primitives.push_back(primitive);
            collision_object1.primitive_poses.push_back(sphere_pose1);
            collision_object1.operation = collision_object1.ADD;

            collision_objects.push_back(collision_object1);

            // Now, let's add the collision object into the world
            ROS_INFO("Add an object into the world");
            planning_scene_interface.addCollisionObjects(collision_objects);

            /* Sleep so we have time to see the object in RViz */
            sleep(2.0);

            moveit::planning_interface::MoveGroup::Plan my_plan;

group.setMaxVelocityScalingFactor(0.1);
            // Planning with collision detection can be slow.  Lets set the planning time
            // to be sure the planner has enough time to plan around the box.  10 seconds
            // should be plenty.
            group.setPlanningTime(20.0);


            // Now when we plan a trajectory it will avoid the obstacle
            group.setStartState(*group.getCurrentState());

//                group.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
            group.setPoseTarget(target_pose);
            success = group.plan(my_plan);

            ROS_INFO("Visualizing plan (pose goal move around box) %s",success?"":"FAILED");
            /* Sleep to give Rviz time to visualize the plan. */
            sleep(4.0);
            char p = 'p';
            ROS_INFO("ENTER TO MOVE");
            std::cin>>p;
            group.move();

            std::vector<std::string> object_ids;
            object_ids.push_back(collision_object1.id);
            planning_scene_interface.removeCollisionObjects(object_ids);
            /* Sleep to give Rviz time to show the object is no longer there. */
            sleep(4.0);
            goal_index++;
        }
    }



    // END_TUTORIAL

    ros::shutdown();
    return 0;
}
