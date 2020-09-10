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
#define NO_GOALS 25

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

    const char* file_name_goal = "/home/ravi/ral_ws/src/brick_wall/src/goal.txt";
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

    /* This sleep is ONLY to allow Rviz to come up */
    sleep(2.0);

    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name
    // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroup group("manipulator");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // (Optional) Create a publisher for visualizing plans in Rviz.
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::Publisher pub_traj = node_handle.advertise<visualization_msgs::Marker>("/traj", 1000);

    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    group.setPlannerId("RRTConnectkConfigDefault");
    geometry_msgs::Pose target_pose;
    std::vector<double> j14{ -0.05195743242372686, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};

    group.setJointValueTarget(j14);
    moveit::planning_interface::MoveGroup::Plan joint_plan;


    bool success = group.plan(joint_plan);

    if(success)
    {


        sleep(1);

        group.execute(joint_plan);


    }
    int k=0;
    for (int i=0;i<2*NO_GOALS;i++)
    {


        if (i%2 == 0)
        {
            target_pose.position.x = goal(12,0);
            target_pose.position.y = goal(12,1);
            target_pose.position.z = goal(12,2);
        }
        else
        {
            ROS_INFO("GOAL NEXT ");
            target_pose.position.x = goal(k,0);
            target_pose.position.y = goal(k,1);
            target_pose.position.z = goal(k,2);
            k++;
        }




    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
    target_pose.orientation=current_pose.pose.orientation;
    //    target_pose.orientation.w = 1.0;

    moveit::planning_interface::MoveGroup::Plan my_plan;


    // Planning with collision detection can be slow.  Lets set the planning time
    // to be sure the planner has enough time to plan around the box.  10 seconds
    // should be plenty.
    group.setPlanningTime(10.0);


    // Now when we plan a trajectory it will avoid the obstacle
    group.setStartState(*group.getCurrentState());

    group.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
//    group.setPoseTarget(target_pose);
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan (pose goal move around box) %s",
             success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(4.0);
    char p = 'p';
    ROS_INFO("ENTER TO MOVE");
    std::cin>>p;
    group.move();
 }



// END_TUTORIAL

ros::shutdown();
return 0;
}
