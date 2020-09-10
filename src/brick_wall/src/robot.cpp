#include <iostream>
#include <brick_wall/brick_wall_demo.h>
#include <pthread.h>



robot::robot():
    nh_("~"),
    planning_group_name_("manipulator"),
    planner_name_("RRTConnectkConfigDefault"),
    POS_TOLARENCE(0.05),
    ANG_TOLARENCE(0.2),
    PLANING_TIME(20.0)
{
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlannerId(planner_name_);
    move_group_->setPlanningTime(PLANING_TIME);
    move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
    move_group_->setGoalPositionTolerance(POS_TOLARENCE);
    //    get_pose_client = nh_.serviceClient<all_services::Pose_of_cube2>("/Pose_of_cube2");
    pose_of_cube_pub = nh_.advertise<geometry_msgs::PoseStamped>("pose_of_cube_topic",100);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("normal_point",1);
    move_group_->setGoalJointTolerance(0.05);
    circle_marker_pub = nh_.advertise<visualization_msgs::Marker>("/circle_traj", 1);


    sleep(5);
}




bool robot::move_to_joint_goal(std::vector<double> joint_goal, float speed_factor)
{

    move_group_->setJointValueTarget(joint_goal);

    move_group_->setMaxAccelerationScalingFactor(speed_factor);


    move_group_->setMaxVelocityScalingFactor(speed_factor);


    moveit::planning_interface::MoveGroup::Plan joint_plan;


    bool success = move_group_->plan(joint_plan);

    std::cout<<"success : "<<success<<"\n";

    if(success)
    {

        //        char p;
        //        p ='p';

        //        ROS_INFO ("Press p to run");
        //        std::cin>>p;
        sleep(1);

        move_group_->execute(joint_plan);


        return true;
    }
    else
        return false;

    //    return (joint_goal_status(joint_goal));

}


bool robot::move_to_pose_goal(std::vector<double> pose_goal_vec, float speed_factor)
{

    geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();
    geometry_msgs::PoseStamped pose_goal;
    pose_goal.pose.position.x = pose_goal_vec.at(0);
    pose_goal.pose.position.y = pose_goal_vec.at(1);
    pose_goal.pose.position.z = pose_goal_vec.at(2);

    pose_goal.pose.orientation=current_pose.pose.orientation;

    move_group_->setPoseTarget(pose_goal);

    move_group_->setMaxAccelerationScalingFactor(speed_factor);
    move_group_->setMaxVelocityScalingFactor(speed_factor);

    moveit::planning_interface::MoveGroup::Plan pose_plan;


    bool success = move_group_->plan(pose_plan);


    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    if(success)
    {

        char p;
        p ='p';

        ROS_INFO ("Press p to run");
        std::cin>>p;

        move_group_->execute(pose_plan);


        return true;
    }
    else
        return false;

}



bool robot::joint_goal_status(std::vector<double> joint_goal)
{
    std::vector<double> current_joint_angles = move_group_->getCurrentJointValues();

    float joint_threshold = 0.1;

    float joint_error = 0;

    for(int i = 0; i<6; i++)
        joint_error = joint_error + std::pow((current_joint_angles.at(i) - joint_goal.at(i)), 2);

    if(std::sqrt(joint_error) < joint_threshold)
        return true;
    else
        return false;

}


bool robot::circle(std::vector<double> center, float radius, float speed_factor)
{
    geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();

    std::vector<geometry_msgs::Pose> waypoints;
    moveit::planning_interface::MoveGroup::Plan pose_plan;

    moveit_msgs::RobotTrajectory trajectory;


    double th=0;
    float speed =10.0;
    float dt = 0.005;
    double x,y,z;



    visualization_msgs::Marker points;
    points.header.frame_id ="/world";
    points.header.stamp = ros::Time::now();
    points.ns ="points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =1.0;



    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.01;
    points.scale.y = 0.01;


    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;


    for (int i=0;i<200;i++)
    {
        geometry_msgs::Pose wp;

        th= th+ speed*dt;

//        x= radius*std::cos(th)+center.at(0);
//        y=  radius*std::sin(th)+center.at(1);
//        z= center.at(2);


                x= center.at(0);
                y= radius*std::cos(th)+ center.at(1);
                z= radius*std::sin(th)+center.at(2);

        wp.position.x = x;
        wp.position.y = y;
        wp.position.z = z;
        wp.orientation = current_pose.pose.orientation;
        waypoints.push_back(wp);

        if(i>0){
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;

            points.points.push_back(p);}

    }

    while (circle_marker_pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    circle_marker_pub.publish(points);

    double fraction = move_group_->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    std::cout << "\n Path Computed: " << 100*fraction <<"\n";
    if(fraction > 0.95)
    {
        robot_trajectory::RobotTrajectory rt (move_group_->getCurrentState()->getRobotModel(), "manipulator");

        rt.setRobotTrajectoryMsg(move_group_->getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        iptp.computeTimeStamps(rt, speed_factor, speed_factor);

        rt.getRobotTrajectoryMsg(trajectory);

        pose_plan.trajectory_ = trajectory;

        std::cout<< "press to run \n";
        char a;
        a='a';
        std::cin>>a;

        move_group_->execute(pose_plan);

        return true;
        //        return pose_goal_status(pose_goal);
    }
    return false;


}

bool robot::compute_cartesian_path(geometry_msgs::Pose pose_goal, float speed_factor)
{
    std::vector<geometry_msgs::Pose> waypoints;

    moveit::planning_interface::MoveGroup::Plan pose_plan;

    moveit_msgs::RobotTrajectory trajectory;

    waypoints.push_back(normal);

    waypoints.push_back(pose_goal);

    double fraction = move_group_->computeCartesianPath(waypoints,
                                                        0.01,  // eef_step
                                                        0.0,   // jump_threshold
                                                        trajectory);

    std::cout << "\n Path Computed: " << 100*fraction <<"\n";
    if(fraction > 0.95)
    {
        robot_trajectory::RobotTrajectory rt (move_group_->getCurrentState()->getRobotModel(), "manipulator");

        rt.setRobotTrajectoryMsg(move_group_->getCurrentState()->getRobotModel(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;

        iptp.computeTimeStamps(rt, speed_factor, speed_factor);

        rt.getRobotTrajectoryMsg(trajectory);

        pose_plan.trajectory_ = trajectory;

        //        std::cout<< "press to run \n";
        //        char a;
        //        a='a';
        //        std::cin>>a;

        move_group_->execute(pose_plan);

        return true;
        //        return pose_goal_status(pose_goal);
    }
    return false;
}

bool robot::pose_goal_status(geometry_msgs::Pose pose_goal)
{
    geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();

    float linear_threshold = 0.01;

    float linear_error =  std::pow((current_pose.pose.position.x - pose_goal.position.x), 2) +
            std::pow((current_pose.pose.position.y - pose_goal.position.y), 2) +
            std::pow((current_pose.pose.position.z - pose_goal.position.z), 2);

    if(std::sqrt(linear_error) < linear_threshold)
        return true;
    else
        return false;

}



//bool robot::get_brick_pickup_pose()
//{

//    all_services::Pose_of_cube2 get_pose_srv;
////    visualization_msgs::Marker marker;

////    marker.header.frame_id = "world";
////    marker.header.stamp = ros::Time::now();
////    marker.ns = "normal_point";
////    marker.id = 0;
////    marker.type = visualization_msgs::Marker::SPHERE;
////    marker.action = visualization_msgs::Marker::ADD;
////    marker.pose.orientation.x = 0.0;
////    marker.pose.orientation.y = 0.0;
////    marker.pose.orientation.z = 0.0;
////    marker.pose.orientation.w = 1.0;
////    marker.scale.x = 0.02f;
////    marker.scale.y = 0.02f;
////    marker.scale.z = 0.02f;
////    marker.color.b = 1.0f;
////    marker.color.a = 1.0f;
////    marker.lifetime = ros::Duration();


//    geometry_msgs::PoseStamped pose_of_cube;
//    pose_of_cube.header.frame_id = "world";

//    get_pose_srv.request.an = 'g';

//    if (get_pose_client.call(get_pose_srv))
//    {
//        if(get_pose_srv.response.final_pose.data.size() > 0)
//        {
//            normal.position.x = get_pose_srv.response.final_pose.data.at(0);
//            normal.position.y = get_pose_srv.response.final_pose.data.at(1);
//            normal.position.z = get_pose_srv.response.final_pose.data.at(2);

//            centroid.position.x = get_pose_srv.response.final_pose.data.at(3);
//            centroid.position.y = get_pose_srv.response.final_pose.data.at(4);
//            centroid.position.z = get_pose_srv.response.final_pose.data.at(5);

//            lift.position.x = centroid.position.x;
//            lift.position.y = centroid.position.y;
//            lift.position.z = centroid.position.z + 0.15;

//            normal.orientation.w = get_pose_srv.response.final_pose.data.at(6);
//            normal.orientation.x = get_pose_srv.response.final_pose.data.at(7);
//            normal.orientation.y = get_pose_srv.response.final_pose.data.at(8);
//            normal.orientation.z = get_pose_srv.response.final_pose.data.at(9);

//            centroid.orientation = normal.orientation;
//            lift.orientation = centroid.orientation;

////            marker.pose.position.x = get_pose_srv.response.final_pose.data.at(0);
////            marker.pose.position.y = get_pose_srv.response.final_pose.data.at(1);
////            marker.pose.position.z = get_pose_srv.response.final_pose.data.at(2);

//            pose_of_cube.pose.position.x = get_pose_srv.response.final_pose.data.at(3);
//            pose_of_cube.pose.position.y = get_pose_srv.response.final_pose.data.at(4);
//            pose_of_cube.pose.position.z = get_pose_srv.response.final_pose.data.at(5);

//            pose_of_cube.pose.orientation.w = get_pose_srv.response.final_pose.data.at(6);
//            pose_of_cube.pose.orientation.x = get_pose_srv.response.final_pose.data.at(7);
//            pose_of_cube.pose.orientation.y = get_pose_srv.response.final_pose.data.at(8);
//            pose_of_cube.pose.orientation.z = get_pose_srv.response.final_pose.data.at(9);
//        }
//        else
//            return false;
//    }
//    else
//    {
//        ROS_ERROR("Failed to call service pose_of_cube");
//        return false;
//    }

////    pose_of_cube_pub.publish(pose_of_cube);
////    marker_pub.publish(marker);
//    return true;
//}


/*geometry_msgs::PoseStamped robot::get_brick_place_pose()
{
    all_services::Pose_of_cube2 get_pose_srv;
//    visualization_msgs::Marker marker;

//    marker.header.frame_id = "world";
//    marker.header.stamp = ros::Time::now();
//    marker.ns = "normal_point";
//    marker.id = 0;
//    marker.type = visualization_msgs::Marker::SPHERE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.scale.x = 0.02f;
//    marker.scale.y = 0.02f;
//    marker.scale.z = 0.02f;
//    marker.color.b = 1.0f;
//    marker.color.a = 1.0f;
//    marker.lifetime = ros::Duration();


    geometry_msgs::PoseStamped pose_of_cube;
    pose_of_cube.header.frame_id = "world";

    get_pose_srv.request.an = 'p';

    if (get_pose_client.call(get_pose_srv))
    {
        if(get_pose_srv.response.final_pose.data.size() > 0)
        {
            normal.position.x = get_pose_srv.response.final_pose.data.at(0);
            normal.position.y = get_pose_srv.response.final_pose.data.at(1);
            normal.position.z = get_pose_srv.response.final_pose.data.at(2);

            centroid.position.x = get_pose_srv.response.final_pose.data.at(3);
            centroid.position.y = get_pose_srv.response.final_pose.data.at(4);
            centroid.position.z = get_pose_srv.response.final_pose.data.at(5);

            lift.position.x = centroid.position.x;
            lift.position.y = centroid.position.y;
            lift.position.z = centroid.position.z + 0.2;

            normal.orientation.w = get_pose_srv.response.final_pose.data.at(6);
            normal.orientation.x = get_pose_srv.response.final_pose.data.at(7);
            normal.orientation.y = get_pose_srv.response.final_pose.data.at(8);
            normal.orientation.z = get_pose_srv.response.final_pose.data.at(9);

            centroid.orientation = normal.orientation;
            lift.orientation = centroid.orientation;

//            marker.pose.position.x = get_pose_srv.response.final_pose.data.at(0);
//            marker.pose.position.y = get_pose_srv.response.final_pose.data.at(1);
//            marker.pose.position.z = get_pose_srv.response.final_pose.data.at(2);

            pose_of_cube.pose.position.x = get_pose_srv.response.final_pose.data.at(3);
            pose_of_cube.pose.position.y = get_pose_srv.response.final_pose.data.at(4);
            pose_of_cube.pose.position.z = get_pose_srv.response.final_pose.data.at(5);

            pose_of_cube.pose.orientation.w = get_pose_srv.response.final_pose.data.at(6);
            pose_of_cube.pose.orientation.x = get_pose_srv.response.final_pose.data.at(7);
            pose_of_cube.pose.orientation.y = get_pose_srv.response.final_pose.data.at(8);
            pose_of_cube.pose.orientation.z = get_pose_srv.response.final_pose.data.at(9);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service pose_of_cube");
//        return false;
    }

//    pose_of_cube_pub.publish(pose_of_cube);
//    marker_pub.publish(marker);
    return pose_of_cube;
}*/



//void robot::view_data(geometry_msgs::PoseStamped pose_of_cube)
//{
//    all_services::Pose_of_cube2 get_pose_srv;
//    get_pose_srv.request.an = 't';
//    get_pose_client.call(get_pose_srv);

//    pose_of_cube.header.frame_id = "world";
//    pose_of_cube_pub.publish(pose_of_cube);
//}
