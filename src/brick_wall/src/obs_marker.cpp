

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

#define obstacles 0
void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << "\n";

    exit(0);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "obs_marker");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    signal(SIGINT, signal_callback_handler);

    sleep(2.0);
    moveit::planning_interface::MoveGroup group("manipulator");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher pub_traj = node_handle.advertise<visualization_msgs::Marker>("/traj", 1000);

    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
    const char* file_name_goal = "/home/ravi/ral_ws/src/brick_wall/src/goal.txt";




    ////////////////////////////// Publish markers ////////////////////////



    Eigen::MatrixXd goal(25,3);

    std::ifstream file_goal;

    file_goal.open(file_name_goal,std::ios_base::in);

    for(int i=0;i<25;i++)
        for(int j=0;j<3;j++)
        {
            std::string line;
            getline(file_goal, line, ',' );
            goal(i,j) = atof(line.c_str());

        }
    file_goal.close();

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
    for (uint32_t i = 0; i < 25; ++i)
    {


        geometry_msgs::Point p;
        p.x = goal(i,0);
        p.y = goal(i,1);
        p.z = goal(i,2)+0.025;

        points.points.push_back(p);


    }
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


    if(obstacles)
    {

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /// ADD OBSTACLES
        ///
        ///
        ///
        ///
        ///
        ///
        ///  /* Define a box to add to the world. */
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
        sphere_pose1.position.x =  0.128;
        sphere_pose1.position.y = 0.3545;
        sphere_pose1.position.z =  0.707;
        sphere_pose1.orientation.w = 1.0;

        collision_object1.primitives.push_back(primitive);
        collision_object1.primitive_poses.push_back(sphere_pose1);
        collision_object1.operation = collision_object1.ADD;

        collision_objects.push_back(collision_object1);

        // /////////////////////////////////////////OBS2////////////////////////////////////////////
        //            moveit_msgs::CollisionObject collision_object2;
        //            collision_object2.header.frame_id = group.getPlanningFrame();

        //            collision_object2.id = "sphere2";

        //            geometry_msgs::Pose sphere_pose2;
        //            sphere_pose2.position.x =  0.65;
        //            sphere_pose2.position.y = 0.1375;
        //            sphere_pose2.position.z =  0.707;
        //            sphere_pose2.orientation.w = 1.0;

        //            collision_object2.primitives.push_back(primitive);
        //            collision_object2.primitive_poses.push_back(sphere_pose2);
        //            collision_object2.operation = collision_object2.ADD;

        //            collision_objects.push_back(collision_object2);
        //            // /////////////////////////////////////////OBS3////////////////////////////////////////////
        //            moveit_msgs::CollisionObject collision_object3;
        //            collision_object3.header.frame_id = group.getPlanningFrame();

        //            collision_object3.id = "sphere3";



        //            geometry_msgs::Pose sphere_pose3;
        //            sphere_pose3.position.x =  0.2445;
        //            sphere_pose3.position.y = -0.28;
        //            sphere_pose3.position.z =  0.707;
        //            sphere_pose3.orientation.w = 1.0;

        //            collision_object3.primitives.push_back(primitive);
        //            collision_object3.primitive_poses.push_back(sphere_pose3);
        //            collision_object3.operation = collision_object3.ADD;

        //            collision_objects.push_back(collision_object3);
        // /////////////////////////////////////////OBS4////////////////////////////////////////////
        moveit_msgs::CollisionObject collision_object4;
        collision_object4.header.frame_id = group.getPlanningFrame();

        collision_object4.id = "sphere4";


        geometry_msgs::Pose sphere_pose4;
        sphere_pose4.position.x =  0.413;
        sphere_pose4.position.y = -0.2395;
        sphere_pose4.position.z =  0.89;
        sphere_pose4.orientation.w = 1.0;

        collision_object4.primitives.push_back(primitive);
        collision_object4.primitive_poses.push_back(sphere_pose4);
        collision_object4.operation = collision_object4.ADD;

        collision_objects.push_back(collision_object4);
        ///////////////////////////////////// OBS5/////////////////////////////////////////////////////
        //            moveit_msgs::CollisionObject collision_object5;
        //            collision_object5.header.frame_id = group.getPlanningFrame();

        //            collision_object5.id = "sphere5";

        //            geometry_msgs::Pose sphere_pose5;
        //            sphere_pose5.position.x =  0.2585;
        //            sphere_pose5.position.y = 0.388;
        //            sphere_pose5.position.z =  0.89;
        //            sphere_pose5.orientation.w = 1.0;

        //            collision_object5.primitives.push_back(primitive);
        //            collision_object5.primitive_poses.push_back(sphere_pose5);
        //            collision_object5.operation = collision_object5.ADD;

        //            collision_objects.push_back(collision_object5);

        //            // /////////////////////////////////////////OBS6////////////////////////////////////////////
        //            moveit_msgs::CollisionObject collision_object6;
        //            collision_object6.header.frame_id = group.getPlanningFrame();

        //            collision_object6.id = "sphere6";


        //            geometry_msgs::Pose sphere_pose6;
        //            sphere_pose6.position.x =  0.63;
        //            sphere_pose6.position.y =0.1195;
        //            sphere_pose6.position.z =  1.08;
        //            sphere_pose6.orientation.w = 1.0;

        //            collision_object6.primitives.push_back(primitive);
        //            collision_object6.primitive_poses.push_back(sphere_pose6);
        //            collision_object6.operation = collision_object6.ADD;

        //            collision_objects.push_back(collision_object6);
        //            // /////////////////////////////////////////OBS7////////////////////////////////////////////
        //            moveit_msgs::CollisionObject collision_object7;
        //            collision_object7.header.frame_id = group.getPlanningFrame();

        //            collision_object7.id = "sphere7";

        //            geometry_msgs::Pose sphere_pose7;
        //            sphere_pose7.position.x =  0.4475;
        //            sphere_pose7.position.y = 0.344;
        //            sphere_pose7.position.z =  1.08;
        //            sphere_pose7.orientation.w = 1.0;

        //            collision_object7.primitives.push_back(primitive);
        //            collision_object7.primitive_poses.push_back(sphere_pose7);
        //            collision_object7.operation = collision_object7.ADD;

        //            collision_objects.push_back(collision_object7);
        // /////////////////////////////////////////OBS8////////////////////////////////////////////
        moveit_msgs::CollisionObject collision_object8;
        collision_object8.header.frame_id = group.getPlanningFrame();

        collision_object8.id = "sphere8";


        geometry_msgs::Pose sphere_pose8;
        sphere_pose8.position.x =  0.284;
        sphere_pose8.position.y = -0.233;
        sphere_pose8.position.z =  1.08;
        sphere_pose8.orientation.w = 1.0;

        collision_object8.primitives.push_back(primitive);
        collision_object8.primitive_poses.push_back(sphere_pose8);
        collision_object8.operation = collision_object8.ADD;

        collision_objects.push_back(collision_object8);

        // Now, let's add the collision object into the world
        ROS_INFO("Add an object into the world");
        planning_scene_interface.addCollisionObjects(collision_objects);

        /* Sleep so we have time to see the object in RViz */
        sleep(2.0);

        char p ='q';

        ROS_INFO ("Press q to remove collision added in RVIZ");
        std::cin>>p;


        // Now, let's remove the collision object from the world.
        ROS_INFO("Remove the object from the world");
        std::vector<std::string> object_ids;
        object_ids.push_back(collision_object1.id);
        //            object_ids.push_back(collision_object2.id);
        //            object_ids.push_back(collision_object3.id);
        object_ids.push_back(collision_object4.id);
        //            object_ids.push_back(collision_object5.id);
        //            object_ids.push_back(collision_object6.id);
        //            object_ids.push_back(collision_object7.id);
        object_ids.push_back(collision_object8.id);


        planning_scene_interface.removeCollisionObjects(object_ids);

    }

    ros::shutdown();
    return 0;
}
