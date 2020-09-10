#include <iostream>
#include <stdio.h>
#include <brick_wall/brick_wall_demo.h>
#include <signal.h>
//#include <all_services/Pose_of_cube2.h>
#include <cstdlib>
#include <visualization_msgs/Marker.h>

#define speed 0.1

void signal_callback_handler(int signum)
{
    std::cout << "Caught Signal" << signum << "\n";

    exit(0);
}






int main(int argc, char** argv)
{

    ros::init( argc, argv,"brick_wall_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot ur5;

//    std::vector<double> j1{-0.11130029359926397, -1.6883624235736292, 1.5193569660186768, -1.472994629536764, -1.5107844511615198, -0.12108451524843389};
//    std::vector<double> p1{0.629, 0.100, 1.273};

//    std::vector<double> j2{0.30305227637290955, -1.451531712208883, 1.2867201566696167, -1.4952862898456019, -1.544483486806051, 0.29230573773384094};
//    std::vector<double> p2{0.677, 0.386, 1.272};

//    std::vector<double> j3{-0.5186393896685999, -1.7323005835162562, 1.549326777458191, -1.4294231573687952, -1.487497631703512, -0.5288279692279261};

//    std::vector<double> p3{ 0.592, -0.140, 1.273};

//    std::vector<double> j4{ 0.8405604958534241, -1.7323005835162562, 1.549326777458191, -1.4294231573687952, -1.487497631703512, -0.5288279692279261};
//    std::vector<double> p4{ 0.262, 0.549, 1.273};

//    std::vector<double> j5{1.3893604278564453, -1.7323005835162562, 1.549326777458191, -1.4294231573687952, -1.487497631703512, -0.5288279692279261};
//    std::vector<double> p5{ -0.063, 0.605, 1.273};

//    std::vector<double> j6{ 2.002960443496704, -1.7323005835162562, 1.549326777458191, -1.4294231573687952, -1.487497631703512, -0.5288279692279261};
//    std::vector<double> p6{-0.400, 0.458, 1.273};

//    std::vector<double> j7{-1.4218395392047327, -1.7323005835162562, 1.549326777458191, -1.4294231573687952, -1.487497631703512, -0.5288279692279261};
//    std::vector<double> p7{ 0.256, -0.552, 1.273};
//    std::vector<double> j8{ -1.9609597365008753, -1.7323005835162562, 1.549326777458191, -1.4294231573687952, -1.487497631703512, -0.5288279692279261};

//    std::vector<double> p8{ -0.063, -0.605, 1.273};



    //----------------------------------------


    std::vector<double> j1{1.9028031826019287, -0.9529545942889612, 2.191471576690674, -2.8612964789019983, -1.4985030333148401, -0.5747202078448694};
    std::vector<double> p1{-0.377, 0.570, 0.524};

    std::vector<double> j2{1.3831788301467896, -0.9529069105731409, 2.191483497619629, -2.86127216020693, -1.4985392729388636, -0.5746963659869593};
    std::vector<double> p2{-0.044, 0.682, 0.524};

    std::vector<double> j3{0.8530757427215576, -0.9529784361468714, 2.191507339477539, -2.86127216020693, -1.4985030333148401, -0.5747202078448694};

    std::vector<double> p3{ 0.306, 0.611, 0.524};

    std::vector<double> j4{ 0.4032557010650635, -0.9529302755938929, 2.191591262817383, -2.861284081135885, -1.4985273520099085, -0.5747202078448694};
    std::vector<double> p4{ 0.541, 0.416, 0.524};

    std::vector<double> j5{-0.05152732530702764, -0.9529545942889612, 2.191471576690674, -2.861284081135885, -1.4985631147967737, -0.5746844450580042};
    std::vector<double> p5{ 0.669, 0.136, 0.524};

    std::vector<double> j6{ -1.256714169179098, -0.9529183546649378, 2.191519260406494, -2.861260239277975, -1.4985273520099085, -0.5747202078448694};
    std::vector<double> p6{0.367, -0.576, 0.524};

    std::vector<double> j7{-1.570625130330221, -0.9303267637835901, 2.1280360221862793, -2.820385758076803, -1.499282185231344, -0.574996296559469};
    std::vector<double> p7{ 0.171, -0.695, 0.524};
    std::vector<double> j8{ -2.013986412678854, -0.9302905241595667, 2.1279759407043457, -2.820373837147848, -1.4993060270892542, -0.5749362150775355};

    std::vector<double> p8{-0.144, -0.701, 0.524};


    //-----------------------------------------


    std::vector<double> j10{ -1.9469416777240198, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p10{-0.070, -0.644, 0.890};

    std::vector<double> j11{-1.5369413534747522, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p11{ 0.193, -0.618, 0.890};

    std::vector<double> j12{  -1.1833413282977503, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p12{ 0.395, -0.513, 0.890};

    std::vector<double> j13{ -0.6521413961993616, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p13{ 0.601, -0.243, 0.890};

    std::vector<double> j14{ -0.05195743242372686, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};

    std::vector<double> p14{  0.633, 0.139, 0.890};

    std::vector<double> j15{ 0.556042492389679, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p15{ 0.440, 0.475, 0.890};

    std::vector<double> j16{ 1.0408425331115723, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p16{ 0.168, 0.626, 0.890};

    std::vector<double> j17{ 1.4832425117492676, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p17{-0.116, 0.637, 0.890};

    std::vector<double> j18{  1.9592424631118774, -1.5542333761798304, 2.0803933143615723, -2.1397221724139612, -1.4880889097796839, -0.5147741476642054};
    std::vector<double> p18{ -0.395, 0.513, 0.890};
    ROS_INFO("Home and start pos initialized");


    float radius=0.2;

//    ur5.move_to_joint_goal(j1,speed);


    while(ros::ok())
    {
        signal(SIGINT, signal_callback_handler);



        usleep(2000000);
        ur5.move_to_joint_goal(j10,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success9 = ur5.circle(p10,radius,speed);
        if(success9)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);
        ur5.move_to_joint_goal(j11,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success11 = ur5.circle(p11,radius,speed);
        if(success11)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");




        usleep(2000000);
        ur5.move_to_joint_goal(j12,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success12 = ur5.circle(p12,radius,speed);
        if(success12)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j13,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success13 = ur5.circle(p13,radius,speed);
        if(success13)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);
        ur5.move_to_joint_goal(j14,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success14 = ur5.circle(p14,radius,speed);
        if(success14)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j15,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success15 = ur5.circle(p15,radius,speed);
        if(success15)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);
        ur5.move_to_joint_goal(j16,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success16 = ur5.circle(p16,radius,speed);
        if(success16)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");



        usleep(2000000);
        ur5.move_to_joint_goal(j17,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success17 = ur5.circle(p17,radius,speed);
        if(success17)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j18,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success18 = ur5.circle(p18,radius,speed);
        if(success18)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j1,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success = ur5.circle(p1,radius,speed);
        if(!success)
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j2,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE2");
        bool success2 = ur5.circle(p2,radius,speed);
        if(!success2)
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);
        ur5.move_to_joint_goal(j3,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE3");
        bool success3 = ur5.circle(p3,radius,speed);
        if(!success3)
             ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j4,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE4");
        bool success4 = ur5.circle(p4,radius,speed);
        if(!success4)
                   ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j5,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE5");
        bool success5 = ur5.circle(p5,radius,speed);
        if(success5)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);
        ur5.move_to_joint_goal(j6,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success6 = ur5.circle(p6,radius,speed);
        if(success6)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");

        usleep(2000000);
        ur5.move_to_joint_goal(j7,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success7 = ur5.circle(p7,radius,speed);
        if(success7)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");


        usleep(2000000);
        ur5.move_to_joint_goal(j8,speed);
        usleep(2000000);

        ROS_INFO("X-Y : POSE1");
        bool success8 = ur5.circle(p8,radius,speed);
        if(success8)
        {
            ROS_INFO("circle done");
        }
        else
            ROS_INFO("OOOOPs !!!!!!! failed------- Time to Debug ");





    }



    return 0;

}
