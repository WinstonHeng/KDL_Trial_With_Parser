#include <ros/ros.h>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

#include <kdl/frames.hpp>   //declaration for frames
#include <kdl/jntarray.hpp> //declaration for joint-state variables

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_kdl/tf2_kdl.h>


void convertKDLtoTF(KDL::Frame cartpos, geometry_msgs::TransformStamped transformStamped)
{
    static tf2_ros::TransformBroadcaster br;
    transformStamped = tf2::kdlToTransform(cartpos);
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "End-Effector";

    br.sendTransform(transformStamped);
}

using namespace KDL;
int main(int argc, char **argv)
{
    KDL::Chain chain;
    Segment s1 = Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 1.0, 0.0)));
    Segment s2 = Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.5, 0.0, 0.0)));
    Segment s3 = Segment(Joint(Joint::None), Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.25, -0.25, 0.0)));
    Segment s4 = Segment(Joint(Joint::RotY), Frame(Rotation::RPY(0.0, M_PI / 4, 0.0), Vector(0.15, 0.15, 0.0)));
    chain.addSegment(s1);
    chain.addSegment(s2);
    chain.addSegment(s3);
    chain.addSegment(s4);
    std::cout << "Number of Segments: " << chain.getNrOfSegments() << std::endl;
    std::cout << "Number of Joints: " << chain.getNrOfJoints() << std::endl;


    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
    std::cout << "The number of joints: " << nj << std::endl;

    // Assign some values to the joint positions
    for (unsigned int i = 0; i < nj; i++)
    {
        float myinput;
        printf("Enter the position of joint %i: ", i);
        scanf("%e", &myinput);
        jointpositions(i) = (double)myinput;
    }

    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
    if (kinematics_status >= 0)
    {
        std::cout << cartpos << std::endl; // last line of matrix gives coordinates of end-effector
        printf("%s \n", "Success, thanks KDL!");
        // std::cout << "Position of end-effector: " << cartpos.p.y() << std::endl;
    }
    else
    {
        printf("%s \n", "Error: could not calculate forward kinematics :(");
    }


    geometry_msgs::TransformStamped transformStamped;
    
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;
    ros::Rate r(100);

    while(n.ok()){
        convertKDLtoTF(cartpos, transformStamped);
        r.sleep();
    }
}


