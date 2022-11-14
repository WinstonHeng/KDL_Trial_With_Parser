#include <ros/ros.h>
#include <iostream>
#include <string>
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp> //smart pointer
#include <iomanip>
#include <kdl/tree.hpp>
#include <stdio.h>

#include <kdl/frames.hpp>   //declaration for frames
#include <kdl/jntarray.hpp> //declaration for joint-state variables
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>

using namespace KDL;

void printOnlyJoints(KDL::Tree tree, std::string root, std::string child, KDL::SegmentMap::const_iterator maproot, KDL::Chain chain)
{
    const KDL::SegmentMap::const_iterator &link = maproot;
    for (unsigned int i = 0; i < GetTreeElementChildren(link->second).size(); i++)
    {
        tree.getChain(root, child, chain);
        std::string segmentWithJoints;                                       // Define variable to store joint state
        for (uint segment = 0; segment < chain.getNrOfSegments(); ++segment) // Iterate over all segments
        {
            KDL::Joint joint = chain.getSegment(segment).getJoint(); // Store name of joint from segment from chain

            KDL::Segment s = chain.getSegment(segment);

            if (joint.getType() != KDL::Joint::None) // If joint exists in segment, store name in joint_state variable
            {
                std::cout << "Segment name is: " << s.getName() << std::endl;
                std::cout << "Joint name is: " << joint.getName() << std::endl;
            }
        }
    }
}

int main(int argc, char **argv)
{
    KDL::Tree tree;
    std::string path = ros::package::getPath("kdl_with_parser");
    if (!kdl_parser::treeFromFile(path + "/urdf/husky_ur5e_robot.urdf", tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }
    std::string root;
    KDL::Chain chain;
    KDL::SegmentMap::const_iterator maproot = tree.getRootSegment();
    root = GetTreeElementSegment(maproot->second).getName();
    std::string child = "tool0";
    printOnlyJoints(tree, root, child, maproot, chain);
}