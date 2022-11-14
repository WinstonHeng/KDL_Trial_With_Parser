#include <ros/ros.h>
#include <iostream>
#include "kdl_parser/kdl_parser.hpp"
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <kdl/tree.hpp>

#include <kdl/frames.hpp>   //declaration for frames
#include <kdl/jntarray.hpp> //declaration for joint-state variables
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_kdl/tf2_kdl.h>
#include <ros/package.h>

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
        std::cout << "Segment: " << s.getName() << " (Joint name: " << joint.getName() << ", Type: " << joint.getTypeName() << ")" << std::endl;
        // std::cout << "Joint name is: " << joint.getName() << std::endl;
      }
    }
  }
}

void convertKDLtoTF(KDL::Frame cartpos, geometry_msgs::TransformStamped transformStamped)
{
  static tf2_ros::TransformBroadcaster br;
  transformStamped = tf2::kdlToTransform(cartpos);
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "End-Effector";
  br.sendTransform(transformStamped);
}

void printLink(const KDL::SegmentMap::const_iterator &link, const std::string &prefix)
{
  std::cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has " << GetTreeElementChildren(link->second).size() << " children" << std::endl;
  for (unsigned int i = 0; i < GetTreeElementChildren(link->second).size(); i++)
  {
    printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
  }
}

KDL::Frame getChainFromTree(KDL::Tree tree, std::string root, std::string child)
{
  KDL::Chain chain;
  if (!tree.getChain(root, child, chain))
  {
    ROS_ERROR("Failed to construct kdl chain!!");
  }

  else
  {
    unsigned int nj = chain.getNrOfJoints();
    std::cout << "Number of Joints in Chain: " << nj << "\n"
              << std::endl;

    KDL::JntArray jointpositions = KDL::JntArray(nj);
    KDL::Frame cartpos;
    Eigen::Affine3f tra3f;
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);

    for (unsigned int i = 0; i < nj; i++)
    {
      float myinput;
      printf("Enter the position of joint %i: ", i);
      scanf("%e", &myinput);
      jointpositions(i) = (double)myinput;
    }

    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
    if (kinematics_status >= 0)
    {
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          tra3f(i, j) = cartpos(i, j);
        }
      }
      std::cout << "\n Rotation Matrix of End-effector:  \n"
                << cartpos.M << "\n"
                << std::endl;
      std::cout << "\n Cartesian Coordinates of End-effector:  \n"
                << cartpos.p << "\n"
                << std::endl;
      return cartpos;
    }
    else
    {
      printf("%s \n", "Error: could not calculate forward kinematics :(");
    }
  }
}

int main(int argc, char **argv)
{
  KDL::Tree tree;
  KDL::Chain chain;
  std::string path = ros::package::getPath("kdl_with_parser");
  if (!kdl_parser::treeFromFile(path + "/urdf/husky_ur5e_robot.urdf", tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }
  std::cout << " ======================================" << std::endl;
  std::cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
  std::cout << " ======================================" << std::endl;
  KDL::SegmentMap::const_iterator maproot = tree.getRootSegment();
  printLink(maproot, "");

  std::string root;
  root = GetTreeElementSegment(maproot->second).getName();
  std::string child = "tool0";

  // Publishing TF
  geometry_msgs::TransformStamped transformStamped;
  std::cout << "\nPrinting Segments with joints... \n"
            << std::endl;
  printOnlyJoints(tree, root, child, maproot, chain);
  KDL::Frame cartpos2 = getChainFromTree(tree, root, child);

  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;
  ros::Rate r(1);

  std::cout << "Publishing TF of End-Effector..." << std::endl;

  while (n.ok())
  {
    convertKDLtoTF(cartpos2, transformStamped);
    r.sleep();
  }
}
