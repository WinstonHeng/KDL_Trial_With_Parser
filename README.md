# kdl_trial_with_parser
Obtained KDL tree using KDL parser. Performed Forward Kinematics on it. 

This package contains 3 codes that pertain to my trial with the KDL library.
Only the code "treeGenerateTest.cpp" is relevant.

First, please run "roscore"
Next, run the code "rosrun kdl_with_parser treeGenerateTest". Input the joint values for the joints.

To view the TF for the end-effector, please run "rosrun rviz rviz" in a seperate terminal. Change the fixed frame to "world" and add a TF display.

1) roscore
2) rosrun kdl_with_parser treeGenerateTest
3) rosrun rviz rviz (Change the fixed frame to "world" and add a TF display.)
