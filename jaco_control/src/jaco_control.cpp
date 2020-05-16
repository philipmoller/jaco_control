#include <ros/ros.h>
#include <ros/console.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include "global.h"


class JacoControl{
  private:
  quaternions q;
  bottle b;
  cup c;
  double execution_time;
  ros::WallTime start_, end_;
  public:

  void moveSleep(){
      static const std::string PLANNING_GROUP = "arm";

      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit::planning_interface::MoveGroupInterface::Plan planHome;
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = 0 * q.d2r;
      joint_group_positions[1] = 120 * q.d2r;
      joint_group_positions[2] = 0 * q.d2r;
      joint_group_positions[3] = 30 * q.d2r;
      joint_group_positions[4] = 0 * q.d2r;
      joint_group_positions[5] = 220 * q.d2r;
      joint_group_positions[6] = 0 * q.d2r;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      move_group.move();
    }

  void jointPlan(float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = x_ori;
    goal_pose.orientation.y = y_ori;
    goal_pose.orientation.z = z_ori;
    goal_pose.orientation.w = w_ori;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan planJoint;

    bool success = (move_group.plan(planJoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }

  void jointPlanWithConstraint(float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setPlanningTime(7);

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = x_ori;
    goal_pose.orientation.y = y_ori;
    goal_pose.orientation.z = z_ori;
    goal_pose.orientation.w = w_ori;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan planJoint;

    bool success = (move_group.plan(planJoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ros::Duration(7.0).sleep();
    //EXECUTE TRAJECTORY
    move_group.move();
  }

  void cartesianPlan(float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "arm";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //START POSITION
    robot_state::RobotState start_pose(*move_group.getCurrentState());

    //GOAL POSITION
    geometry_msgs::Pose goal_pose;
    goal_pose.orientation.x = x_ori;
    goal_pose.orientation.y = y_ori;
    goal_pose.orientation.z = z_ori;
    goal_pose.orientation.w = w_ori;
    goal_pose.position.x = x_pos;
    goal_pose.position.y = y_pos;
    goal_pose.position.z = z_pos;
    move_group.setPoseTarget(goal_pose);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(goal_pose);

    //SET VELOCITY
    move_group.setMaxVelocityScalingFactor(1);
    move_group.setPlanningTime(5.0);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    moveit::planning_interface::MoveGroupInterface::Plan planCartesian;

    planCartesian.trajectory_= trajectory;

    ros::Duration(5.0).sleep();

    move_group.execute(planCartesian);
  }

  void grasp(float radian){
    static const std::string PLANNING_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setPlanningTime(1);

    moveit::planning_interface::MoveGroupInterface::Plan planGrasp;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //Joint angles for the three main finger joints
    joint_group_positions[0] = radian;
    joint_group_positions[1] = radian;
    joint_group_positions[2] = radian;
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(planGrasp) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }

  void grasptip(float radian){
    static const std::string PLANNING_GROUP = "grippertip";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    move_group.setPlanningTime(1);

    moveit::planning_interface::MoveGroupInterface::Plan planGraspTip;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //Joint angles for the three main finger joints
    joint_group_positions[0] = radian;
    joint_group_positions[1] = radian;
    joint_group_positions[2] = radian;
    move_group.setJointValueTarget(joint_group_positions);

    bool success = (move_group.plan(planGraspTip) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //EXECUTE TRAJECTORY
    move_group.move();
  }

  void createObject(std::string type, int id, float x_pos, float y_pos, float z_pos, float x_dim, float y_dim, float z_dim, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "gripper";
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();

    if (id == 1){
    collision_object.id = "Object1";
    }
    else if (id == 2){
    collision_object.id = "Object2";
    }
    else if (id == 3){
    collision_object.id = "Object3";
    }
    else if (id == 4){
    collision_object.id = "Object4";
    }
    else if (id == 5){
    collision_object.id = "Object5";
    }

    shape_msgs::SolidPrimitive primitive;
    if (type == "Cylinder"){
      primitive.type = primitive.CYLINDER;
    }
    if (type == "Box"){
      primitive.type = primitive.BOX;
    }
    if (type == "Sphere"){
      primitive.type = primitive.SPHERE;
    }
    if (type == "Cone"){
      primitive.type = primitive.CONE;
    }

    primitive.dimensions.resize(3);
    primitive.dimensions[0] = x_dim;
    primitive.dimensions[1] = y_dim;
    primitive.dimensions[2] = z_dim;

    geometry_msgs::Pose figure_pose;
    figure_pose.orientation.w = w_ori;
    figure_pose.orientation.x = x_ori;
    figure_pose.orientation.y = y_ori;
    figure_pose.orientation.z = z_ori;
    figure_pose.position.x = x_pos;
    figure_pose.position.y = y_pos;
    figure_pose.position.z = z_pos;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(figure_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::Duration(0.5).sleep();
    //move_group.setPlanningTime(10.0);
  }

  void removeObject(int id){
    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();
    if (id == 1){
      collision_object.id = "Object1";
    }
    else if (id == 2){
      collision_object.id = "Object2";
    }
    else if (id == 3){
      collision_object.id = "Object3";
    }
    else if (id == 4){
      collision_object.id = "Object4";
    }
    else if (id == 5){
      collision_object.id = "Object5";
    }
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);
    /* Sleep to give Rviz time to show the object is no longer there. */
    sleep(0.5);
  }

  void attachObject(int id){
    static const std::string PLANNING_GROUP = "gripper";
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();
    if (id == 1){
    collision_object.id = "Object1";
    move_group.attachObject(collision_object.id);
    }
    else if (id == 2){
    collision_object.id = "Object2";
    move_group.attachObject(collision_object.id);
    }
  /*  else if (id == 3){
    collision_object.id = "Object3";
    move_group.attachObject(collision_object.id);
    }
    else if (id == 4){
    collision_object.id = "Object4";
    move_group.attachObject(collision_object.id);
    }
    else if (id == 5){
    collision_object.id = "Object5";
    move_group.attachObject(collision_object.id);
  } */
    sleep(1.0);
  }

  void detachObject(int id){
    static const std::string PLANNING_GROUP = "gripper";
    moveit_msgs::CollisionObject collision_object;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    collision_object.header.frame_id = move_group.getPlanningFrame();
    if (id == 1){
    collision_object.id = "Object1";
    move_group.detachObject(collision_object.id);
    }
    else if (id == 2){
    collision_object.id = "Object2";
    move_group.detachObject(collision_object.id);
    }
    sleep(1.0);
  }

  void gripperConstraints(float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "gripper";
    ocm.header.frame_id = "arm";
    ocm.orientation.x = x_ori;
    ocm.orientation.y = y_ori;
    ocm.orientation.z = z_ori;
    ocm.orientation.w = w_ori;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);
  }

  void clearConstraints(){
    static const std::string PLANNING_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.clearPathConstraints();
  }

  void orientGraspVertical(float x_pos, float y_pos){
    q.ang = atan2(y_pos,x_pos)-(90*q.d2r);

    q.Y = q.ang;          //YAW = Z
    q.P = 0.0;          //PITCH = Y
    q.R = -1.570796;    //ROLL = X

    q.cy = cos(q.Y * 0.5);
    q.sy = sin(q.Y * 0.5);
    q.cp = cos(q.P * 0.5);
    q.sp = sin(q.P * 0.5);
    q.cr = cos(q.R * 0.5);
    q.sr = sin(q.R * 0.5);

    q.w = q.cr * q.cp * q.cy + q.sr * q.sp * q.sy;
    q.x = q.sr * q.cp * q.cy - q.cr * q.sp * q.sy;
    q.y = q.cr * q.sp * q.cy + q.sr * q.cp * q.sy;
    q.z = q.cr * q.cp * q.sy - q.sr * q.sp * q.cy;
  }

  void orientGraspHorizontal(float x_pos, float y_pos){
    q.ang = atan2(y_pos,x_pos)-(90*q.d2r);

    q.Y = q.ang;          //YAW = Z
    q.P = 1.570796;     //PITCH = Y
    q.R = -1.570796;    //ROLL = X


    q.cy = cos(q.Y * 0.5);
    q.sy = sin(q.Y * 0.5);
    q.cp = cos(q.P * 0.5);
    q.sp = sin(q.P * 0.5);
    q.cr = cos(q.R * 0.5);
    q.sr = sin(q.R * 0.5);

    q.w = q.cr * q.cp * q.cy + q.sr * q.sp * q.sy;
    q.x = q.sr * q.cp * q.cy - q.cr * q.sp * q.sy;
    q.y = q.cr * q.sp * q.cy + q.sr * q.cp * q.sy;
    q.z = q.cr * q.cp * q.sy - q.sr * q.sp * q.cy;
  }

  void pourBottleAt(float x_pos, float y_pos, float z_pos){
    pouring pour;
    q.ang = atan2(y_pos,x_pos)-(90*q.d2r);
    q.hyp = sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)));

    q.ang2 = atan2(0.1,q.hyp);
    float full_ang = q.ang + q.ang2 + (90*q.d2r);
    float hyp2 = sqrt((0.1*0.1)+(q.hyp*q.hyp));

    pour.x = hyp2 * cos(full_ang);
    pour.y = hyp2 * sin(full_ang);
    pour.z = z_pos + 0.1;

    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(b.x, b.y, b.z+0.2, q.x, q.y, q.z, q.w);

    gripperConstraints(q.x, q.y, q.z, q.w);
    ros::Duration(1).sleep();
    jointPlanWithConstraint(pour.x, pour.y, pour.z+0.1, q.x, q.y, q.z, q.w);
    cartesianPlan(pour.x, pour.y, pour.z, q.x, q.y, q.z, q.w);
    clearConstraints();
    ros::Duration(1).sleep();

    orientGraspHorizontal(x_pos, y_pos);
    cartesianPlan(pour.x, pour.y, pour.z, q.x, q.y, q.z, q.w);

    ros::Duration(5).sleep();

    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(pour.x, pour.y, pour.z, q.x, q.y, q.z, q.w);
    ros::Duration(1).sleep();
    cartesianPlan(pour.x, pour.y, pour.z+0.1, q.x, q.y, q.z, q.w);
  }

  void linearRetract(float x_pos, float y_pos, float z_pos){
    retract retr;
    q.ang = atan2(y_pos,x_pos);
    q.hyp = (sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)))-0.15);

    retr.x = q.hyp * cos(q.ang);
    retr.y = q.hyp * sin(q.ang);
    retr.z = z_pos+0.1;

    cartesianPlan(retr.x, retr.y, retr.z, q.x, q.y, q.z, q.w);
  }

  void linearApproach(float x_pos, float y_pos, float z_pos){
    approach appr;
    q.ang = atan2(y_pos,x_pos);
    q.hyp = (sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)))-0.15);

    appr.y = q.hyp * sin(q.ang);
    appr.x = q.hyp * cos(q.ang);
    appr.z = z_pos+0.1;

    jointPlan(appr.x, appr.y, appr.z, q.x, q.y, q.z, q.w);
  }

  void closeGripper(){
    grasptip(0.55);
    grasp(0.55);
  }

  void openGripper(){
    grasptip(0);
    grasp(0);
  }

  void pickObject(float x_pos, float y_pos, float z_pos, int obj_id){
    orientGraspVertical(x_pos, y_pos);
    linearApproach(x_pos, y_pos, z_pos);
    cartesianPlan(x_pos, y_pos, z_pos, q.x, q.y, q.z, q.w);
    closeGripper();
    attachObject(obj_id);
    gripperConstraints(q.x, q.y, q.z, q.w);
    ros::Duration(1).sleep();
    cartesianPlan(x_pos, y_pos, z_pos+0.2, q.x, q.y, q.z, q.w);
    clearConstraints();
    ros::Duration(1).sleep();
  }

  void placeObject(float x_pos, float y_pos, float z_pos, int obj_id){
    orientGraspVertical(x_pos, y_pos);
    gripperConstraints(q.x, q.y, q.z, q.w);
    ros::Duration(1).sleep();
    jointPlanWithConstraint(x_pos, y_pos, z_pos+0.2, q.x, q.y, q.z, q.w);
    cartesianPlan(x_pos, y_pos, z_pos, q.x, q.y, q.z, q.w);
    clearConstraints();
    ros::Duration(1).sleep();
    openGripper();
    detachObject(obj_id);
    linearRetract(x_pos, y_pos, z_pos);
  }

  void simulateUser(){
    createObject("Sphere", 3, 0.45, -0.25, 0.55, 0.15, 0.1, 0.035, 0, 0, 0, 1);
    createObject("Box", 4, 0.45, -0.25, 0.15, 0.18, 0.4, 0.48, 0, 0, 0, 1);
    createObject("Box", 5, -0.85, -0.25, 0.25, 0.75, 1.2, 0.05, 0, 0, 0, 1);
  }
  void timer_start(){
    start_ = ros::WallTime::now();
  }
  void timer_end(){
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec()*1e-6;
    ROS_INFO_STREAM("Task execution time (ms): " << execution_time);
  }

  void pour(){
    moveSleep();
    timer_start();
    ros::Duration(2).sleep();

    pickObject(b.x, b.y, b.z, 1);

    pourBottleAt(c.x, c.y, c.z);

    placeObject(b.x, b.y, b.z, 1);

    moveSleep();
    timer_end();
  }

  void pick(int obj_id){
    timer_start();
    if(obj_id == 1){
      pickObject(b.x, b.y, b.z, obj_id);
    }
    else if(obj_id == 2){
      pickObject(c.x, c.y, c.z, obj_id);
    }
    else{
      ROS_INFO("Unrecognized object");
    }
    timer_end();
  }


  void place(float x_pos, float y_pos, float z_pos, int obj_id){
    timer_start();
    placeObject(x_pos, y_pos, z_pos, obj_id);
    linearRetract(x_pos, y_pos, z_pos);
    moveSleep();
    timer_end();
  }

 void simulateObjects(){
   simulateUser();
   ros::Duration(2).sleep();
   //Spawn Bottle
   createObject("Cylinder", 1, b.x, b.y, b.z, 0.2, 0.035, 0.035, 0, 0, 0, 1);
   ros::Duration(2).sleep();
   //Spawn Cup
   createObject("Cylinder", 2, c.x, c.y, c.z, 0.1, 0.05, 0.05, 0, 0, 0, 1);
   ros::Duration(2).sleep();
 }

 int menu1(){
   while(ros::ok){
   int choice;
   int option;
   int object;

   moveSleep();

   ROS_INFO("Choose task:");
   ROS_INFO("1. Pick");
   ROS_INFO("2. Pour");
   ROS_INFO("0. Power Off");
   std::cin >> choice;

   if(choice==1){
     ROS_INFO("Choose object:");
     ROS_INFO("1. Bottle");
     ROS_INFO("2. Cup");
     ROS_INFO("0. Exit");
     std::cin >> object;

     if(object == 1){
       pick(object);
     }

       else if(object == 2){
         pick(object);
       }

       else if(object == 0){
         ROS_INFO("Power: Off");
         ros::shutdown();
         return 0;
       }

       else{
         ROS_INFO("Unrecognized object, please try again");
         choice = 1;
       }

     ROS_INFO("Enter 1 to place");
     std::cin >> option;
     if(option == 1 && object == 1){
       place(b.x, b.y, b.z, object);
     }
     else if(option == 1 && object == 2){
       place(c.x, c.y, c.z, object);
     }
     else{
       ROS_INFO("Not a valid input, try again");
       std::cin >> option;
     }
   }

   else if(choice==2){
     pour();
   }
   else if(choice==0){
     ROS_INFO("Exiting");
     ros::shutdown();
     return 0;
   }
  }
 }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jaco_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  JacoControl JC;
  JC.simulateObjects();
  JC.menu1();

  ros::spin();
  ros::shutdown();
  return 0;
}
