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

const double d2r = 0.01745329251; //Convert from degree to radian
float q_w;
float q_x;
float q_y;
float q_z;
float ang;

float pour_x, pour_y, pour_z;
float retr_x, retr_y, retr_z;

//Bottle position
float b_x = -0.5;
float b_y = -0.5;
float b_z = 0.50;

//Cup position
float c_x = -0.7;
float c_y = 0.0;
float c_z = 0.45;

class JacoControl{
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

      joint_group_positions[0] = -15 * d2r;
      joint_group_positions[1] = 120 * d2r;
      joint_group_positions[2] = 0 * d2r;
      joint_group_positions[3] = 30 * d2r;
      joint_group_positions[4] = 0 * d2r;
      joint_group_positions[5] = 180 * d2r;
      joint_group_positions[6] = 0 * d2r;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      move_group.move();
    }

    void moveStretch(){
      static const std::string PLANNING_GROUP = "arm";

      moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

      moveit::planning_interface::MoveGroupInterface::Plan planHome;
      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

      std::vector<double> joint_group_positions;
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

      joint_group_positions[0] = 0 * d2r;
      joint_group_positions[1] = 270 * d2r;
      joint_group_positions[2] = 0 * d2r;
      joint_group_positions[3] = 180 * d2r;
      joint_group_positions[4] = 0 * d2r;
      joint_group_positions[5] = 180 * d2r;
      joint_group_positions[6] = 0 * d2r;
      move_group.setJointValueTarget(joint_group_positions);

      bool success = (move_group.plan(planHome) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      move_group.move();
    }


  void jointPlan(std::string movement_group, float x_pos, float y_pos, float z_pos, float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = movement_group;

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

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.01;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ros::Duration(5).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan planCartesian;

    planCartesian.trajectory_= trajectory;

    //bool success = (move_group.plan(planCartesian) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.execute(planCartesian);
  }

  void grasp(float radian){
    static const std::string PLANNING_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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

    sleep(0.5);
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
    sleep(5.0);
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
    sleep(5.0);
  }

  void gripperConstraints(float x_ori, float y_ori, float z_ori, float w_ori){
    static const std::string PLANNING_GROUP = "arm";

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
    ocm.weight = 1.0;

    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);

    //move_group.setPlanningTime(10.0);
  }

  void clearConstraints(){
    static const std::string PLANNING_GROUP = "gripper";

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.clearPathConstraints();
  }

  void orientGraspVertical(float x_pos, float y_pos){
    ang = atan2(y_pos,x_pos)-(90*d2r);

    float Y = ang;          //YAW = Z
    float P = 0.0;          //PITCH = Y
    float R = -1.570796;    //ROLL = X

    float cy = cos(Y * 0.5);
    float sy = sin(Y * 0.5);
    float cp = cos(P * 0.5);
    float sp = sin(P * 0.5);
    float cr = cos(R * 0.5);
    float sr = sin(R * 0.5);

    q_w = cr * cp * cy + sr * sp * sy;
    q_x = sr * cp * cy - cr * sp * sy;
    q_y = cr * sp * cy + sr * cp * sy;
    q_z = cr * cp * sy - sr * sp * cy;
  }

  void orientGraspHorizontal(float x_pos, float y_pos){
    ang = atan2(y_pos,x_pos)-(90*d2r);

    float Y = ang;          //YAW = Z
    float P = 1.570796;     //PITCH = Y
    float R = -1.570796;    //ROLL = X

    float cy = cos(Y * 0.5);
    float sy = sin(Y * 0.5);
    float cp = cos(P * 0.5);
    float sp = sin(P * 0.5);
    float cr = cos(R * 0.5);
    float sr = sin(R * 0.5);

    q_w = cr * cp * cy + sr * sp * sy;
    q_x = sr * cp * cy - cr * sp * sy;
    q_y = cr * sp * cy + sr * cp * sy;
    q_z = cr * cp * sy - sr * sp * cy;
  }

  void pourBottleAt(float x_pos, float y_pos, float z_pos){
    float ang = atan2(y_pos,x_pos)-(90*d2r);
    float hyp = sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)));

    float ang2 = atan2(0.1,hyp);
    float full_ang = ang + ang2 + (90*d2r);
    float hyp2 = sqrt((0.1*0.1)+(hyp*hyp));

    pour_x = hyp2 * cos(full_ang);
    pour_y = hyp2 * sin(full_ang);
    pour_z = z_pos + 0.1;

    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(pour_x, pour_y, pour_z, q_x, q_y, q_z, q_w);

    orientGraspHorizontal(x_pos, y_pos);
    cartesianPlan(pour_x, pour_y, pour_z, q_x, q_y, q_z, q_w);

    ros::Duration(5).sleep();

    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(pour_x, pour_y, pour_z, q_x, q_y, q_z, q_w);
  }

  void linearRetract(float x_pos, float y_pos, float z_pos){
    float ang = atan2(y_pos,x_pos);
    float hyp = (sqrt((abs(x_pos)*abs(x_pos))+(abs(y_pos)*abs(y_pos)))-0.2);

    retr_x = hyp * cos(ang);
    retr_y = hyp * sin(ang);
    retr_z = z_pos+0.1;

    cartesianPlan(retr_x, retr_y, retr_z, q_x, q_y, q_z, q_w);
  }

  void closeGripper(){
    grasptip(0.5);
    grasp(0.5);
  }

  void openGripper(){
    grasptip(0);
    grasp(0);
  }

  void pickObject(float x_pos, float y_pos, float z_pos, int obj_id){
    orientGraspVertical(x_pos, y_pos);
    ros::Duration(2).sleep();
    cartesianPlan(x_pos, y_pos, z_pos, q_x, q_y, q_z, q_w);
    ros::Duration(5).sleep();
    closeGripper();
    ros::Duration(5).sleep();
    attachObject(obj_id);
    ros::Duration(5).sleep();
    cartesianPlan(x_pos, y_pos, z_pos+0.15, q_x, q_y, q_z, q_w);
  }

  void placeObject(float x_pos, float y_pos, float z_pos, int obj_id){
    orientGraspVertical(x_pos, y_pos);
    cartesianPlan(x_pos, y_pos, z_pos+0.15, q_x, q_y, q_z, q_w);
    cartesianPlan(x_pos, y_pos, z_pos, q_x, q_y, q_z, q_w);
    openGripper();
    detachObject(obj_id);
    linearRetract(x_pos, y_pos, z_pos);
  }

  void simulateUserAndTable(){
    createObject("Sphere", 3, 0.45, -0.25, 0.55, 0.15, 0.1, 0.035, 0, 0, 0, 1);
    createObject("Box", 4, 0.45, -0.25, 0.15, 0.18, 0.4, 0.48, 0, 0, 0, 1);
    //createObject("Box", 5, -0.7, -0.35, 0.125, 0.75, 1.0, 0.05, 0, 0, 0, 1);
  }

  void pour(){
    moveSleep();
    ros::Duration(2).sleep();

    pickObject(b_x, b_y, b_z, 1);

    pourBottleAt(c_x, c_y, c_z);

    placeObject(b_x, b_y, b_z, 1);

    moveSleep();
  }

  void pick(int obj_id){
    if(obj_id == 1){
      pickObject(b_x, b_y, b_z, obj_id);
    }
    else if(obj_id == 2){
      pickObject(c_x, c_y, c_z, obj_id);
    }
    else{
      ROS_INFO("Unrecognized object");
    }
  }

  void place(float x_pos, float y_pos, float z_pos, int obj_id){
    placeObject(x_pos, y_pos, z_pos, obj_id);
    linearRetract(x_pos, y_pos, z_pos);
    moveSleep();
  }
};
