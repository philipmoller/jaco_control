#include "jaco_control.h"

int choice;
int object;
int place;

int main(int argc, char** argv){
  ros::init(argc, argv, "jaco_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  JacoControl JC;

  JC.simulateUserAndTable();
  ros::Duration(2).sleep();

  //Spawn Bottle
  JC.createObject("Cylinder", 1, b_x, b_y, b_z, 0.2, 0.035, 0.035, 0, 0, 0, 1);
  ros::Duration(2).sleep();
  //Spawn Cup
  JC.createObject("Cylinder", 2, c_x, c_y, c_z, 0.1, 0.05, 0.05, 0, 0, 0, 1);
  ros::Duration(2).sleep();



  JC.moveSleep();

  //JC.orientGraspVertical(b_x, b_y);
  //JC.cartesianPlan(b_x, b_x, b_z, q_x, q_y, q_z, q_w);



  while(ros::ok){
    ROS_INFO("Choose task:");
    ROS_INFO("1. Pick");
    ROS_INFO("2. Pour");
    ROS_INFO("0. Exit");
    std::cin >> choice;

    if(choice==1){
      ROS_INFO("Choose object:");
      ROS_INFO("1. Bottle");
      ROS_INFO("2. Cup");
      ROS_INFO("0. Exit");
      std::cin >> object;

      if(object == 1){
        //JC.pick(1);
        JC.orientGraspVertical(b_x, b_y);
        ros::Duration(2).sleep();
        JC.cartesianPlan(b_x, b_x, b_z, q_x, q_y, q_z, q_w);
        ros::shutdown();
        return 0;
      }

      else if(object == 2){
        JC.pick(2);
      }

      else if(object == 0){
        ROS_INFO("Exiting");
        break;
      }

      else{
        ROS_INFO("Unrecognized object");
        break;
      }
      ROS_INFO("Put down?");
      ROS_INFO("Enter 1 to place");
      std::cin >> place;
      if(place == 1 && object == 1){
        JC.place(b_x, b_y, b_z, object);

      }
      else if(place == 1 && object == 2){
        JC.place(c_x, c_y, c_z, object);
      }
      else{
        std::cin >> place;
      }
    }

    else if(choice==2){
      JC.pour();
    }


    else if(choice==0){
      ROS_INFO("Exiting");
      ros::shutdown();
      break;
    }



    else{
      ROS_INFO("Unrecognized input");
    }
  }


  ros::shutdown();
  return 0;
}
