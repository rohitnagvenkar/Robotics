#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10000);

    const double degree = M_PI / 180;

    double inc = 0.005;
    double base_arm_inc = 0.005;
    double arm1_armbase_inc = 0.005;
    double arm2_arm1_inc = 0.005;
    double gripper_inc = 0.005;
    double tip_inc = 0.005;

    double angle = 0;
    double base_arm = 0;
    double arm1_armbase = 0;
    double gripper = 0;
    double tip = 0;
    double arm2_arm1 = 0;

    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "vehiclebase";

    while (ros::ok())
    {
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);

        joint_state.name[0] = "vehiclebase_to_armbase";
        joint_state.position[0] = arm1_armbase;

        joint_state.name[1] = "armstand_to_arm_1";
        joint_state.position[1] = arm1_armbase;

        joint_state.name[2] = "arm_1_to_arm_2";
        joint_state.position[2] = arm1_armbase;

        joint_state.name[3] = "arm_2_to_gripperbase";
        joint_state.position[3] = arm1_armbase;

        joint_state.name[4] = "gripper_socket_to_gripperbase";
        joint_state.position[4] = arm1_armbase;

        joint_state.name[5] = "gripperbase_to_gripper1";
        joint_state.position[5] = arm1_armbase;

        joint_state.name[6] = "gripperbase_to_gripper2";
        joint_state.position[6] = arm1_armbase;

        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle);
        odom_trans.transform.translation.y = sin(angle);
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        arm2_arm1 += arm2_arm1_inc;

        if (arm2_arm1 < -1.5 || arm2_arm1 > 1.5)
        {
            arm2_arm1_inc *= -1;
            arm1_armbase += arm1_armbase_inc;
        }
        if (arm1_armbase > 1.2 || arm1_armbase < -1.0)
        {
            arm1_armbase_inc *= -1;
            base_arm += base_arm_inc;
        }
        if (base_arm > 1 || base_arm < -1.0)
        {
            arm1_armbase_inc *= -1;
            gripper += gripper_inc;
        }
        if (gripper < 0 || gripper > 1)
        {
            gripper_inc *= -1;
            angle += degree / 4;
        }
        loop_rate.sleep();
    }
    return 0;
}