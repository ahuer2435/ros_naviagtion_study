#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    double angle= 0;

    int test;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        if(test == 0.5)
        {
            test = 0;
        }
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] ="base_to_wheel1";
        joint_state.position[0] = 0.2;
        joint_state.name[1] ="base_to_wheel2";
        joint_state.position[1] = 0.5;
        joint_state.name[2] ="base_to_wheel3";
        joint_state.position[2] = 0.8;
        joint_state.name[3] ="base_to_wheel4";
        joint_state.position[3] = 1.0;

        test+=0.1;

        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle);
        odom_trans.transform.translation.y = sin(angle);
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        angle += degree;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
