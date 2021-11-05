//
// Created by accrea on 4/28/20.
//

#include "aria_driver/ros_control_udp.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv){
    constexpr int LOOP_HZ = 100;

    // Limit the maximal period to twice the nominal value.
    // This shall prevent ROS-controllers from using huge periods when
    // computing error correction.
    constexpr double SAFETY_PERIOD = 2.0 / LOOP_HZ;

    ros::init(argc, argv, "ros_control_udp");

    RosControlUDP rosControlUDP;
    controller_manager::ControllerManager controllerManager(&rosControlUDP);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate loop_rate(LOOP_HZ);



    while(rosControlUDP.nh_.ok()){
        const ros::Time time = ros::Time::now();
        const ros::Duration period(
            std::min(SAFETY_PERIOD, (time - prev_time).toSec())
        );
        if (AriaClient_isConnected() > 0){
            rosControlUDP.read();
            controllerManager.update(time, period);
            rosControlUDP.write();
        }
        prev_time = time;
        loop_rate.sleep();
    }

    AriaClient_StopCommunication();

    return 0;
}
