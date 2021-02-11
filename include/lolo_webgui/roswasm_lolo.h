#ifndef ROSWASM_LOLO_H
#define ROSWASM_LOLO_H

#include <roswasm_webgui/roswasm_widget.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

//#include <lolo_msgs/ThrusterRPMs.h>
//#include <lolo_msgs/Leak.h>
#include <smarc_msgs/Leak.h>
#include <smarc_msgs/ThrusterRPM.h>

namespace roswasm_webgui {

//bool draw_ballast_angles(lolo_msgs::BallastAngles& msg, roswasm::Publisher& pub);
//bool draw_percent(lolo_msgs::PercentStamped& msg, roswasm::Publisher& pub);
//bool draw_thruster_rpms(lolo_msgs::ThrusterRPMs& msg, roswasm::Publisher& pub);
bool draw_thruster_rpm(smarc_msgs::ThrusterRPM& msg, roswasm::Publisher& pub);
//bool draw_thruster_angles(lolo_msgs::ThrusterAngles& msg, roswasm::Publisher& pub);

class LoloActuatorWidget {
private:
    //TopicWidget<lolo_msgs::ThrusterAngles>* thruster_angles;
    TopicWidget<std_msgs::Float32>* rudder_angles;
    TopicWidget<std_msgs::Float32>* elevator_angle;
    TopicWidget<std_msgs::Float32>* elevon_port_angle;
    TopicWidget<std_msgs::Float32>* elevon_stbd_angle;
    TopicWidget<smarc_msgs::ThrusterRPM>* thruster1_rpm;
    TopicWidget<smarc_msgs::ThrusterRPM>* thruster2_rpm;
    roswasm::Publisher rpm1_pub;
    roswasm::Publisher rpm2_pub;
    roswasm::Timer pub_timer;
    bool rpm_pub_enabled;
public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_actuator_window);
    LoloActuatorWidget(roswasm::NodeHandle& nh);
};

class LoloDashboardWidget {
private:
    bool was_leak;
    TopicBuffer<smarc_msgs::Leak>* leak;
    TopicBuffer<sensor_msgs::NavSatFix>* gps;
    TopicBuffer<sensor_msgs::BatteryState>* battery;
    TopicBuffer<nav_msgs::Odometry>* odom;
    //TopicBuffer<lolo_msgs::ThrusterRPMs>* rpms;
    TopicBuffer<smarc_msgs::ThrusterRPM>* rpm1;
    TopicBuffer<smarc_msgs::ThrusterRPM>* rpm2;
    TopicBuffer<std_msgs::Float64>* depth;
    TopicBuffer<std_msgs::Float64>* pitch;
    TopicBuffer<std_msgs::Float64>* roll;
    TopicBuffer<std_msgs::Float64>* yaw;
public:
    bool is_emergency() { return was_leak; }
    void show_window(bool& show_dashboard_window);
    LoloDashboardWidget(roswasm::NodeHandle& nh);
};

class LoloTeleopWidget {
private:
    bool enabled;
    //lolo_msgs::ThrusterAngles angles_msg;
    std_msgs::Float32 rudder_msg;
    std_msgs::Float32 elevon_msg;
    smarc_msgs::ThrusterRPM rpm1_msg;
    smarc_msgs::ThrusterRPM rpm2_msg;
    roswasm::Publisher rpm1_pub;
    roswasm::Publisher rpm2_pub;
    roswasm::Publisher rudder_pub;
    roswasm::Publisher elevator_pub;
    roswasm::Publisher elevon_port_pub;
    roswasm::Publisher elevon_stbd_pub;
    roswasm::Timer pub_timer;
public:
    void pub_callback(const ros::TimerEvent& e);
    void show_window(bool& show_teleop_window);
    LoloTeleopWidget(roswasm::NodeHandle& nh);
};

} // namespace roswasm_webgui

#endif // ROSWASM_EXAMPLES_H
