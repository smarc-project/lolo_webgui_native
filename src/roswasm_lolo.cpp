#include <lolo_webgui/roswasm_lolo.h>

#include <roswasm_webgui/imgui/imgui.h>

namespace roswasm_webgui {

bool draw_thruster_rpm(smarc_msgs::ThrusterRPM& msg, roswasm::Publisher& pub)
{
    ImGui::PushID("RPM cmd slider");
    ImGui::SliderInt("", &msg.rpm, -2000, 2000, "%drpm");
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }
    bool lock = ImGui::IsItemActive();

    ImGui::PopID();
    ImGui::SameLine();
    ImGui::InputInt("RPM cmd input", &msg.rpm);
    if (ImGui::IsItemDeactivatedAfterChange()) {
        pub.publish(msg);
    }

    return lock;
}

LoloActuatorWidget::LoloActuatorWidget(roswasm::NodeHandle& nh) : rpm_pub_enabled(false)
{
    //thruster_angles = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Hori (rad)", -0.1, 0.18, "Vert (rad)", -0.1, 0.15), "core/thrust_vector_cmd", "core/thrust_fb1", "core/thrust_fb2");
    //thruster_rpms = new TopicPairWidget<geometry_msgs::Pose2D, std_msgs::Float64>(nh, DrawFloatPair("Thruster 1", -1000., 1000., "Thruster 2", -1000., 1000.), "core/rpm_cmd", "core/rpm_fb1", "core/rpm_fb2");
    //thruster_angles = new TopicWidget<lolo_msgs::ThrusterAngles>(nh, &draw_thruster_angles, "core/thrust_vector_cmd");
    rudder_angles = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-.6, .6), "core/rudder_cmd");
    elevator_angle = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-.6, .6), "core/elevator_cmd");
    elevon_port_angle = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-.6, .6), "core/elevon_port_cmd");
    elevon_stbd_angle = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(-.6, .6), "core/elevon_strb_cmd");

    //vbs_front_port = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "core/vbs_front_port_cmd", "core/vbs_front_port_fb");
    //vbs_front_stbd = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "core/vbs_front_stbd_cmd", "core/vbs_front_stbd_fb");
    //vbs_back_port = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "core/vbs_back_port_cmd", "core/vbs_back_port_fb");
    //vbs_back_stbd = new TopicWidget<std_msgs::Float32>(nh, DrawFloat32(0., 100.), "core/vbs_back_stbd_cmd", "core/vbs_back_stbd_fb");

    //thruster_rpms = new TopicWidget<lolo_msgs::ThrusterRPMs>(nh, &draw_thruster_rpms, "core/rpm_cmd", "core/rpm_fb");
    thruster1_rpm = new TopicWidget<smarc_msgs::ThrusterRPM>(nh, &draw_thruster_rpm, "core/thruster1_cmd");
    thruster2_rpm = new TopicWidget<smarc_msgs::ThrusterRPM>(nh, &draw_thruster_rpm, "core/thruster2_cmd");
    rpm1_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster1_cmd", 1000);
    rpm2_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster2_cmd", 1000);

    pub_timer = nh.createTimer(roswasm::Duration(0.08), std::bind(&LoloActuatorWidget::pub_callback, this, std::placeholders::_1));
    pub_timer.stop();
}

void LoloActuatorWidget::pub_callback(const ros::TimerEvent& e)
{
    if (rpm_pub_enabled) {
        rpm1_pub.publish(thruster1_rpm->get_msg());
        rpm2_pub.publish(thruster2_rpm->get_msg());
    }
}

void LoloActuatorWidget::show_window(bool& show_actuator_window)
{
    ImGui::Begin("Actuator controls", &show_actuator_window);

    if (ImGui::CollapsingHeader("Rudder Angles", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("RPMs");
        ImGui::PushID("First");
        ImGui::Text("Rudders");
        ImGui::SameLine();
        rudder_angles->show_widget();
        ImGui::PopID();
        ImGui::PushID("Second");
        ImGui::Text("Elevator");
        ImGui::SameLine();
        elevator_angle->show_widget();
        ImGui::PopID();
        ImGui::PushID("Third");
        ImGui::Text("Port Elevon");
        ImGui::SameLine();
        elevon_port_angle->show_widget();
        ImGui::PopID();
        ImGui::PushID("Fourth");
        ImGui::Text("Starboard Elevon");
        ImGui::SameLine();
        elevon_stbd_angle->show_widget();
        ImGui::PopID();
        ImGui::PopID();
    }
    if (ImGui::CollapsingHeader("Thruster RPMs", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("RPMs");
        ImGui::PushID("First");
        ImGui::Text("Thruster 1");
        ImGui::SameLine();
        thruster1_rpm->show_widget();
        ImGui::PopID();
        ImGui::PushID("Second");
        ImGui::Text("Thruster 2");
        ImGui::SameLine();
        thruster2_rpm->show_widget();
        ImGui::PopID();
        ImGui::Checkbox("Publish RPMs at 10hz", &rpm_pub_enabled);
        ImGui::PopID();
        if (rpm_pub_enabled) { // && pub_timer == nullptr) {
            pub_timer.start();
        }
        else { //if (!rpm_pub_enabled && pub_timer != nullptr) {
            pub_timer.stop();
        }
    }
    //if (ImGui::CollapsingHeader("VBS", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        //ImGui::PushID("VBS");
        //ImGui::PushID("First");
        //ImGui::Text("Front port");
        //ImGui::SameLine();
        //vbs_front_port->show_widget();
        //ImGui::PopID();
        //ImGui::PushID("Second");
        //ImGui::Text("Front starboard");
        //ImGui::SameLine();
        //vbs_front_stbd->show_widget();
        //ImGui::PopID();
        //ImGui::PushID("Third");
        //ImGui::Text("Back port");
        //ImGui::SameLine();
        //vbs_back_port->show_widget();
        //ImGui::PopID();
        //ImGui::PushID("Fourth");
        //ImGui::Text("Back starboard");
        //ImGui::SameLine();
        //vbs_back_stbd->show_widget();
        //ImGui::PopID();
        //ImGui::PopID();
    //}

    ImGui::End();
}

LoloDashboardWidget::LoloDashboardWidget(roswasm::NodeHandle& nh) : was_leak(false)
{
    leak = new TopicBuffer<smarc_msgs::Leak>(nh, "core/leak_fb");
    gps = new TopicBuffer<sensor_msgs::NavSatFix>(nh, "core/gps");
    battery = new TopicBuffer<sensor_msgs::BatteryState>(nh, "core/battery");
    odom = new TopicBuffer<nav_msgs::Odometry>(nh, "dr/odom", 1000);
    //rpms = new TopicBuffer<lolo_msgs::ThrusterRPMs>(nh, "core/rpm_fb", 1000);
    rpm1 = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster1_cmd", 1000);
    rpm2 = new TopicBuffer<smarc_msgs::ThrusterRPM>(nh, "core/thruster2_cmd", 1000);
    depth = new TopicBuffer<std_msgs::Float64>(nh, "dr/depth", 1000);
    pitch = new TopicBuffer<std_msgs::Float64>(nh, "dr/pitch", 1000);
    roll = new TopicBuffer<std_msgs::Float64>(nh, "dr/roll", 1000);
    yaw = new TopicBuffer<std_msgs::Float64>(nh, "dr/yaw", 1000);
}

void LoloDashboardWidget::show_window(bool& show_dashboard_window)
{
    ImGui::Begin("Status dashboard", &show_dashboard_window);

    if (ImGui::CollapsingHeader("Critical Info", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        was_leak = was_leak || leak->get_msg().value;

        float sz = ImGui::GetTextLineHeight();
        std::string status_text;
        ImColor status_color;
        if (!was_leak) {
            status_text = "No leaks!";
            status_color = ImColor(0, 255, 0);
        }
        else {
            status_text = "Leak!!!!!";
            status_color = ImColor(255, 0, 0);
        }
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(p, ImVec2(p.x+sz, p.y+sz), status_color);
        ImGui::Dummy(ImVec2(sz, sz));
        ImGui::SameLine();
        ImGui::Text("%s", status_text.c_str());

        ImGui::SameLine(150);
        ImGui::Text("Battery: %.0f%%", battery->get_msg().percentage);
    }

    if (ImGui::CollapsingHeader("GPS and depth", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Lat: %.5f", gps->get_msg().latitude);
        ImGui::SameLine(150);
        ImGui::Text("Lon: %.5f", gps->get_msg().longitude);
        ImGui::SameLine(300);
        ImGui::Text("Depth: %.2fm", depth->get_msg().data);
    }

    if (ImGui::CollapsingHeader("DR translation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("X: %.2fm", odom->get_msg().pose.pose.position.x);
        ImGui::SameLine(150);
        ImGui::Text("Y: %.2fm", odom->get_msg().pose.pose.position.y);
        ImGui::SameLine(300);
        ImGui::Text("Z: %.2fm", odom->get_msg().pose.pose.position.z);
    }

    if (ImGui::CollapsingHeader("DR rotation", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Roll: %.2fdeg", 180./M_PI*roll->get_msg().data);
        ImGui::SameLine(150);
        ImGui::Text("Pitch: %.2fdeg", 180./M_PI*pitch->get_msg().data);
        ImGui::SameLine(300);
        ImGui::Text("Yaw: %.2fdeg", 180./M_PI*yaw->get_msg().data);
    }

    if (ImGui::CollapsingHeader("Actuator feedback", ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("RPMs: %d, %d rpm", rpm1->get_msg().rpm, rpm2->get_msg().rpm);
    }

    ImGui::End();
}

LoloTeleopWidget::LoloTeleopWidget(roswasm::NodeHandle& nh) : enabled(false) //, pub_timer(nullptr)
{
    rudder_pub = nh.advertise<std_msgs::Float32>("core/rudder_cmd", 1000);
    elevator_pub = nh.advertise<std_msgs::Float32>("core/elevator_cmd", 1000);
    elevon_port_pub = nh.advertise<std_msgs::Float32>("core/elevon_port_cmd", 1000);
    elevon_stbd_pub = nh.advertise<std_msgs::Float32>("core/elevon_strb_cmd", 1000);
    rpm1_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster1_cmd", 1000);
    rpm2_pub = nh.advertise<smarc_msgs::ThrusterRPM>("core/thruster2_cmd", 1000);
    pub_timer = nh.createTimer(roswasm::Duration(0.08), std::bind(&LoloTeleopWidget::pub_callback, this, std::placeholders::_1));
    pub_timer.stop();
}

void LoloTeleopWidget::pub_callback(const ros::TimerEvent& e)
{
    if (enabled) {
        //angle_pub.publish(angles_msg);
        elevon_port_pub.publish(elevon_msg);
        //elevon_msg.data *= -1.;
        elevon_stbd_pub.publish(elevon_msg);
        elevator_pub.publish(elevon_msg);
        //elevon_msg.data *= -1.;
        rpm1_pub.publish(rpm1_msg);
        rpm2_pub.publish(rpm2_msg);
        rudder_pub.publish(rudder_msg);
    }
}

void LoloTeleopWidget::show_window(bool& show_teleop_window)
{
    ImGuiIO& io = ImGui::GetIO();
    ImVec4 col = ImGui::GetStyle().Colors[23];

    ImGui::SetNextWindowSize(ImVec2(472, 80), ImGuiCond_FirstUseEver);
    ImGui::Begin("Keyboard teleop", &show_teleop_window);

    rudder_msg.data = elevon_msg.data = rpm1_msg.rpm = rpm2_msg.rpm = 0.0f;

    float sz = ImGui::GetTextLineHeight();
    ImGui::BeginGroup();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 0.75f*sz));
    ImGui::Checkbox("Teleop enabled", &enabled);
    if (enabled) {
        pub_timer.start();
    }
    else { //if (!enabled && pub_timer != nullptr) {
        pub_timer.stop();
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    bool key_down = enabled && io.KeysDownDuration[263] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##left", ImGuiDir_Left) || key_down) {
        //angles_msg.thruster_horizontal_radians = -0.10;
        rudder_msg.data = -0.6;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[265] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##up", ImGuiDir_Up) || key_down) {
        //angles_msg.thruster_vertical_radians = 0.10;
        elevon_msg.data = -0.6;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[264] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##down", ImGuiDir_Down) || key_down) {
        //angles_msg.thruster_vertical_radians = -0.10;
        elevon_msg.data = 0.6;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, 1.5f*sz));
    key_down = enabled && io.KeysDownDuration[262] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::ArrowButton("##right", ImGuiDir_Right) || key_down) {
        //angles_msg.thruster_horizontal_radians = 0.10;
        rudder_msg.data = 0.6;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    ImGui::Dummy(ImVec2(sz, sz));
    ImGui::Text("Thrust Vector");
    ImGui::EndGroup();

    ImGui::SameLine();

    ImGui::BeginGroup();
    key_down = enabled && io.KeysDownDuration[87] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("w") || key_down) {
        rpm1_msg.rpm = rpm2_msg.rpm = 1000;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    key_down = enabled && io.KeysDownDuration[83] >= 0.0f;
    if (key_down) {
        ImGui::PushStyleColor(ImGuiCol_Button, col);
    }
    if (ImGui::Button("s") || key_down) {
        rpm1_msg.rpm = rpm2_msg.rpm = -1000;
    }
    if (key_down) {
        ImGui::PopStyleColor();   
    }
    ImGui::EndGroup();
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("Forward");
    ImGui::Dummy(ImVec2(sz, 0.5f*sz));
    ImGui::Text("Reverse");
    ImGui::EndGroup();
    ImGui::EndGroup();

    ImGui::End();

    //ImGui::Text("Keys down:");      for (int i = 0; i < IM_ARRAYSIZE(io.KeysDown); i++) if (io.KeysDownDuration[i] >= 0.0f)     { ImGui::SameLine(); ImGui::Text("%d (%.02f secs)", i, io.KeysDownDuration[i]); }
}


} // namespace roswasm_webgui
