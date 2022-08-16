#include <iostream>
#include <math.h>
#include <tf/tf.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265

using namespace std; 

class Perception{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_goalpoint;
        ros::Publisher pub_marker_boundary;
        ros::Publisher pub_marker_object;
        // ros::Publisher pub_perception_pose;
        // ros::Subscriber sub_totem_gt;
        ros::Subscriber sub_current_pose;
        ros::Subscriber sub_wamv_pose;

        geometry_msgs::PoseStamped pub_msg_goal;
        
        // [7] = {position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z}
        float current_pose[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        float wamv_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //default
        float camera_tilt_angle = 45; 
        float camera_vertical_FOV = 60;
        float camera_horizontal_FOV = 90;
        float camera_view_distance = 25;
        float camera_offset_x = 0.0;
        float camera_offset_y = 0.0;
        float camera_offset_z = 0.0;

        XmlRpc::XmlRpcValue xml_green_totem;
        XmlRpc::XmlRpcValue xml_red_totem;
        float **boundary;
        float **red_totem_list;
        float **green_totem_list;
        bool *red_totem_status;
        bool *green_totem_status;
        int object_num = 0;
        int object_dim = 0;
        //default
    public:
        Perception();

        void getParam();
        void loadObject();
        void simPerception();

        void dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void wamvPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void vectorP2P(float *p1, float *p2, float *vector);
        float distanceP2P(float *p1, float *p2);
        float innerProduct(float *v1, float *v2);
        float thetaCalculate(float *object_pose, float *end_point_1, float *end_point_2);
        
        bool isInFOV(float *object_pose);
        void boundaryGenerate(float height);
        void goalPointEval();
        void boundaryVisualize();
        void objectVisualize();
        void goalPointVisualize();
};

Perception :: Perception(){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pub_marker_boundary = n.advertise<visualization_msgs::Marker>("drone_marker/fov_edge", 10);
    pub_marker_object = n.advertise<visualization_msgs::MarkerArray>("drone_marker/object", 10);
    // pub_totem_pose = n.advertise<::>("", 10);
    // sub_totem_gt = n.subscribe<::>("", 1,  &Perception::, this);
    sub_current_pose = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1,  &Perception::dronePositionCallback, this);
    // sub_current_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Perception::dronePositionCallback, this);
    sub_wamv_pose = n.subscribe<geometry_msgs::PoseStamped>("/landing_area", 1,  &Perception::wamvPositionCallback, this);
}

void Perception :: dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose[0] =  msg->pose.position.x + camera_offset_x;
    current_pose[1] =  msg->pose.position.y + camera_offset_y;
    current_pose[2] =  msg->pose.position.z + camera_offset_z;
    current_pose[3] =  msg->pose.orientation.x;
    current_pose[4] =  msg->pose.orientation.y;
    current_pose[5] =  msg->pose.orientation.z;
    current_pose[6] =  msg->pose.orientation.w;
    return;
}

void Perception :: wamvPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    wamv_pose[0] =  msg->pose.position.x;
    wamv_pose[1] =  msg->pose.position.y;
    wamv_pose[2] =  msg->pose.position.z;
    wamv_pose[3] =  msg->pose.orientation.x;
    wamv_pose[4] =  msg->pose.orientation.y;
    wamv_pose[5] =  msg->pose.orientation.z;
    wamv_pose[6] =  msg->pose.orientation.w;
    return;
}

float Perception :: innerProduct(float *v1, float *v2){
    return v1[0] * v2[0] + v1[1] * v2[1];
}

float Perception :: distanceP2P(float *p1, float *p2){
    return sqrt(powf(abs(p1[0] - p2[0]), 2) + powf(abs(p1[1] - p2[1]), 2));
}

void Perception :: vectorP2P(float *p1, float *p2, float *vector){
    vector[0] = p2[0] - p1[0];
    vector[1] = p2[1] - p1[1];
    return; 
}

float Perception :: thetaCalculate(float *object_pose, float *end_point_1, float *end_point_2){
    float v1[2], v2[2];
    vectorP2P(object_pose, end_point_1, v1);
    vectorP2P(object_pose, end_point_2, v2);
    return acos(innerProduct(v1, v2) / distanceP2P(object_pose, end_point_1) / distanceP2P(object_pose, end_point_2)) * (180 / PI);
}

bool Perception :: isInFOV(float *object_pose){
    float theta1 = 0.0, theta2 = 0.0, theta3 = 0.0, theta4 = 0.0;
    theta1 = thetaCalculate(object_pose, boundary[0], boundary[1]);
    theta2 = thetaCalculate(object_pose, boundary[1], boundary[2]);
    theta3 = thetaCalculate(object_pose, boundary[2], boundary[3]);
    theta4 = thetaCalculate(object_pose, boundary[3], boundary[0]);
    // cout << "angle: " << theta1 << " " << theta2 << " " << theta3 << " " << theta4 << endl;
    // cout << "sum: " << theta1 + theta2 + theta3 + theta4 << endl;
    if(abs(theta1 + theta2 + theta3 + theta4 - 360) < 1){
        return true;
    }
    else{
        return false;
    }
}

void Perception :: simPerception(){
    boundaryGenerate(current_pose[2]);

    for(int i = 0; i < object_num; i++){
        if(isInFOV(red_totem_list[i])){ red_totem_status[i] = true;}
        else{ red_totem_status[i] = false;}

        if(isInFOV(green_totem_list[i])){ green_totem_status[i] = true; }
        else{ green_totem_status[i] = false; }
    }
    goalPointEval();
    
    objectVisualize();
    boundaryVisualize();
    goalPointVisualize();

    return;
}

void Perception :: loadObject(){
    boundary = new float *[4];
    for(int i = 0; i < 4; i++){ boundary[i] = new float[3]; }

    red_totem_list = new float *[object_num];
    red_totem_status = new bool [object_num];

    green_totem_list = new float *[object_num];
    green_totem_status = new bool [object_num];

    for(int i = 0; i < object_num; i++){ red_totem_list[i] = new float[object_dim]; }
    for(int i = 0; i < object_num; i++){ green_totem_list[i] = new float[object_dim]; }
    
    for(int i = 0; i < object_num; i++){
        red_totem_status[i] = false;
        green_totem_status[i] = false;

        for(int j = 0; j < object_dim; j++){
            try{
                std::ostringstream out_str_red, out_str_green;
                out_str_red << xml_red_totem[i][j];
                out_str_green << xml_green_totem[i][j];

                std::istringstream in_str_red(out_str_red.str()), in_str_green(out_str_green.str());
                in_str_red >> red_totem_list[i][j];
                in_str_green >> green_totem_list[i][j];
            }
            catch(...){
                throw;
            }
        }
    }

    return;
}

void Perception :: boundaryGenerate(float height){
    float angle_inner_xz_plane, angle_outer_xz_plane, angle_right_yz_plane, angle_left_yz_plane;
    double roll, pitch, yaw;
    tf::Quaternion q(
        current_pose[3],
        current_pose[4],
        current_pose[5],
        current_pose[6]);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    angle_inner_xz_plane = 90 - camera_tilt_angle - camera_vertical_FOV / 2 - (float)(pitch * 180 / PI);
    angle_outer_xz_plane = 90 - camera_tilt_angle + camera_vertical_FOV / 2 - (float)(pitch * 180 / PI);
    angle_right_yz_plane = camera_horizontal_FOV / 2 - (float)(roll * 180 / PI);
    angle_left_yz_plane = camera_horizontal_FOV / 2 + (float)(roll * 180 / PI);

    cout << "inner: " << angle_inner_xz_plane << " outer: " << angle_outer_xz_plane << endl;
    cout << "left: " << angle_left_yz_plane << " right: " << angle_right_yz_plane << endl;

    // calculate roll + pitch
    if(angle_outer_xz_plane >= 90){ // limit by camera view distance
        // up-left (+, +)
        boundary[0][0] = current_pose[0] + camera_view_distance * cos(angle_outer_xz_plane - 90);
        boundary[0][1] = current_pose[1] + camera_view_distance * tan(angle_left_yz_plane * (PI / 180));
        boundary[0][2] = 0;
        // up-right (+, -)
        boundary[1][0] = current_pose[0] + camera_view_distance * cos(angle_outer_xz_plane - 90);
        boundary[1][1] = current_pose[1] - camera_view_distance * tan(angle_right_yz_plane * (PI / 180));
        boundary[1][2] = 0;
    }else{
        // up-left (+, +)
        boundary[0][0] = current_pose[0] + height * tan(angle_outer_xz_plane * (PI / 180));
        boundary[0][1] = current_pose[1] + height / cos(angle_outer_xz_plane * (PI / 180))
                        * tan(angle_left_yz_plane * (PI / 180));
        boundary[0][2] = 0;
        // up-right (+, -)
        boundary[1][0] = current_pose[0] + height * tan(angle_outer_xz_plane * (PI / 180));
        boundary[1][1] = current_pose[1] - height / cos(angle_outer_xz_plane * (PI / 180))
                        * tan(angle_right_yz_plane * (PI / 180));
        boundary[1][2] = 0;
    }
    // down-right (+, -)
    boundary[2][0] = current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
    boundary[2][1] = current_pose[1] - height / cos(angle_inner_xz_plane * (PI / 180))
                     * tan(angle_right_yz_plane * (PI / 180));
    boundary[2][2] = 0;
    // down-left (+, +)
    boundary[3][0] = current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
    boundary[3][1] = current_pose[1] + height / cos(angle_inner_xz_plane * (PI / 180))
                     * tan(angle_left_yz_plane * (PI / 180));
    boundary[3][2] = 0;

    // calculate yaw 

    float **tmp = new float *[4];
    for(int i = 0; i < 4; i++){ 
        tmp[i] = new float[3];
    }
    for(int i = 0; i < 4; i++){ 
        for(int j = 0; j < 3; j++){
            tmp[i][j] = boundary[i][j];
        }
    }
    boundary[0][0] = current_pose[0] + distanceP2P(tmp[0], current_pose)
                     * cos(atan((tmp[0][1] - current_pose[1]) / (tmp[0][0] - current_pose[0]))
                     + yaw);
    boundary[0][1] = current_pose[1] + distanceP2P(tmp[0], current_pose)
                     * sin(atan((tmp[0][1] - current_pose[1]) / (tmp[0][0] - current_pose[0]))
                     + yaw);
    boundary[1][0] = current_pose[0] + distanceP2P(tmp[1], current_pose)
                     * cos(atan((tmp[1][1] - current_pose[1]) / (tmp[1][0] - current_pose[0]))
                     + yaw);
    boundary[1][1] = current_pose[1] + distanceP2P(tmp[1], current_pose)
                     * sin(atan((tmp[1][1] - current_pose[1]) / (tmp[1][0] - current_pose[0]))
                     + yaw);
    boundary[2][0] = current_pose[0] + distanceP2P(tmp[2], current_pose)
                     * cos(atan((tmp[2][1] - current_pose[1]) / (tmp[2][0] - current_pose[0]))
                     + yaw);
    boundary[2][1] = current_pose[1] + distanceP2P(tmp[2], current_pose)
                     * sin(atan((tmp[2][1] - current_pose[1]) / (tmp[2][0] - current_pose[0]))
                     + yaw);
    boundary[3][0] = current_pose[0] + distanceP2P(tmp[3], current_pose)
                     * cos(atan((tmp[3][1] - current_pose[1]) / (tmp[3][0] - current_pose[0]))
                     + yaw);
    boundary[3][1] = current_pose[1] + distanceP2P(tmp[3], current_pose)
                     * sin(atan((tmp[3][1] - current_pose[1]) / (tmp[3][0] - current_pose[0]))
                     + yaw);
    
    for(int i = 0; i < 4; i++){ delete [] tmp[i]; }
    delete[] tmp;

    return;
}

void Perception :: goalPointEval(){
    int red_totem_num = 0, green_totem_num = 0;
    for(int i = 0; i < object_num; i++){
        red_totem_num+=red_totem_status[i];
        green_totem_num+=green_totem_status[i];
    }
    // cout << "red: " << red_totem_num << " ,green: " << green_totem_num << endl;
    return;
}

void Perception :: objectVisualize(){
    visualization_msgs::MarkerArray pub_msg_object;

    for (int i = 0; i < object_num; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "object";
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        // red
        marker.color.r = red_totem_status[i];
        marker.color.b = ~red_totem_status[i];
        marker.color.a = 1.0;
        marker.pose.position.x = red_totem_list[i][0];
        marker.pose.position.y = red_totem_list[i][1];
        marker.pose.position.z = red_totem_list[i][2];
        pub_msg_object.markers.push_back(marker);
    }

    for (int i = 0; i < object_num; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "object";
        marker.id = i + object_num;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        // green  
        marker.color.g = green_totem_status[i];
        marker.color.b = ~green_totem_status[i];
        marker.color.a = 1.0;
        marker.pose.position.x = green_totem_list[i][0];
        marker.pose.position.y = green_totem_list[i][1];
        marker.pose.position.z = green_totem_list[i][2];
        pub_msg_object.markers.push_back(marker);
    }

    pub_marker_object.publish(pub_msg_object);

    return;
}

void Perception :: boundaryVisualize(){
    visualization_msgs::Marker pub_msg_boundary;

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            cout << boundary[i][j] << " ";
        }
        cout << endl;
    }

    pub_msg_boundary.header.frame_id = "local_origin";
    pub_msg_boundary.header.stamp = ros::Time::now();
    pub_msg_boundary.ns = "fov";
    pub_msg_boundary.action = visualization_msgs::Marker::ADD;
    pub_msg_boundary.pose.orientation.w = 1.0;
    pub_msg_boundary.type = visualization_msgs::Marker::LINE_STRIP;
    pub_msg_boundary.scale.x = 0.1;
    pub_msg_boundary.scale.y = 0.1;
    pub_msg_boundary.scale.z = 0.1;
    pub_msg_boundary.color.b = 1.0;
    pub_msg_boundary.color.a = 1.0;

    for (int i = 0; i < 4; i++){
        geometry_msgs::Point p;
        p.x = boundary[i][0];
        p.y = boundary[i][1];
        p.z = boundary[i][2];
        pub_msg_boundary.points.push_back(p);
    }

    geometry_msgs::Point p;
    p.x = boundary[0][0];
    p.y = boundary[0][1];
    p.z = boundary[0][2];
    pub_msg_boundary.points.push_back(p);

    pub_marker_boundary.publish(pub_msg_boundary);
    return;
}

void Perception :: goalPointVisualize(){

    return;
}

void Perception :: getParam(){
    n.getParam("/perception/camera_tilt_angle", camera_tilt_angle);
    n.getParam("/perception/camera_vertical_FOV", camera_vertical_FOV);
    n.getParam("/perception/camera_horizontal_FOV", camera_horizontal_FOV);
    n.getParam("/perception/red_totem", xml_red_totem);
    n.getParam("/perception/green_totem", xml_green_totem);
    n.getParam("/perception/camera_offset_x", camera_offset_x);
    n.getParam("/perception/camera_offset_y", camera_offset_y);
    n.getParam("/perception/camera_offset_z", camera_offset_z);
    n.getParam("/perception/camera_view_distance", camera_view_distance);
    object_num = xml_red_totem.size();
    object_dim = xml_red_totem[0].size();
    return;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "simulate_perception");
    Perception uav;
    uav.getParam();
    uav.loadObject();

    while(ros::ok()){
        uav.simPerception();
        ros::spinOnce();
    }
    return 0;
}
