#include <iostream>
#include <math.h>


#include <ros/ros.h>
#include <tf/tf.h>
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
        ros::Publisher pub_marker_goal;
        // ros::Publisher pub_perception_pose;
        // ros::Subscriber sub_totem_gt;
        ros::Subscriber sub_current_pose;
        ros::Subscriber sub_wamv_pose;

        geometry_msgs::PoseStamped pub_msg_goal;
        
        // [7] = {position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z}
        float current_pose[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        float wamv_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // default param
        float camera_tilt_angle = 45; 
        float camera_vertical_FOV = 60;
        float camera_horizontal_FOV = 90;
        float camera_view_distance = 25;
        float camera_offset_x = 0.0;
        float camera_offset_y = 0.0;
        float camera_offset_z = 0.0;
        float cruise_height = 0.0;
        float duplicate_margin = 5.0;
        float goal_extend_length = 0.0;
        float evaluate_pair_threshold = 20;
        // default param

        XmlRpc::XmlRpcValue xml_green_totem;
        XmlRpc::XmlRpcValue xml_red_totem;

        // ground truth param for simulation
        float **boundary;
        float **red_totem_list_gt;
        float **green_totem_list_gt;
        bool *red_totem_status;
        bool *green_totem_status;
        int object_num = 0;
        int object_dim = 0;
        // ground truth param for simulation

        // real evaluation param 
        float current_goal[7] = {0, 0, 0, 0, 0, 0, 0};
        float memo_heading[2] = {0, 0};
        int red_totem_num = 0;
        int green_totem_num = 0;
        vector <vector<float>> red_totem_list;
        vector <vector<float>> green_totem_list;
        vector <vector<float>> goal_list;
        // real evaluation param 

        vector <float> debug;
        
    public:
        Perception();

        void getParam();
        void loadObject();
        void simPerception();

        void dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void wamvPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

        void vectorStructPush(float *input_array, vector <vector<float>> &append_vector);
        void vectorStructClear();

        void vectorP2P(float *p1, float *p2, float *vector);
        void vectorRotate(float *origin_vector, float angle, float *new_vector);
        float distanceP2P(float *p1, float *p2);
        float vector2distance(float *v);
        float innerProduct(float *v1, float *v2);
        float thetaCalculate(float *object_pose, float *end_point_1, float *end_point_2);
        
        bool isInFOV(float *object_pose);
        void boundaryGenerate(float height);
        void simPoseExtract(float **totem_list, int index, float *return_pose);

        void goalPointEval();
        void pairEval();
        void multiPairEval();
        void goalExtend(float *red_totem, float *green_totem, float *new_goal);
        void publishGoal();

        void boundaryVisualize();
        void objectVisualize();
        void goalPointVisualize();
        void pairVisualize();
};

Perception :: Perception(){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pub_marker_boundary = n.advertise<visualization_msgs::Marker>("drone_marker/fov_edge", 10);
    pub_marker_object = n.advertise<visualization_msgs::MarkerArray>("drone_marker/object", 10);
    pub_marker_goal = n.advertise<visualization_msgs::MarkerArray>("drone_marker/goal", 10);
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

void Perception :: vectorStructClear(){
    red_totem_list.clear();
    green_totem_list.clear();
    return;
}

void Perception :: vectorStructPush(float *input_array, vector <vector<float>> &append_vector){ 
    float exist_totem[3];
    for(int i = 0; i < append_vector.size(); i++){
        exist_totem[0] = append_vector[i][0];
        exist_totem[1] = append_vector[i][1];
        exist_totem[2] = append_vector[i][2];
        if(distanceP2P(exist_totem, input_array) < duplicate_margin){ 
            return;
        }
    }
    vector <float> p;
    for(int i = 0; i < 3; i++){
        p.push_back(input_array[i]);
    }
    append_vector.push_back(p);
    return;
}

float Perception :: innerProduct(float *v1, float *v2){
    return v1[0] * v2[0] + v1[1] * v2[1];
}

float Perception :: distanceP2P(float *p1, float *p2){
    return sqrt(powf(abs(p1[0] - p2[0]), 2) + powf(abs(p1[1] - p2[1]), 2));
}

float Perception :: vector2distance(float *v){
    return sqrt(powf(abs(v[0]), 2) + powf(abs(v[1]), 2));
} 

void Perception :: vectorP2P(float *p1, float *p2, float *vector){
    vector[0] = p2[0] - p1[0];
    vector[1] = p2[1] - p1[1];
    return; 
}

void Perception :: vectorRotate(float *origin_vector, float angle, float *new_vector){
    new_vector[0] = cos(angle * (PI / 180)) * origin_vector[0] - sin(angle * (PI / 180)) * origin_vector[1];
    new_vector[1] = sin(angle * (PI / 180)) * origin_vector[0] + cos(angle * (PI / 180)) * origin_vector[1];
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
    // detection for simulation (known ground truth)
    float tmp_totem[3];
    boundaryGenerate(current_pose[2]);
    for(int i = 0; i < object_num; i++){
        if(isInFOV(red_totem_list_gt[i])){ 
            red_totem_status[i] = true;
            vectorStructPush(red_totem_list_gt[i], red_totem_list);
        }
        else{ red_totem_status[i] = false;}

        if(isInFOV(green_totem_list_gt[i])){ 
            green_totem_status[i] = true; 
            vectorStructPush(green_totem_list_gt[i], green_totem_list);
        }
        else{ green_totem_status[i] = false; }
    }
    // detection for simulation (known ground truth)

    // detection based on real perception model
    // TODO
    // detection based on real perception model

    // evaluate based on perception result
    goalPointEval();
    // evaluate based on perception result

    // visualize
    objectVisualize();
    boundaryVisualize();
    goalPointVisualize();
    pairVisualize();
    // visualize

    // clear totem
    vectorStructClear();
    // clear totem

    return;
}

void Perception :: loadObject(){
    // real
    boundary = new float *[4];
    // real

    // virtual
    for(int i = 0; i < 4; i++){ boundary[i] = new float[3]; }

    red_totem_list_gt = new float *[object_num];
    red_totem_status = new bool [object_num];

    green_totem_list_gt = new float *[object_num];
    green_totem_status = new bool [object_num];


    for(int i = 0; i < object_num; i++){ red_totem_list_gt[i] = new float[object_dim]; }
    for(int i = 0; i < object_num; i++){ green_totem_list_gt[i] = new float[object_dim]; }
    
    for(int i = 0; i < object_num; i++){
        red_totem_status[i] = false;
        green_totem_status[i] = false;

        for(int j = 0; j < object_dim; j++){
            try{
                std::ostringstream out_str_red, out_str_green;
                out_str_red << xml_red_totem[i][j];
                out_str_green << xml_green_totem[i][j];

                std::istringstream in_str_red(out_str_red.str()), in_str_green(out_str_green.str());
                in_str_red >> red_totem_list_gt[i][j];
                in_str_green >> green_totem_list_gt[i][j];
            }
            catch(...){
                throw;
            }
        }
    }
    // virtual
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

    if(angle_inner_xz_plane < 0){
        // down-right (+, -)
        boundary[2][0] = current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
        boundary[2][1] = current_pose[1] + height / cos(angle_inner_xz_plane * (PI / 180))
                        * tan(angle_right_yz_plane * (PI / 180));
        boundary[2][2] = 0;
        // down-left (+, +)
        boundary[3][0] = current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
        boundary[3][1] = current_pose[1] - height / cos(angle_inner_xz_plane * (PI / 180))
                        * tan(angle_left_yz_plane * (PI / 180));
        boundary[3][2] = 0;
    }else{
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
    }
    

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
    red_totem_num = red_totem_list.size();
    green_totem_num = green_totem_list.size();
    
    cout << "red: " << red_totem_num << " ,green: " << green_totem_num << endl;

    if(red_totem_num == 0 && green_totem_num == 0){ cout << "no totem" << endl; }
    else if(red_totem_num == 1 && green_totem_num == 1){ pairEval(); }
    else if(red_totem_num == 0 || green_totem_num == 0){ cout << "no pair" << endl; }
    else if(red_totem_num >= 1 && green_totem_num >= 1){ multiPairEval(); }
    else{
        cout << "Exception orrur !" << endl;
    }
    publishGoal();
    return;
}

void Perception :: simPoseExtract(float **totem_list, int index, float *return_pose){
    return_pose[0] = totem_list[index][0];
    return_pose[1] = totem_list[index][1];
    return_pose[2] = totem_list[index][2];
    return;
}

void Perception :: pairEval(){
    cout << "pairEval" << endl; // 1 red totem, 1 green totem
    float red_totem[3], green_totem[3];

    red_totem[0] = red_totem_list[0][0];
    red_totem[1] = red_totem_list[0][1];
    red_totem[2] = red_totem_list[0][2];

    green_totem[0] = green_totem_list[0][0];
    green_totem[1] = green_totem_list[0][1];
    green_totem[2] = green_totem_list[0][2];

    cout << "distance: " << distanceP2P(red_totem, green_totem) << endl;
    goalExtend(red_totem, green_totem, current_goal);
    return;
}

void Perception :: multiPairEval(){
    cout << "multiPairEval" << endl; // 1+ red totem, 1+ green totem

    float mid_distance = 10000, current_distance = 0;
    float pair_vector[2], heading_vector[2], red_totem[3], green_totem[3], mid_point[3];
    int index[2];

    for(int i = 0; i < red_totem_list.size(); i++){
        for(int j = 0; j < green_totem_list.size(); j++){
            for(int k = 0; k < 3; k++){
                red_totem[k] = red_totem_list[i][k];
                green_totem[k] = green_totem_list[j][k];
            }
            if(distanceP2P(red_totem, green_totem) > evaluate_pair_threshold){ continue; }
            for(int i = 0; i < 2; i++){
                mid_point[i] = red_totem[i] + (green_totem[i] - red_totem[i]) / 2;
            }
            current_distance = distanceP2P(mid_point, current_pose);
            if(mid_distance > current_distance){
                index[0] = i;
                index[1] = j;
                mid_distance = current_distance;
            }
        }
    }
    red_totem[0] = red_totem_list[index[0]][0];
    red_totem[1] = red_totem_list[index[0]][1];
    red_totem[2] = red_totem_list[index[0]][2];

    green_totem[0] = green_totem_list[index[1]][0];
    green_totem[1] = green_totem_list[index[1]][1];
    green_totem[2] = green_totem_list[index[1]][2];
    goalExtend(red_totem, green_totem, current_goal);
    return;
}

void Perception :: goalExtend(float *red_totem, float *green_totem, float *new_goal){
    float v1[2], v2[2];
    vectorP2P(green_totem, red_totem, v1);
    vectorRotate(v1, -90, v2);

    for(int i = 0; i < 2; i++){
        debug.push_back(red_totem[i] + (green_totem[i] - red_totem[i]) / 2);
    }
    
    current_goal[0] = red_totem[0] + (green_totem[0] - red_totem[0]) / 2 + v2[0] * goal_extend_length / distanceP2P(green_totem, red_totem);
    current_goal[1] = red_totem[1] + (green_totem[1] - red_totem[1]) / 2 + v2[1] * goal_extend_length / distanceP2P(green_totem, red_totem);
    current_goal[2] = 0; 
    
    for(int i = 0; i < 2; i++){
        debug.push_back(current_goal[i]);
    }

    return;
}

void Perception :: publishGoal(){
    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.header.stamp = ros::Time::now();
    pub_msg_goal.pose.position.x = current_goal[0];
    pub_msg_goal.pose.position.y = current_goal[1];
    pub_msg_goal.pose.position.z = current_goal[2];

    pub_goalpoint.publish(pub_msg_goal);
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
        marker.pose.position.x = red_totem_list_gt[i][0];
        marker.pose.position.y = red_totem_list_gt[i][1];
        marker.pose.position.z = red_totem_list_gt[i][2];
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
        marker.pose.position.x = green_totem_list_gt[i][0];
        marker.pose.position.y = green_totem_list_gt[i][1];
        marker.pose.position.z = green_totem_list_gt[i][2];
        pub_msg_object.markers.push_back(marker);
    }

    pub_marker_object.publish(pub_msg_object);

    return;
}

void Perception :: boundaryVisualize(){
    visualization_msgs::Marker pub_msg_boundary;
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
    visualization_msgs::MarkerArray pub_msg_goal;

    if(debug.size() != 4){return;}

    visualization_msgs::Marker marker1;
    marker1.header.frame_id = "local_origin";
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "goal";
    marker1.id = 0;
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.pose.orientation.w = 1.0;
    marker1.type = visualization_msgs::Marker::SPHERE;
    marker1.scale.x = 1;
    marker1.scale.y = 1;
    marker1.scale.z = 1;
    marker1.color.r = 1;
    marker1.color.b = 1;
    marker1.color.a = 0.5;
    marker1.pose.position.x = debug[0];
    marker1.pose.position.y = debug[1];
    marker1.pose.position.z = 0;
    pub_msg_goal.markers.push_back(marker1);

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "local_origin";
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "goal";
    marker2.id = 1;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.orientation.w = 1.0;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.scale.x = 1;
    marker2.scale.y = 1;
    marker2.scale.z = 1;
    marker2.color.r = 1;
    marker2.color.b = 1;
    marker2.color.a = 0.5;
    marker2.pose.position.x = debug[2];
    marker2.pose.position.y = debug[3];
    marker2.pose.position.z = 0;
    pub_msg_goal.markers.push_back(marker2);

    debug.clear();
    pub_marker_goal.publish(pub_msg_goal);
    return;
}

void Perception :: pairVisualize(){
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
    n.getParam("/perception/cruise_height", cruise_height);
    n.getParam("/perception/duplicate_margin", duplicate_margin);
    n.getParam("/perception/goal_extend_length", goal_extend_length);
    n.getParam("/perception/evaluate_pair_threshold", evaluate_pair_threshold);
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

