#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <behavior_tree/behavior_tree.h>

#define PI 3.14159265

using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}

class Perception{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_goalpoint;
        ros::Publisher pub_marker_boundary;
        ros::Publisher pub_marker_object;
        ros::Publisher pub_marker_goal;
        ros::Publisher pub_pair_state;

        ros::Subscriber sub_clear_signal;
        ros::Subscriber sub_current_pose;
        ros::Subscriber sub_wamv_pose;

        geometry_msgs::PoseStamped pub_msg_goal;
        
        
        // default param
        float m_camera_tilt_angle = 45; 
        float m_camera_vertical_FOV = 60;
        float m_camera_horizontal_FOV = 90;
        float m_camera_view_distance = 25;
        float m_camera_offset_x = 0.0;
        float m_camera_offset_y = 0.0;
        float m_camera_offset_z = 0.0;
        float m_duplicate_margin_object = 5.0;
        float m_duplicate_margin_goal = 5.0;
        float m_goal_extend_length = 0.0;
        float m_evaluate_pair_threshold = 20;
        // default param

        XmlRpc::XmlRpcValue m_xml_green_totem;
        XmlRpc::XmlRpcValue m_xml_red_totem;
        XmlRpc::XmlRpcValue m_xml_white_totem;

        // ground truth param for simulation
        float **m_boundary;
        float **m_red_totem_list_gt;
        float **m_green_totem_list_gt;
        float **m_white_totem_list_gt;

        bool *m_red_totem_status;
        bool *m_green_totem_status;
        bool *m_white_totem_status;

        int m_red_num_gt = 0;
        int m_green_num_gt = 0;
        int m_white_num_gt = 0;

        int object_dim = 0;
        // ground truth param for simulation

        // real evaluation param 

        // [7] = {position.x, position.y, position.z, orientation.w, orientation.x, orientation.y, orientation.z}
        float m_current_pose[7] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
        float m_wamv_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float m_current_goal[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float m_current_goal_backup[7] = {-10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // m_phase 1 = find entry white pair
        // m_phase 2 = follow path
        // m_phase 3 = find exit white pair
        int m_phase = 1;

        int m_pair_direction = -1; // if m_pair_direction == 1,  left side is green, right side is red
                                // if m_pair_direction == -1, left side is red,   right side is green
        bool m_clear_status = false;
        bool m_first_goal_status = false;
        bool m_white_pair_status = false;
        bool m_pair_status = false;
        bool m_eval_pair_status = false;

        vector <vector<float>> m_red_totem_list;
        vector <vector<float>> m_green_totem_list;
        vector <vector<float>> m_white_totem_list;
        vector <vector<float>> m_goal_list;
        vector <vector<float>> m_all_goal_list;
        // real evaluation param 

        

        
    public:
        bt::Action action;
        bt::Condition condition_find_pair;
        bt::Condition condition_eval_pair;
        Perception();

        void getParam();
        void loadObject();
        

        void clearSignalCallback(const std_msgs::Bool::ConstPtr& msg);
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
        void detection();

        void goalPointEval();
        void pairEval();
        void multiPairEval();
        void goalExtend(float *red_totem, float *green_totem, float *new_goal);
        void publishGoal();
        void publishPair(bool state);

        void boundaryVisualize();
        void objectVisualize();
        void goalPointVisualize();
        void pairVisualize();
        void visualize();

        void actionSet(int state);
        void conditionFindPairSet(bool state);
        void conditionEvalPairSet(bool state);
        void conditionSetLastState();
        void prolongEvalState(bool state);

};

Perception :: Perception() : action(ros::this_node::getName()),
                            condition_eval_pair(appendString(ros::this_node::getName(), (string)"_running")),
                            condition_find_pair("find_pair"){
    pub_goalpoint = n.advertise<geometry_msgs::PoseStamped>("perception_result/goal", 10);
    pub_marker_boundary = n.advertise<visualization_msgs::Marker>("drone_marker/fov_edge", 10);
    pub_marker_object = n.advertise<visualization_msgs::MarkerArray>("drone_marker/object", 10);
    pub_marker_goal = n.advertise<visualization_msgs::MarkerArray>("drone_marker/goal", 10);

    pub_pair_state = n.advertise<std_msgs::Bool>("perception/state", 10);
    sub_clear_signal = n.subscribe<std_msgs::Bool>("perception/clear", 1,  &Perception::clearSignalCallback, this);
    // pub_totem_pose = n.advertise<::>("", 10);
    // sub_totem_gt = n.subscribe<::>("", 1,  &Perception::, this);

    sub_current_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Perception::dronePositionCallback, this);
    sub_wamv_pose = n.subscribe<geometry_msgs::PoseStamped>("/landing_area", 1,  &Perception::wamvPositionCallback, this);
}

void Perception :: clearSignalCallback(const std_msgs::Bool::ConstPtr& msg){
    m_clear_status = msg->data;
    return;
}

void Perception :: dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_current_pose[0] =  msg->pose.position.x + m_camera_offset_x;
    m_current_pose[1] =  msg->pose.position.y + m_camera_offset_y;
    m_current_pose[2] =  msg->pose.position.z + m_camera_offset_z;
    m_current_pose[3] =  msg->pose.orientation.x;
    m_current_pose[4] =  msg->pose.orientation.y;
    m_current_pose[5] =  msg->pose.orientation.z;
    m_current_pose[6] =  msg->pose.orientation.w;
    return;
}

void Perception :: wamvPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_wamv_pose[0] =  msg->pose.position.x;
    m_wamv_pose[1] =  msg->pose.position.y;
    m_wamv_pose[2] =  msg->pose.position.z;
    m_wamv_pose[3] =  msg->pose.orientation.x;
    m_wamv_pose[4] =  msg->pose.orientation.y;
    m_wamv_pose[5] =  msg->pose.orientation.z;
    m_wamv_pose[6] =  msg->pose.orientation.w;
    return;
}

void Perception :: vectorStructClear(){
    m_red_totem_list.clear();
    m_green_totem_list.clear();
    m_white_totem_list.clear();
    // m_goal_list.clear();
    return;
}

void Perception :: vectorStructPush(float *input_array, vector <vector<float>> &append_vector){ 
    float exist_totem[3];
    for(int i = 0; i < append_vector.size(); i++){
        exist_totem[0] = append_vector[i][0];
        exist_totem[1] = append_vector[i][1];
        exist_totem[2] = append_vector[i][2];
        if(distanceP2P(exist_totem, input_array) < m_duplicate_margin_object){ 
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
    theta1 = thetaCalculate(object_pose, m_boundary[0], m_boundary[1]);
    theta2 = thetaCalculate(object_pose, m_boundary[1], m_boundary[2]);
    theta3 = thetaCalculate(object_pose, m_boundary[2], m_boundary[3]);
    theta4 = thetaCalculate(object_pose, m_boundary[3], m_boundary[0]);
    if(abs(theta1 + theta2 + theta3 + theta4 - 360) < 1){
        return true;
    }
    else{
        return false;
    }
}

void Perception :: detection(){

    // detection for simulation (known ground truth)
    boundaryGenerate(m_current_pose[2]);
    for(int i = 0; i < m_red_num_gt; i++){
        if(isInFOV(m_red_totem_list_gt[i])){ 
            m_red_totem_status[i] = true;
            vectorStructPush(m_red_totem_list_gt[i], m_red_totem_list);
        }
        else{ m_red_totem_status[i] = false;}
    }

    for(int i = 0; i < m_green_num_gt; i++){
        if(isInFOV(m_green_totem_list_gt[i])){ 
            m_green_totem_status[i] = true; 
            vectorStructPush(m_green_totem_list_gt[i], m_green_totem_list);
        }
        else{ m_green_totem_status[i] = false; }
    }

    for(int i = 0; i < m_white_num_gt; i++){
        if(isInFOV(m_white_totem_list_gt[i])){ 
            m_white_totem_status[i] = true; 
            vectorStructPush(m_white_totem_list_gt[i], m_white_totem_list);
        }
        else{ m_white_totem_status[i] = false; }
    }
    // detection for simulation (known ground truth)

    // detection based on real perception model
    // TODO
    // detection based on real perception model

    if(m_clear_status){
        m_clear_status = false;
        m_pair_status = false;
        // vectorStructClear();
    }

    if(m_pair_status){
        // publishPair(true);
        conditionFindPairSet(true);
    }else{
        // publishPair(false);
        conditionFindPairSet(false);
    }
    return;
}

void Perception :: loadObject(){
    // virtual
    m_boundary = new float *[4];

    for(int i = 0; i < 4; i++){ m_boundary[i] = new float[3]; }

    m_red_totem_list_gt = new float *[m_red_num_gt];
    m_green_totem_list_gt = new float *[m_green_num_gt];
    m_white_totem_list_gt = new float *[m_white_num_gt];

    m_red_totem_status = new bool [m_red_num_gt];
    m_green_totem_status = new bool [m_green_num_gt];
    m_white_totem_status = new bool [m_white_num_gt];

    for(int i = 0; i < m_red_num_gt; i++){ m_red_totem_list_gt[i] = new float[object_dim]; }
    for(int i = 0; i < m_green_num_gt; i++){ m_green_totem_list_gt[i] = new float[object_dim]; }
    for(int i = 0; i < m_white_num_gt; i++){ m_white_totem_list_gt[i] = new float[object_dim]; }
    
    for(int i = 0; i < m_red_num_gt; i++){
        m_red_totem_status[i] = false;
        for(int j = 0; j < object_dim; j++){
            try{
                std::ostringstream out_str;
                out_str << m_xml_red_totem[i][j];

                std::istringstream in_str(out_str.str());
                in_str >> m_red_totem_list_gt[i][j];
            }
            catch(...){
                throw;
            }
        }
    }
    for(int i = 0; i < m_green_num_gt; i++){
        m_green_totem_status[i] = false;
        for(int j = 0; j < object_dim; j++){
            try{
                std::ostringstream out_str;
                out_str << m_xml_green_totem[i][j];

                std::istringstream in_str(out_str.str());
                in_str >> m_green_totem_list_gt[i][j];
            }
            catch(...){
                throw;
            }
        }
    }
    for(int i = 0; i < m_white_num_gt; i++){
        m_white_totem_status[i] = false;
        for(int j = 0; j < object_dim; j++){
            try{
                std::ostringstream out_str;
                out_str << m_xml_white_totem[i][j];

                std::istringstream in_str(out_str.str());
                in_str >> m_white_totem_list_gt[i][j];
            }
            catch(...){
                throw;
            }
        }
    }
    // virtual
    return;
}

void Perception :: prolongEvalState(bool state){
    ros::Rate rate(10);
    for(int i = 0; i < 15; i++){
        conditionEvalPairSet(state);
        rate.sleep();
    }
    return;
}

void Perception :: boundaryGenerate(float height){
    float angle_inner_xz_plane, angle_outer_xz_plane, angle_right_yz_plane, angle_left_yz_plane;
    double roll, pitch, yaw;
    tf::Quaternion q(
        m_current_pose[3],
        m_current_pose[4],
        m_current_pose[5],
        m_current_pose[6]);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    angle_inner_xz_plane = 90 - m_camera_tilt_angle - m_camera_vertical_FOV / 2 - (float)(pitch * 180 / PI);
    angle_outer_xz_plane = 90 - m_camera_tilt_angle + m_camera_vertical_FOV / 2 - (float)(pitch * 180 / PI);
    angle_right_yz_plane = m_camera_horizontal_FOV / 2 - (float)(roll * 180 / PI);
    angle_left_yz_plane = m_camera_horizontal_FOV / 2 + (float)(roll * 180 / PI);

    // calculate roll + pitch
    if(angle_outer_xz_plane >= 90){ // limit by camera view distance
        // up-left (+, +)
        m_boundary[0][0] = m_current_pose[0] + m_camera_view_distance * cos(angle_outer_xz_plane - 90);
        m_boundary[0][1] = m_current_pose[1] + m_camera_view_distance * tan(angle_left_yz_plane * (PI / 180));
        m_boundary[0][2] = 0;
        // up-right (+, -)
        m_boundary[1][0] = m_current_pose[0] + m_camera_view_distance * cos(angle_outer_xz_plane - 90);
        m_boundary[1][1] = m_current_pose[1] - m_camera_view_distance * tan(angle_right_yz_plane * (PI / 180));
        m_boundary[1][2] = 0;
    }else{
        // up-left (+, +)
        m_boundary[0][0] = m_current_pose[0] + height * tan(angle_outer_xz_plane * (PI / 180));
        m_boundary[0][1] = m_current_pose[1] + height / cos(angle_outer_xz_plane * (PI / 180))
                        * tan(angle_left_yz_plane * (PI / 180));
        m_boundary[0][2] = 0;
        // up-right (+, -)
        m_boundary[1][0] = m_current_pose[0] + height * tan(angle_outer_xz_plane * (PI / 180));
        m_boundary[1][1] = m_current_pose[1] - height / cos(angle_outer_xz_plane * (PI / 180))
                        * tan(angle_right_yz_plane * (PI / 180));
        m_boundary[1][2] = 0;
    }

    if(angle_inner_xz_plane < 0){
        // down-right (+, -)
        m_boundary[2][0] = m_current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
        m_boundary[2][1] = m_current_pose[1] + height / cos(angle_inner_xz_plane * (PI / 180))
                        * tan(angle_right_yz_plane * (PI / 180));
        m_boundary[2][2] = 0;
        // down-left (+, +)
        m_boundary[3][0] = m_current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
        m_boundary[3][1] = m_current_pose[1] - height / cos(angle_inner_xz_plane * (PI / 180))
                        * tan(angle_left_yz_plane * (PI / 180));
        m_boundary[3][2] = 0;
    }else{
        // down-right (+, -)
        m_boundary[2][0] = m_current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
        m_boundary[2][1] = m_current_pose[1] - height / cos(angle_inner_xz_plane * (PI / 180))
                        * tan(angle_right_yz_plane * (PI / 180));
        m_boundary[2][2] = 0;
        // down-left (+, +)
        m_boundary[3][0] = m_current_pose[0] + height * tan(angle_inner_xz_plane * (PI / 180));
        m_boundary[3][1] = m_current_pose[1] + height / cos(angle_inner_xz_plane * (PI / 180))
                        * tan(angle_left_yz_plane * (PI / 180));
        m_boundary[3][2] = 0;
    }
    

    // calculate yaw 

    float **tmp = new float *[4];
    for(int i = 0; i < 4; i++){ 
        tmp[i] = new float[3];
    }
    for(int i = 0; i < 4; i++){ 
        for(int j = 0; j < 3; j++){
            tmp[i][j] = m_boundary[i][j];
        }
    }
    m_boundary[0][0] = m_current_pose[0] + distanceP2P(tmp[0], m_current_pose)
                     * cos(atan((tmp[0][1] - m_current_pose[1]) / (tmp[0][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[0][1] = m_current_pose[1] + distanceP2P(tmp[0], m_current_pose)
                     * sin(atan((tmp[0][1] - m_current_pose[1]) / (tmp[0][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[1][0] = m_current_pose[0] + distanceP2P(tmp[1], m_current_pose)
                     * cos(atan((tmp[1][1] - m_current_pose[1]) / (tmp[1][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[1][1] = m_current_pose[1] + distanceP2P(tmp[1], m_current_pose)
                     * sin(atan((tmp[1][1] - m_current_pose[1]) / (tmp[1][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[2][0] = m_current_pose[0] + distanceP2P(tmp[2], m_current_pose)
                     * cos(atan((tmp[2][1] - m_current_pose[1]) / (tmp[2][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[2][1] = m_current_pose[1] + distanceP2P(tmp[2], m_current_pose)
                     * sin(atan((tmp[2][1] - m_current_pose[1]) / (tmp[2][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[3][0] = m_current_pose[0] + distanceP2P(tmp[3], m_current_pose)
                     * cos(atan((tmp[3][1] - m_current_pose[1]) / (tmp[3][0] - m_current_pose[0]))
                     + yaw);
    m_boundary[3][1] = m_current_pose[1] + distanceP2P(tmp[3], m_current_pose)
                     * sin(atan((tmp[3][1] - m_current_pose[1]) / (tmp[3][0] - m_current_pose[0]))
                     + yaw);
    
    for(int i = 0; i < 4; i++){ delete [] tmp[i]; }
    delete[] tmp;

    return;
}

void Perception :: goalPointEval(){
    
    // m_goal_list.clear();
    int dulicate_cnt = 0;
    int red_totem_num = m_red_totem_list.size();
    int green_totem_num = m_green_totem_list.size();
    int white_totem_num = m_white_totem_list.size();

    cout << "R: " << red_totem_num << " G: " << green_totem_num << " W: " << white_totem_num << endl;
    // m_phase
    if(0){

    }else{

    if(red_totem_num == 0 && green_totem_num == 0){ 
        // cout << "no totem" << endl; 
    }
    else if(red_totem_num == 1 && green_totem_num == 1){ 
        m_pair_status = true;
        pairEval(); 
    }
    else if(red_totem_num == 0 || green_totem_num == 0){ 
        // cout << "no pair" << endl; 
    }
    else if(red_totem_num >= 1 && green_totem_num >= 1){ 
        m_pair_status = true;
        multiPairEval();
    }

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
    // cout << "pairEval" << endl; 
    float red_totem[3], green_totem[3];

    red_totem[0] = m_red_totem_list[0][0];
    red_totem[1] = m_red_totem_list[0][1];
    red_totem[2] = m_red_totem_list[0][2];

    green_totem[0] = m_green_totem_list[0][0];
    green_totem[1] = m_green_totem_list[0][1];
    green_totem[2] = m_green_totem_list[0][2];

    // cout << "distance: " << distanceP2P(red_totem, green_totem) << endl;
    goalExtend(red_totem, green_totem, m_current_goal);
    return;
}

void Perception :: multiPairEval(){
    // cout << "multiPairEval" << endl; 
    float mid_distance = 10000, current_distance = 0, theta = 0;
    float pair_vector[2], heading_vector[2], red_totem[3], green_totem[3], mid_point[3];
    int index[2];


    // version 2
    for(int i = 0; i < m_red_totem_list.size(); i++){
        for(int k = 0; k < 3; k++){ red_totem[k] = m_red_totem_list[i][k]; }

        current_distance = distanceP2P(red_totem, m_current_pose);
        if(mid_distance > current_distance){
            index[0] = i;
            mid_distance = current_distance;
        }
    }
    red_totem[0] = m_red_totem_list[index[0]][0];
    red_totem[1] = m_red_totem_list[index[0]][1];
    red_totem[2] = m_red_totem_list[index[0]][2];

    mid_distance = 10000; 

    for(int i = 0; i < m_green_totem_list.size(); i++){
        for(int k = 0; k < 3; k++){ green_totem[k] = m_green_totem_list[i][k]; }

        current_distance = distanceP2P(green_totem, m_current_pose);
        if(mid_distance > current_distance){
            index[0] = i;
            mid_distance = current_distance;
        }
    }
    green_totem[0] = m_green_totem_list[index[0]][0];
    green_totem[1] = m_green_totem_list[index[0]][1];
    green_totem[2] = m_green_totem_list[index[0]][2];
    // version 2

    
    goalExtend(red_totem, green_totem, m_current_goal);
    return;
}

void Perception :: goalExtend(float *red_totem, float *green_totem, float *new_goal){
    m_first_goal_status = true;
    m_goal_list.clear();
    float v1[2], v2[2];
    vectorP2P(green_totem, red_totem, v1);
    vectorRotate(v1, m_pair_direction * 90, v2);

    vector <float> p;
    for(int i = 0; i < 3; i++){
        p.push_back(red_totem[i] + (green_totem[i] - red_totem[i]) / 2);
    }
    m_goal_list.push_back(p);
    p.clear();

    
    new_goal[0] = red_totem[0] + (green_totem[0] - red_totem[0]) / 2 + v2[0] * m_goal_extend_length / distanceP2P(green_totem, red_totem);
    new_goal[1] = red_totem[1] + (green_totem[1] - red_totem[1]) / 2 + v2[1] * m_goal_extend_length / distanceP2P(green_totem, red_totem);
    new_goal[2] = 0; 

    for(int i = 0; i < 3; i++){
        p.push_back(new_goal[i]);
    }
    m_goal_list.push_back(p);
    
    return;
}

void Perception :: publishPair(bool state){
    std_msgs::Bool pub_msg_pair;
    pub_msg_pair.data = state;
    pub_pair_state.publish(pub_msg_pair);
    return;
}

void Perception :: publishGoal(){
    if(m_goal_list.size() != 2){return;}

    
    // int checkbit = 0, flag = 0;

    // cout << "x: " << m_goal_list[1][0] << endl << "y: " << m_goal_list[1][1] << endl << "z: " << m_goal_list[1][2] << endl;
    // for(int i = 0; i < m_all_goal_list.size(); i++){
    //     for(int j = 0; j < 3; j++){
    //         if(m_all_goal_list[i][j] == m_goal_list[1][j]){checkbit++;}
    //     }
    //     if(checkbit == 3){
    //         cout << "du!" << endl;
    //         checkbit = 0;
    //     }else{
    //         flag++;
    //     }
        
    // }
    
    // if(flag == m_all_goal_list.size()){
        
    //     cout << "push" << endl;
    //     vector <float> p;
    //     for(int i = 0; i < 3; i++){
    //         p.push_back(m_goal_list[1][i]);
    //     }
    //     m_all_goal_list.push_back(p);
    //     p.clear();

    // }
    // cout << "new size: " << m_all_goal_list.size() << endl;

    pub_msg_goal.header.frame_id = "local_origin";
    pub_msg_goal.header.stamp = ros::Time::now();
    pub_msg_goal.pose.position.x = m_goal_list[1][0];
    pub_msg_goal.pose.position.y = m_goal_list[1][1];
    pub_msg_goal.pose.position.z = 0;

    
    float yaw = 0, safety_theta = 0;
    float origin_goal_position[2], extend_x_position[2], extend_goal_position[2];

    origin_goal_position[0] = m_goal_list[0][0];
    origin_goal_position[1] = m_goal_list[0][1];
    extend_x_position[0] = m_goal_list[0][0] + 1;
    extend_x_position[1] = m_goal_list[0][1];

    extend_goal_position[0] = m_goal_list[1][0];
    extend_goal_position[1] = m_goal_list[1][1];
    
    yaw = thetaCalculate(origin_goal_position, extend_x_position, extend_goal_position);
    
    if((extend_goal_position[1] - origin_goal_position[1]) < 0){
        yaw = -yaw;
    }
    safety_theta = yaw;

    if(distanceP2P(m_current_pose, origin_goal_position) < m_duplicate_margin_goal){
        // cout << "Mode: Force pose" << endl;
        tf::Quaternion q;
        q.setRPY(0, 0, yaw * PI / 180);
        q = q.normalize();

        pub_msg_goal.pose.orientation.x = q.getX();
        pub_msg_goal.pose.orientation.y = q.getY();
        pub_msg_goal.pose.orientation.z = q.getZ();
        pub_msg_goal.pose.orientation.w = q.getW();
    }else{
        origin_goal_position[0] = m_current_pose[0];
        origin_goal_position[1] = m_current_pose[1];

        extend_x_position[0] = m_current_pose[0] + 1;
        extend_x_position[1] = m_current_pose[1];

        extend_goal_position[0] = m_goal_list[1][0];
        extend_goal_position[1] = m_goal_list[1][1];

        yaw = thetaCalculate(origin_goal_position, extend_x_position, extend_goal_position);
        
        if((extend_goal_position[1] - origin_goal_position[1]) < 0){
            yaw = -yaw;
        }

        // check whether the pose is guiding toward wrong direction
        if(abs(safety_theta - yaw) >= 90){
            // cout << "Mode: In wrong direction" << endl;
            yaw = safety_theta;
        }else{
            // cout << "Mode: heading..." << endl;
        }
        // check whether the pose is guiding toward wrong direction

        tf::Quaternion q;
        q.setRPY(0, 0, yaw * PI / 180);
        q = q.normalize();

        pub_msg_goal.pose.orientation.x = q.getX();
        pub_msg_goal.pose.orientation.y = q.getY();
        pub_msg_goal.pose.orientation.z = q.getZ();
        pub_msg_goal.pose.orientation.w = q.getW();
    }
    // cout << "x: " << pub_msg_goal.pose.position.x << endl;
    // cout << "y: " << pub_msg_goal.pose.position.y << endl;
    // cout << "z: " << pub_msg_goal.pose.position.z << endl;
    pub_goalpoint.publish(pub_msg_goal);
    return;
}

void Perception :: objectVisualize(){
    visualization_msgs::MarkerArray pub_visual_object;

    for (int i = 0; i < m_red_num_gt; i++){
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
        marker.color.r = m_red_totem_status[i];
        marker.color.g = 0;
        marker.color.b = ~m_red_totem_status[i];
        marker.color.a = 1.0;
        marker.pose.position.x = m_red_totem_list_gt[i][0];
        marker.pose.position.y = m_red_totem_list_gt[i][1];
        marker.pose.position.z = m_red_totem_list_gt[i][2];
        pub_visual_object.markers.push_back(marker);
    }

    for (int i = 0; i < m_green_num_gt; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "object";
        marker.id = i + m_red_num_gt;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        // green  
        marker.color.g = m_green_totem_status[i];
        marker.color.b = ~m_green_totem_status[i];
        marker.color.r = 0;
        marker.color.a = 1.0;
        marker.pose.position.x = m_green_totem_list_gt[i][0];
        marker.pose.position.y = m_green_totem_list_gt[i][1];
        marker.pose.position.z = m_green_totem_list_gt[i][2];
        pub_visual_object.markers.push_back(marker);
    }

    for (int i = 0; i < m_white_num_gt; i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "object";
        marker.id = i + m_red_num_gt + m_green_num_gt;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        // white  
        marker.color.r = m_white_totem_status[i];
        marker.color.g = m_white_totem_status[i];
        marker.color.b = m_white_totem_status[i];
        marker.color.a = 1.0;
        marker.pose.position.x = m_white_totem_list_gt[i][0];
        marker.pose.position.y = m_white_totem_list_gt[i][1];
        marker.pose.position.z = m_white_totem_list_gt[i][2];
        pub_visual_object.markers.push_back(marker);
    }

    pub_marker_object.publish(pub_visual_object);

    return;
}

void Perception :: boundaryVisualize(){
    visualization_msgs::Marker pub_visual_boundary;
    pub_visual_boundary.header.frame_id = "local_origin";
    pub_visual_boundary.header.stamp = ros::Time::now();
    pub_visual_boundary.ns = "fov";
    pub_visual_boundary.action = visualization_msgs::Marker::ADD;
    pub_visual_boundary.pose.orientation.w = 1.0;
    pub_visual_boundary.type = visualization_msgs::Marker::LINE_STRIP;
    pub_visual_boundary.scale.x = 0.1;
    pub_visual_boundary.scale.y = 0.1;
    pub_visual_boundary.scale.z = 0.1;
    pub_visual_boundary.color.b = 1.0;
    pub_visual_boundary.color.a = 1.0;

    for (int i = 0; i < 4; i++){
        geometry_msgs::Point p;
        p.x = m_boundary[i][0];
        p.y = m_boundary[i][1];
        p.z = m_boundary[i][2];
        pub_visual_boundary.points.push_back(p);
    }

    geometry_msgs::Point p;
    p.x = m_boundary[0][0];
    p.y = m_boundary[0][1];
    p.z = m_boundary[0][2];
    pub_visual_boundary.points.push_back(p);

    pub_marker_boundary.publish(pub_visual_boundary);
    return;
}

void Perception :: goalPointVisualize(){
    // if(m_goal_list.size() != 2){return;}
    visualization_msgs::MarkerArray pub_visual_goal;

    for(int i = 0; i < m_all_goal_list.size(); i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "local_origin";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.r = 1;
        marker.color.b = 1;
        marker.color.a = 0.5;
        marker.pose.position.x = m_goal_list[i][0];
        marker.pose.position.y = m_goal_list[i][1];
        marker.pose.position.z = m_goal_list[i][2];
        pub_visual_goal.markers.push_back(marker);
    }
    pub_marker_goal.publish(pub_visual_goal);
    return;
}

void Perception :: pairVisualize(){
    return;
}

void Perception :: visualize(){
    ros::Rate rate(10);

    objectVisualize();
    boundaryVisualize();
    // goalPointVisualize();
    pairVisualize();

    rate.sleep();
    return;
}

void Perception :: getParam(){
    n.getParam("/pair_eval/camera_tilt_angle", m_camera_tilt_angle);
    n.getParam("/pair_eval/camera_vertical_FOV", m_camera_vertical_FOV);
    n.getParam("/pair_eval/camera_horizontal_FOV", m_camera_horizontal_FOV);
    n.getParam("/pair_eval/red_totem", m_xml_red_totem);
    n.getParam("/pair_eval/green_totem", m_xml_green_totem);
    n.getParam("/pair_eval/white_totem", m_xml_white_totem);
    n.getParam("/pair_eval/camera_offset_x", m_camera_offset_x);
    n.getParam("/pair_eval/camera_offset_y", m_camera_offset_y);
    n.getParam("/pair_eval/camera_offset_z", m_camera_offset_z);
    n.getParam("/pair_eval/camera_view_distance", m_camera_view_distance);
    n.getParam("/pair_eval/duplicate_margin_object", m_duplicate_margin_object);
    n.getParam("/pair_eval/duplicate_margin_goal", m_duplicate_margin_goal);
    n.getParam("/pair_eval/goal_extend_length", m_goal_extend_length);
    n.getParam("/pair_eval/evaluate_pair_threshold", m_evaluate_pair_threshold);
    m_red_num_gt = m_xml_red_totem.size();
    m_green_num_gt = m_xml_green_totem.size();
    m_white_num_gt = m_xml_white_totem.size();

    object_dim = m_xml_red_totem[0].size();
    return;
}

void Perception :: actionSet(int state){
    switch(state){
        case 1:
            action.set_success();
            break;
        case 0:
            action.set_running();
            break;
        case -1:
            action.set_failure();
            break;
    }
    action.publish();
    return;
}

void Perception :: conditionFindPairSet(bool state){
    condition_find_pair.set(state);
    condition_find_pair.publish();
    return;
}

void Perception :: conditionEvalPairSet(bool state){
    condition_eval_pair.set(state);
    condition_eval_pair.publish();
    return;
}

void Perception :: conditionSetLastState(){

    condition_eval_pair.set(false);
    condition_find_pair.set(m_pair_status);
    
    condition_eval_pair.publish();
    condition_find_pair.publish();
    return;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "simulate_perception");
    Perception uav;
    uav.getParam();
    uav.loadObject();

    while(ros::ok()){
        if(uav.action.is_active() && uav.action.active_has_changed()){
            ROS_INFO("Action: Start");
            uav.conditionEvalPairSet(true);
        }
        else if(uav.action.active_has_changed() && !(uav.action.is_active())){
            ROS_INFO("Action: Done");
            uav.actionSet(1);
        }
        else if(uav.action.is_active()){
            uav.actionSet(0);
            uav.goalPointEval();
            // uav.prolongEvalState(true);
            uav.conditionEvalPairSet(false);
        }else{
            uav.conditionSetLastState();
        }
        uav.detection();
        // uav.goalPointEval();
        uav.visualize();
        
        ros::spinOnce();
    }
    return 0;
}

