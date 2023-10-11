#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 


class State{
    private:
        ros::NodeHandle n;
        ros::Subscriber sub_pair_state;
        ros::Subscriber sub_reset_signal;
        ros::Subscriber sub_explore_1;
        ros::Subscriber sub_explore_2;
        ros::Publisher pub_clear_pair;

        bool explore_1_state = false;
        bool explore_2_state = false;
        bool pair_state = false;
        bool pair_finish = false;
    public:
        bt::Condition condition;
        State();
        void conditionSet(bool state);
        void stateCallback(const std_msgs::Bool::ConstPtr& msg);
        void resetCallback(const std_msgs::Bool::ConstPtr& msg);
        void explore1Callback(const std_msgs::Bool::ConstPtr& msg);
        void explore2Callback(const std_msgs::Bool::ConstPtr& msg);
        void stateEval();
        void publishClear();
};

State :: State() : condition(ros::this_node::getName()){
    sub_pair_state = n.subscribe<std_msgs::Bool>("perception/state", 1,  &State::stateCallback, this);
    sub_reset_signal = n.subscribe<std_msgs::Bool>("state_manager/reset", 1,  &State::resetCallback, this);
    sub_explore_1 = n.subscribe<std_msgs::Bool>("explore_1/finished", 1,  &State::explore1Callback, this);
    sub_explore_2 = n.subscribe<std_msgs::Bool>("explore_2/finished", 1,  &State::explore2Callback, this);

    pub_clear_pair = n.advertise<std_msgs::Bool>("perception/clear", 10);
}

void State :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void State :: stateCallback(const std_msgs::Bool::ConstPtr& msg){
    pair_state = msg->data;
    return;
}

void State :: resetCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data){ 
        // cout << "rest recieved!!" << endl;
        pair_finish = false; 
        publishClear();
    }
    return;
}

void State :: explore1Callback(const std_msgs::Bool::ConstPtr& msg){
    explore_1_state = msg->data;
    return;
}

void State :: explore2Callback(const std_msgs::Bool::ConstPtr& msg){
    explore_2_state = msg->data;
    return;
}

void State :: publishClear(){
    std_msgs::Bool pub_msg_clear;
    pub_msg_clear.data = true;
    pub_clear_pair.publish(pub_msg_clear);
    return;
}

void State :: stateEval(){
    ros::Rate rate(15);
    bool debug = false;
    conditionSet(pair_finish);

    if(pair_state){
        pair_finish = true;
        debug = true;
    }
    
    // cout << "D:" << debug; 
    // cout << " P: " << pair_finish << endl;
    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "state_pair");
    State pair;
    while(ros::ok()){
        pair.stateEval();
        ros::spinOnce();
    }
    return 0;
}