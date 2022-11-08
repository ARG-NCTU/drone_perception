#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <behavior_tree/behavior_tree.h>

using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}

class Switcher{
    private:
        

        int m_current_camera_index = 0; // 1 = forward, 2 = downward, 3 = backward
    public:
        bt::Action switch_forward;
        bt::Action switch_downward;
        bt::Action switch_backward;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub_camera_forward_img;
        image_transport::Subscriber sub_camera_downward_img;
        image_transport::Subscriber sub_camera_backward_img;
        image_transport::Publisher pub_img;

        

        sensor_msgs::Image msg_camera_forward;
        sensor_msgs::Image msg_camera_downward;
        sensor_msgs::Image msg_camera_backward;
        
        Switcher();
        void imageForwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageDownwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageBackwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void actionSet(int action_index, int state);
        void switchImageSource(int camera_index);
        void publishImage(int camera_index);
        void visualize();
};

Switcher :: Switcher() : switch_forward(appendString(ros::this_node::getName(), (string)"_forward")),
                        switch_downward(appendString(ros::this_node::getName(), (string)"_downward")),
                        switch_backward(appendString(ros::this_node::getName(), (string)"_backward")),
                        it(nh){
    sub_camera_forward_img = it.subscribe("forward/image", 1,  &Switcher::imageForwardCallback, this);
    sub_camera_downward_img = it.subscribe("downward/image", 1,  &Switcher::imageDownwardCallback, this);
    sub_camera_backward_img = it.subscribe("backward/image", 1,  &Switcher::imageBackwardCallback, this);
    pub_img = it.advertise("desired_image", 10);
}

void Switcher :: imageForwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_forward = *msg;
    publishImage(1);
    return;
}

void Switcher :: imageDownwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_downward = *msg;
    publishImage(2);
    return;
}

void Switcher :: imageBackwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_backward = *msg;
    publishImage(3);
    return;
}

void Switcher :: publishImage(int camera_index){
    // if(camera_index != m_current_camera_index){ return; }
    // else{
        switch(m_current_camera_index){
            case 1:
                pub_img.publish(msg_camera_forward);
                ROS_INFO("Forward");
                break;
            case 2:
                pub_img.publish(msg_camera_downward);
                ROS_INFO("Downward");
                break;
            case 3:
                pub_img.publish(msg_camera_backward);
                ROS_INFO("Backward");
                break;
        }
    // }
    return;
}

void Switcher :: actionSet(int action_index, int state){
    switch(action_index){
        case 1:
            switch(state){
                case 1:
                    switch_forward.set_success();
                    break;
                case 0:
                    switch_forward.set_running();
                    break;
                case -1:
                    switch_forward.set_failure();
                    break;
            }
            switch_forward.publish();
            break;
        case 2:
            switch(state){
                case 1:
                    switch_downward.set_success();
                    break;
                case 0:
                    switch_downward.set_running();
                    break;
                case -1:
                    switch_downward.set_failure();
                    break;
            }
            switch_downward.publish();
            break;
        case 3:
            switch(state){
                case 1:
                    switch_backward.set_success();
                    break;
                case 0:
                    switch_backward.set_running();
                    break;
                case -1:
                    switch_backward.set_failure();
                    break;
            }
            switch_backward.publish();
            break;
    }
    return;
}

void Switcher :: switchImageSource(int camera_index){
    m_current_camera_index = camera_index;
    return;
}

void Switcher :: visualize(){
    ros::Rate rate(30);
    // ROS_INFO("Now: %d", m_current_camera_index);
    // publishImage(3);
    rate.sleep();
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "image_switcher");
    Switcher image;

    ROS_INFO("HELLO");
    while(ros::ok()){
        if(image.switch_forward.is_active() && image.switch_forward.active_has_changed()){
            ROS_INFO("Forward: Start");
        }
        else if(image.switch_forward.active_has_changed() && !(image.switch_forward.is_active())){
            // ROS_INFO("Forward: End");
            image.actionSet(1, 1);
        }
        else if(image.switch_forward.is_active()){
            // ROS_INFO("Forward: Run");
            image.actionSet(1, 0);
            image.switchImageSource(1);
        }else if(image.switch_downward.is_active() && image.switch_downward.active_has_changed()){
            ROS_INFO("Downward: Start");
        }
        else if(image.switch_downward.active_has_changed() && !(image.switch_downward.is_active())){
            // ROS_INFO("Downward: End");
            image.actionSet(2, 1);
        }
        else if(image.switch_downward.is_active()){
            // ROS_INFO("Downward: Run");
            image.actionSet(2, 0);
            image.switchImageSource(2);
        }else if(image.switch_backward.is_active() && image.switch_backward.active_has_changed()){
            ROS_INFO("Backward: Start");
        }
        else if(image.switch_backward.active_has_changed() && !(image.switch_backward.is_active())){
            // ROS_INFO("Backward: End");
            image.actionSet(3, 1);
        }
        else if(image.switch_backward.is_active()){
            // ROS_INFO("Backward: Run");
            image.actionSet(3, 0);
            image.switchImageSource(3);
        }
        image.visualize();
        ros::spinOnce();
    }
    return 0;
}
