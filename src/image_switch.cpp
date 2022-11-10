#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <behavior_tree/behavior_tree.h>

using namespace std; 

string appendString(const string &s_body, const string &s_suffix){
    std::string origin = s_body;
    std::string later = s_suffix;
    origin.append(later);
    return origin;
}

// d435_backward_color_optical_frame
// zedm_forward_left_camera_optical_frame
// d435_downward_color_optical_frame
class Switcher{
    private:
        // 1 = forward, 2 = downward, 3 = backward
        int m_current_camera_index = 0; 
    public:
        bt::Action switch_forward;
        bt::Action switch_downward;
        bt::Action switch_backward;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber sub_camera_forward_img_rgb;
        image_transport::Subscriber sub_camera_downward_img_rgb;
        image_transport::Subscriber sub_camera_backward_img_rgb;

        image_transport::Subscriber sub_camera_forward_img_depth;
        image_transport::Subscriber sub_camera_downward_img_depth;
        image_transport::Subscriber sub_camera_backward_img_depth;

        image_transport::Publisher pub_img_rgb;
        image_transport::Publisher pub_img_depth;

        ros::Subscriber sub_camera_forward_info;
        ros::Subscriber sub_camera_downward_info;
        ros::Subscriber sub_camera_backward_info;
        ros::Publisher pub_camera_info;
        

        sensor_msgs::Image msg_camera_forward_rgb;
        sensor_msgs::Image msg_camera_downward_rgb;
        sensor_msgs::Image msg_camera_backward_rgb;

        sensor_msgs::Image msg_camera_forward_depth;
        sensor_msgs::Image msg_camera_downward_depth;
        sensor_msgs::Image msg_camera_backward_depth;

        sensor_msgs::CameraInfo msg_camera_forward_info;
        sensor_msgs::CameraInfo msg_camera_downward_info;
        sensor_msgs::CameraInfo msg_camera_backward_info;
        
        
        Switcher();
        void imageRGBForwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageRGBDownwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageRGBBackwardCallback(const sensor_msgs::Image::ConstPtr& msg);

        void imageDepthForwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageDepthDownwardCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageDepthBackwardCallback(const sensor_msgs::Image::ConstPtr& msg);

        void camInfoForwardCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void camInfoDownwardCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void camInfoBackwardCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

        void actionSet(int action_index, int state);
        void switchImageSource(int camera_index);
        void publishRGB();
        void publishDepth();
        void publishInfo();
        void visualize();
};

Switcher :: Switcher() : switch_forward(appendString(ros::this_node::getName(), (string)"_forward")),
                        switch_downward(appendString(ros::this_node::getName(), (string)"_downward")),
                        switch_backward(appendString(ros::this_node::getName(), (string)"_backward")),
                        it(nh){
    sub_camera_forward_img_rgb = it.subscribe("forward/image/rgb", 1,  &Switcher::imageRGBForwardCallback, this);
    sub_camera_downward_img_rgb = it.subscribe("downward/image/rgb", 1,  &Switcher::imageRGBDownwardCallback, this);
    sub_camera_backward_img_rgb = it.subscribe("backward/image/rgb", 1,  &Switcher::imageRGBBackwardCallback, this);
    pub_img_rgb = it.advertise("desired_image/rgb", 10);

    sub_camera_forward_img_depth = it.subscribe("forward/image/depth", 1,  &Switcher::imageDepthForwardCallback, this);
    sub_camera_downward_img_depth = it.subscribe("downward/image/depth", 1,  &Switcher::imageDepthDownwardCallback, this);
    sub_camera_backward_img_depth = it.subscribe("backward/image/depth", 1,  &Switcher::imageDepthBackwardCallback, this);
    pub_img_depth = it.advertise("desired_image/depth", 10);

    sub_camera_forward_info = nh.subscribe<sensor_msgs::CameraInfo>("forward/image/info", 1,  &Switcher::camInfoForwardCallback, this);
    sub_camera_downward_info = nh.subscribe<sensor_msgs::CameraInfo>("downward/image/info", 1,  &Switcher::camInfoDownwardCallback, this);
    sub_camera_backward_info = nh.subscribe<sensor_msgs::CameraInfo>("backward/image/info", 1,  &Switcher::camInfoBackwardCallback, this);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("desired_image/info", 10);
    
}
// rgb
void Switcher :: imageRGBForwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_forward_rgb = *msg;
    publishRGB();
    return;
}

void Switcher :: imageRGBDownwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_downward_rgb = *msg;
    publishRGB();
    return;
}

void Switcher :: imageRGBBackwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_backward_rgb = *msg;
    publishRGB();
    return;
}
// depth
void Switcher :: imageDepthForwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_forward_depth = *msg;
    publishDepth();
    return;
}

void Switcher :: imageDepthDownwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_downward_depth = *msg;
    publishDepth();
    return;
}

void Switcher :: imageDepthBackwardCallback(const sensor_msgs::Image::ConstPtr& msg){
    msg_camera_backward_depth = *msg;
    publishDepth();
    return;
}
// cam_info
void Switcher :: camInfoForwardCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    msg_camera_forward_info = *msg;
    publishInfo();
    return;
}

void Switcher :: camInfoDownwardCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    msg_camera_downward_info = *msg;
    publishInfo();
    return;
}

void Switcher :: camInfoBackwardCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    msg_camera_backward_info = *msg;
    publishInfo();
    return;
}

void Switcher :: publishRGB(){
    switch(m_current_camera_index){
        case 1:
            pub_img_rgb.publish(msg_camera_forward_rgb);
            // ROS_INFO("Forward");
            break;
        case 2:
            pub_img_rgb.publish(msg_camera_downward_rgb);
            // ROS_INFO("Downward");
            break;
        case 3:
            pub_img_rgb.publish(msg_camera_backward_rgb);
            // ROS_INFO("Backward");
            break;
    }
    return;
}

void Switcher :: publishDepth(){
    switch(m_current_camera_index){
        case 1:
            pub_img_depth.publish(msg_camera_forward_depth);
            // ROS_INFO("Forward");
            break;
        case 2:
            pub_img_depth.publish(msg_camera_downward_depth);
            // ROS_INFO("Downward");
            break;
        case 3:
            pub_img_depth.publish(msg_camera_backward_depth);
            // ROS_INFO("Backward");
            break;
    }
    return;
}

void Switcher :: publishInfo(){
    switch(m_current_camera_index){
        case 1:
            pub_camera_info.publish(msg_camera_forward_info);
            // ROS_INFO("Forward");
            break;
        case 2:
            pub_camera_info.publish(msg_camera_downward_info);
            // ROS_INFO("Downward");
            break;
        case 3:
            pub_camera_info.publish(msg_camera_backward_info);
            // ROS_INFO("Backward");
            break;
    }
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
    // publishRGB(3);
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
