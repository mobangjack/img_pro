#ifndef __IMG_PRO_H__
#define __IMG_PRO_H__

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

typedef std::function<void(cv_bridge::CvImagePtr&, cv_bridge::CvImage&)> ImgProCallback;

class ImgPro
{
    protected:
        std::string image_in_, image_out_;

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport img_tp_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;
        
    protected:
        void process();
        void publish();

    protected:
        ImgProCallback callback;

    protected:
        void imageCallback(const sensor_msgs::ImageConstPtr& _msg);

    public:
        ImgPro(ImgProCallback callback_ = nullptr, std::string image_in = "image_in", std::string image_out = "image_out");
        ~ImgPro();
        void spin(double hz = 30);
                    
};

#endif
