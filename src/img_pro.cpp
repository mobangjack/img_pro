#include "img_pro.h"

ImgPro::ImgPro(ImgProCallback callback_, std::string image_in, std::string image_out) : 
    nh_(ros::this_node::getName()),
    img_tp_(nh_),
    callback(callback_),
    image_in_(image_in),
    image_out_(image_out)
{
    nh_.param("image_in", image_in_, image_in);
    nh_.param("image_out", image_out_, image_out);
    image_sub_ = img_tp_.subscribe(image_in_, 1, &ImgPro::imageCallback, this);
    image_pub_ = img_tp_.advertise(image_out_, 100);
}

ImgPro::~ImgPro()
{
    //
}

void ImgPro::spin(double hz)
{
      ros::Rate rate(hz);

      while ( ros::ok() )
      {
            ros::spinOnce();

            process();
                        
            rate.sleep();            
      }
}

void ImgPro::process()
{
    if (cv_img_ptr_in_ != nullptr && callback)
    {
        cv_img_out_.image = cv_img_ptr_in_->image;

        callback(cv_img_ptr_in_, cv_img_out_);

        publish();

        cv_img_ptr_in_ = nullptr;
    }
}

void ImgPro::publish()
{
    cv_img_out_.header.seq++;
    cv_img_out_.header.stamp = ros::Time::now();
    image_pub_.publish(cv_img_out_.toImageMsg());
}

void ImgPro::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);
        cv_img_out_.header.frame_id = _msg->header.frame_id;
        cv_img_out_.encoding = _msg->encoding;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("ImgPro::imageCallback(): cv_bridge exception: %s", e.what());
        return;
    }      
}


