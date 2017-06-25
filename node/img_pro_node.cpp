#include "img_pro.h"
#include "fast/fast.h"
#include "fast/fast_cv.h"

void callback__(cv_bridge::CvImagePtr& cv_img_ptr_in_, cv_bridge::CvImage& cv_img_out_)
{
      cv::Rect_<int> box;

      box.x = (cv_img_ptr_in_->image.cols/2)-10;
      box.y = (cv_img_ptr_in_->image.rows/2)-10;
      box.width = 20;
      box.height = 20;

      cv::rectangle(cv_img_out_.image, box, cv::Scalar(0,255,255), 3);
      cv::ellipse(cv_img_out_.image,cv::Point(50,50),cv::Size(20,10),27,0,360,cv::Scalar(0,0,255),2);
}

void callback(cv_bridge::CvImagePtr& cv_img_ptr_in_, cv_bridge::CvImage& cv_img_out_)
{
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat img;
      cv::cvtColor(cv_img_out_.image, img, CV_BGR2GRAY);
      fastcv::keypoint_detect_10_sse2(img, 75, keypoints);
      cv::drawKeypoints(cv_img_ptr_in_->image, keypoints, cv_img_out_.image, cv::Scalar(255, 0, 0));
}

int main(int argc, char **argv)
{
      ros::init(argc, argv, "img_pro");
      
      ImgPro imgpro(callback, "image_in", "image_out");
      
      imgpro.spin(60);
            
      return 0;
}
