if (cam_info_taken == true){
  sensor_msgs::CameraInfo info = cam_info;
  float centre_x = cam_info.K[2];
  float centre_y = cam_info.K[5];
  float focal_x = cam_info.K[0];
  float focal_y = cam_info.K[4];

  cv::Mat cv_image = cv::Mat(cam_info.height, cam_info.width, CV_32FC1,cv::Scalar(std::numeric_limits<float>::max()));

  for (int i=0; i<msg->points.size();i++){
    if (msg->points[i].z == msg->points[i].z){
        float z = msg->points[i].z*1000.0;
        float u = (msg->points[i].x*1000.0*focal_x) / z;
        float v = (msg->points[i].y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

    if (pixel_pos_x > (cam_info.width-1)){
      pixel_pos_x = cam_info.width -1;
    }
    if (pixel_pos_y > (cam_info.height-1)){
      pixel_pos_y = cam_info.height-1;
    }
    cv_image.at<float>(pixel_pos_y,pixel_pos_x) = z;
    }       
  }

  cv_image.convertTo(cv_image,CV_16UC1);

  sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
  output_image->header = info.header;
  output_image->header.stamp = info.header.stamp = t;
  publish_result.publish(output_image);
  publish_cam_info.publish(info);
}