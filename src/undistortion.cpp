#include<ros/ros.h>
#include<opencv2/opencv.hpp>

// 校正影像去畸变
void undistortImgs(std::vector<std::string> img_paths, cv::Mat cameraMatrix, cv::Mat distCoeffs) {
    std::vector<cv::Mat> inputImgs;
    for (int i = 0; i < img_paths.size(); ++i) {
        cv::Mat img = cv::imread(img_paths[i]);
        cv::Mat dst;
        cv::undistort(img, dst, cameraMatrix, distCoeffs);
        cv::Mat compare(img.size().width, img.size().height, CV_8UC3);
        cv::hconcat(img, dst, compare);
        cv::imwrite(img_paths[i] + "_undistort.jpg", compare);
        std::cout << "Undistorting.." << i + 1 << "/" << img_paths.size() << std::endl;
    }
}

// 针对本场景，将vector转换成Mat，从而让OpenCV可读取
cv::Mat cvtVec2Mat(std::vector<float> vec, int mat_height, int mat_width) {
    cv::Mat restore_mat(mat_height, mat_width,CV_32F);
    for (int i = 0; i < mat_height; i++) {
        for (int j = 0; j < mat_width; j++) {
            int index = i * mat_width + j;
            float val = vec[index];
            restore_mat.at<float>(i, j) = val;
        }
    }
    return restore_mat;
}


int main(int argc, char *argv[])
{
    // 指定图片路径
    std::string dir_path = "/root/imgs/";
    // 构造图片路径，这里只加载9张图片
    std::vector<std::string> paths;
    for (int i = 1; i < 10; ++i) {
        char tmp_path[100];
        sprintf(tmp_path, (dir_path + "img%02d.jpg").c_str(), i);
        paths.push_back(tmp_path);
    }
    
    // 加载参数也有多种方式
    // 1.通过OpenCV的FileStorage加载
    cv::FileStorage fsr = cv::FileStorage(dir_path + "calib_res.yaml", cv::FileStorage::READ);
    cv::Mat cv_camera_mat = fsr["cam_mat"].mat();
    cv::Mat cv_dist = fsr["dist_mat"].mat();

    // 2.通过ROS的load加载
    // 初始化节点并指定节点名称
    ros::init(argc, argv, "undistortion");
    // 先将所有参数载入
    system(("rosparam load "+ dir_path + "all_params.yaml").c_str());
    
    // 找到对应数据，注意get函数的用法，数据通过函数的引用返回
    std::vector<float> cam_vec;
    ros::param::get("cam_mat",cam_vec);
    std::vector<float> dist_vec;
    ros::param::get("dist_mat",dist_vec);
    
    // 将数据转换成Mat类型
    cv::Mat camera_mat, dist;
    camera_mat = cvtVec2Mat(cam_vec,3,3);
    dist = cvtVec2Mat(dist_vec,1,5);
    std::cout<<"Parameter load finished."<<std::endl;
    
    // 校正图片
    undistortImgs(paths,camera_mat,dist);
    std::cout<<"Undistortion finished."<<std::endl;
    
    return 0;
}