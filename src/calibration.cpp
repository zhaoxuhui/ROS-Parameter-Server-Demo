#include<ros/ros.h>
#include<opencv2/opencv.hpp>

// 实现参考https://blog.csdn.net/u011574296/article/details/73823569，写得很详细
void calibrateCamera(std::vector<std::string> img_paths, cv::Size boardSize, cv::Size gridSize,
                     cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {
    // 迭代算法的终止准则
    // 第一个参数是类型，CV_TERMCRIT_ITER和CV_TERMCRIT_EPS之一或二者的组合
    // 第二个参数是最大迭代次数
    // 第三个参数是结果的精确性
    cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 30, 0.001);

    // 每张图片里角点的像素坐标与真实坐标
    std::vector<cv::Point3f> pts;
    std::vector<cv::Point2f> corners;

    // 由每张图片的像素坐标和角点坐标构成的vector
    std::vector<std::vector<cv::Point3f> > objpoints;
    std::vector<std::vector<cv::Point2f> > imgpoints;
    // 构造真实坐标，z分量为0
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            cv::Point3f pt;
            pt.x = i * gridSize.width;
            pt.y = j * gridSize.height;
            pt.z = 0;
            pts.push_back(pt);
        }
    }

    // 依次加载图片，并识别棋盘格
    for (int k = 0; k < img_paths.size(); ++k) {
        cv::Mat tmp_img = cv::imread(img_paths[k]);
        cv::Mat gray;
        cv::cvtColor(tmp_img, gray, cv::COLOR_BGR2GRAY);
        bool ret = cv::findChessboardCorners(gray, boardSize, corners);
        if (ret) {
            objpoints.push_back(pts);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            imgpoints.push_back(corners);
            cv::drawChessboardCorners(tmp_img, boardSize, corners, ret);
            std::cout << "Load " << k + 1 << "/" << img_paths.size() << std::endl;
            cv::imshow("board", tmp_img);
            cv::waitKey(500);
        }
    }

    cv::Size img_size;
    img_size.height = cv::imread(img_paths[0]).size().height;
    img_size.width = cv::imread(img_paths[0]).size().width;
    std::vector<cv::Mat> rvecsMat, tvecsMat;    // 用于接收不同图片相对于第一张图片的R、t
    // 最后一个参数表示畸变系数只求到k3，完整的畸变参数输出顺序如下
    // (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])
    cv::calibrateCamera(objpoints,
                        imgpoints,
                        img_size,
                        cameraMatrix,
                        distCoeffs,
                        rvecsMat,
                        tvecsMat,
                        CV_CALIB_FIX_K3);
}

// 针对本场景，按行优先存储，将Mat转换成vector形式，从而让ROS可以读取
std::vector<float> cvtMat2Vec(cv::Mat mat){
    std::vector<float> vec;
    for(int i=0;i<mat.size().height;i++){
        for(int j=0;j<mat.size().width;j++){
            vec.push_back(mat.at<double>(i,j));
        }
    }
    return vec;
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
    cv::Size boardSize = cv::Size(8, 6);    // 棋盘格大小
    cv::Size gridSize = cv::Size(30, 30);   // 棋盘格每一格的长度，单位mm
    cv::Mat camera_mat, dist;               // 用于接收相机内参与畸变参数
    // 校正相机
    calibrateCamera(paths, boardSize, gridSize, camera_mat, dist);
    std::cout<<"Calibration finished."<<std::endl;

    // 将Mat类型的数据转换为vector类型
    std::vector<float> cam_mat;
    cam_mat = cvtMat2Vec(camera_mat);
    std::vector<float> dist_mat;
    dist_mat = cvtMat2Vec(dist);
    std::cout<<"Data conversion finished."<<std::endl;

    // 初始化节点并指定名称
    ros::init(argc, argv, "calibration");
    // 设置参数
    ros::param::set("cam_mat",cam_mat);
    ros::param::set("dist_mat",dist_mat);
    std::cout<<"Parameter set finished."<<std::endl;
    
    // 保存参数也有多种方法
    // 1.调用OpenCV的FileStorage库保存参数
    cv::FileStorage fsw = cv::FileStorage(dir_path + "calib_res.yaml", cv::FileStorage::WRITE);
    fsw.write("cam_mat", camera_mat);
    fsw.write("dist_mat", dist);
    fsw.release();

    // 2.调用ROS的dump命令将所有参数保存到指定文件中
    system(("rosparam dump "+ dir_path + "all_params.yaml").c_str());
    std::cout<<"Parameter save finished."<<std::endl;
    return 0;
}