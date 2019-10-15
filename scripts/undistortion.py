# coding=utf-8
import rospy
import cv2
import numpy as np
import os


# 校正影像去畸变
def undistortImgs(img_paths, cameraMatrix, distCoeffs):
    for i in range(img_paths.__len__()):
        img = cv2.imread(img_paths[i])
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            cameraMatrix, distCoeffs, (w, h), 1, (w, h))
        # undistort
        dst = cv2.undistort(img, cameraMatrix, dist, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]

        cv2.imwrite(img_paths[i]+"_undistort.jpg", dst)
        print "Undistorting...", (i + 1).__str__() + \
            "/" + img_paths.__len__().__str__()

# 针对本场景，将list转换成NDArray，从而让OpenCV可读取


def cvtList2NDArray(vec, mat_height, mat_width):
    restore_mat = np.ndarray([mat_height, mat_width], float)
    for i in range(mat_height):
        for j in range(mat_width):
            index = i * mat_width+j
            val = vec[index]
            restore_mat[i, j] = val
    return restore_mat


if __name__ == "__main__":
    # 指定图片路径
    dir_path = "/root/imgs/"
    # 构造图片路径，这里只加载9张图片
    paths = []
    for i in range(1, 10):
        tmp_path = dir_path+"img"+i.__str__().zfill(2)+".jpg"
        paths.append(tmp_path)

    # 加载参数也有多种方式
    # 1.通过OpenCV的FileStorage加载
    fs = cv2.FileStorage("config.yml", cv2.FILE_STORAGE_READ)
    cv_camera_mat = fs.getNode("cam_mat")
    cv_dist_mat = fs.getNode("dist_mat")

    # 2.通过ROS的load加载
    # 初始化节点并指定节点名称
    rospy.init_node("undistortion")
    # 先将所有参数载入
    os.system("rosparam load " + dir_path + "all_params.yaml")

    # 找到对应数据
    cam_vec = rospy.get_param("cam_mat")
    dist_vec = rospy.get_param("dist_mat")

    # 将数据转换成NDArray类型
    camera_mat = cvtList2NDArray(cam_vec, 3, 3)
    dist = cvtList2NDArray(dist_vec, 1, 5)

    # 校正图片
    undistortImgs(paths, camera_mat, dist)
    print "Undistortion finished."
