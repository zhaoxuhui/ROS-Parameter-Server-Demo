# coding=utf-8
import rospy
import cv2
import numpy as np
import os


def calibrateCamera(img_paths, row, col):
    ROWS = row
    COLOMONS = col

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((ROWS * COLOMONS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:ROWS, 0:COLOMONS].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image planeself

    for fname in img_paths:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (ROWS, COLOMONS), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(
                img, (ROWS, COLOMONS), corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    return mtx, dist


# 针对本场景，按行优先存储，将NDArray转换成list形式，从而让ROS可以读取
def cvtNDArray2List(mat):
    res_list = []
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            res_list.append(float(mat[i, j]))
    return res_list


if __name__ == "__main__":
    # 指定图片路径
    dir_path = "/root/imgs/"
    # 构造图片路径，这里只加载9张图片
    paths = []
    for i in range(1, 10):
        tmp_path = dir_path+"img"+i.__str__().zfill(2)+".jpg"
        paths.append(tmp_path)
    # 校正相机
    camera_mat, dist = calibrateCamera(paths, 8, 6)
    print "Calibration finished."

    # 将NDArray类型的数据转换为list类型
    cam_mat = cvtNDArray2List(camera_mat)
    dist_mat = cvtNDArray2List(dist)

    # 初始化节点并指定名称
    rospy.init_node("calibration")
    # 设置参数
    rospy.set_param("cam_mat", cam_mat)
    rospy.set_param("dist_mat", dist_mat)
    print "Parameter set finished."

    # 保存参数也有多种方法
    # 1.调用OpenCV的FileStorage库保存参数
    fs = cv2.FileStorage(dir_path + "calib_res.yaml", cv2.FILE_STORAGE_WRITE)
    fs.write("cam_mat", camera_mat)
    fs.write("dist_mat", dist)
    fs.release()

    # 2.调用ROS的dump命令将所有参数保存到指定文件中
    os.system("rosparam dump " + dir_path + "all_params.yaml")
    print "Parameter save finished."
