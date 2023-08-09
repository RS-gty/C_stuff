#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <list>
#include <chrono>

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace chrono;

Mat frame(const Vector2d& size, const Vector3d& camera, double angle, const list<Vector3d>& point, const double rotate=0) {
    /* Settings */
    double width = (int) size[0];
    double height = (int) size[1];

    /* image */
    Mat image = Mat::zeros(Size((int) width, (int) height), CV_8UC3);

    /* Space Initialization */
    double ratio;
    Vector3d w(width / 2, 0, 0);
    Vector3d h(0, height / 2, 0);
    double r = camera.norm() * tan(angle) / sqrt(pow(width, 2) + pow(height, 2));

    Vector3d rdir = Vector3d(0, 0, 1) * camera.norm();
    Vector3d rot = camera.cross(rdir);

    Quaterniond q1(pow(camera.norm(), 2) + camera.dot(rdir), rot[0], rot[1], rot[2]);
    Matrix3d m1 = q1.normalized().toRotationMatrix();

    for (auto &A: point) {
        ratio = - camera.dot(A) / camera.dot(camera - A);
        if(ratio >= 0) {
            Vector3d intersect = (m1 * A + ratio * (rdir - m1 * A)) / r;
            circle(image,
                   Point(intersect[0]+width/2, intersect[1]+height/2),
                   15, Scalar(0, 0, 255), -1);
        }
    }


    return image;
}

int main() {

    list<Vector3d> L;
    Vector3d v1(1, -1, 1);
    Vector3d v2(1, -1, -1);
    Vector3d v3(-1, -1, -1);
    Vector3d v4(-1, -1, 1);
    Vector3d v5(1, 1, 1);
    Vector3d v6(1, 1, -1);
    Vector3d v7(-1, 1, -1);
    Vector3d v8(-1, 1, 1);
    L = {v1, v2, v3, v4, v5, v6, v7, v8};
    VideoWriter video("test.mp4", cv::VideoWriter::fourcc('M', 'P', '4', '2'), 60.0, Size(1600, 900));
    auto start = system_clock::now();
    for (double i; i < 240; i = i + 1) {
        video << frame(Vector2d(1600, 900), Vector3d(3*sin(M_PI * i / 60), 3*cos(M_PI * i / 60), 0), M_PI / 2 - 0.4, L,
                       0);
    }
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout << double(duration.count()) * microseconds::period::num / microseconds::period::den << endl;

    return 0;
}