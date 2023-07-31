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

double V_angle(const Vector3d &v1, const Vector3d &v2) {
    double d = v1.dot(v2);
    return acos(d / (v1.norm() * v2.norm()));
}

Mat frame(Vector2d size, Vector3d camera, double angle, double rotate, list<Vector3d> point, string filepath) {
    /* Settings */
    double width = (int) size[0];
    double height = (int) size[1];

    /* Space Initialization */
    Mat image = Mat::zeros(Size((int) width, (int) height), CV_8UC3);

    Vector3d dir = camera;
    double n_dir = dir.norm();
    Vector3d w(width / 2, 0, 0);
    Vector3d h(0, height / 2, 0);

    Vector3d rdir = Vector3d(0, 0, 1);
    Vector3d rot = dir.cross(rdir);
    Quaterniond q1(sqrt(pow(dir.norm(), 2) * pow(dir.norm(), 2)) + dir.dot(rdir), rot[0], rot[1], rot[2]);
    Matrix3d m1 = q1.normalized().toRotationMatrix();
    double h1 = rdir.norm();

    for (auto &A: point) {
        Vector3d A_ = m1 * A;
        double h2 = A_[2];
        double coef = (1 - h2) * n_dir * tan(angle) / sqrt(pow(width, 2) + pow(height, 2));

        if (0 < h2 and h2 < n_dir) {
            if (abs((A_)[0]) <= abs(coef * width) and abs((A_)[1]) <= abs(coef * height)) {
                circle(image,
                       Point((A_)[0] * (h1 / (h1 + abs(h2))) / (n_dir * tan(angle) / sqrt(pow(width, 2) + pow(height, 2))) +
                             width / 2,
                             (A_)[1] * (h1 / (h1 + abs(h2))) / (n_dir * tan(angle) / sqrt(pow(width, 2) + pow(height, 2))) +
                             height / 2),
                       15, Scalar(0, 0, 255), -1);
            }
        } else if(h2 >= n_dir);
        else {
            circle(image,
                   Point((A_)[0] * (h1 / (h1 + abs(h2))) / (n_dir * tan(angle) / sqrt(pow(width, 2) + pow(height, 2))) +
                         width / 2,
                         (A_)[1] * (h1 / (h1 + abs(h2))) / (n_dir * tan(angle) / sqrt(pow(width, 2) + pow(height, 2))) +
                         height / 2),
                   15, Scalar(0, 0, 255), -1);
        }
    }

    return image;
}

int main() {

    list<Vector3d> L;
    Vector3d v1(1, -1, -3);
    Vector3d v2(1, -1, -1);
    Vector3d v3(-1, -1, -1);
    Vector3d v4(-1, -1, -3);
    Vector3d v5(1, 1, -3);
    Vector3d v6(1, 1, -1);
    Vector3d v7(-1, 1, -1);
    Vector3d v8(-1, 1, -3);
    L = {v1, v2, v3, v4, v5, v6, v7, v8};
    VideoWriter video("test.mp4", cv::VideoWriter::fourcc('M', 'P', '4', '2'), 60.0, Size(1600, 900));
    auto start = system_clock::now();
    for (double i; i < 120; i = i + 1) {
        string fp = "G:/Programs/C++/New/data/IM" + to_string((int) i) + ".jpg";
        video << frame(Vector2d(1600, 900), Vector3d(3*sin(M_PI * i / 60), 3*cos(M_PI * i / 60), 3).normalized(), M_PI / 3, 0,
                       L, fp);
    }
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout << double(duration.count()) * microseconds::period::num / microseconds::period::den << endl;

    return 0;
}