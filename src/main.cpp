#include "opencv2\opencv.hpp"
#include <ctime>
#include <iostream>

using namespace std;
using namespace cv;

#define multiple 1.5      /*倍率，换算目标点所用*/
#define min -1            /*极小量*/
#define max 10000         /*极大量*/
#define pointSetnumeber 5 /*点集大小*/
#define speedSetnumber 2  /*速度集合大小*/
#define blue 0            /*蓝色*/
#define red 1             /*红色*/
#define change 1          /*偏移*/
#define retain 0          /*保持*/

/*________________变量区_________________*/
Point2f pointSet[pointSetnumeber]; /*定义点集,存放目标点*/
int pointNumber = 0;               /*配合pointSet[pointSetnumeber]，存放点*/
int runNumber = 0;                 /*存放目标点的次数，次数达标开始预测*/
float speed;
float acceleratedSpeed;         /*速度，加速度*/
float speedSet[speedSetnumber]; /*速度集合*/
int speedNumber = 0;            /*配合speedSet[speedSetnumber]，存放速度*/
float predictdistance;
Point2f predictPoint;   /*定义预测距离和预测点*/
float lastDistance = 0; /*初始化距离*/
int frame;              /*帧数*/
int color;              /*控制识别颜色，0代表蓝色，1代表红色*/

int minId;
double minArea = max; /*存放中心处面积*/
Point2f center;       /*定义外接圆中心坐标*/
float radius;         /*定义外接圆半径*/
Point2f oldcenter;    /*定义旧外接圆圆心坐标*/
Point2f newcenter;    /*定义新外接圆中心坐标*/
float newradius;      /*定义新外接圆半径*/

int maxId;
double maxArea = min; /*存放筛选后最大面积轮廓*/
float referenceR;     /*半径参考长度*/
Point2f rectMid;      /*半径参考长度所在轮廓几何中心*/
Point2f target;       /*目标点*/
int state;            /*是否偏移*/
/*-------------------------------------------------------------------------*/

/*________________函数声明___________________*/
float distance(Point2f lastPoint, Point2f presentPoint); /*计算两点间的距离*/
/*---------------------------------------------------------------------*/

float distance(Point2f lastPoint, Point2f presentPoint)
{
    float distance;
    distance = sqrt((presentPoint.x - lastPoint.x) * (presentPoint.x - lastPoint.x) + (presentPoint.y - lastPoint.y) * (presentPoint.y - lastPoint.y));
    return distance;
}

int main()
{
    VideoCapture cap("../resources/PowerRune.mp4");
    Mat image;
    color = red; /*hong色*/
    for (;;)
    {
        cap.read(image);
        /*开始处理图像*/
        clock_t start = clock();
        /*改变大小，提高帧率*/
        resize(image, image, Size(image.cols * 0.35, image.rows * 0.35));
        /*测试效果展示*/
        Mat test;
        image.copyTo(test);
        /*容器，存放分离通道后的图像*/
        vector<Mat> imgChannels;
        split(image, imgChannels);
        /*红色*/
        Mat redImage = imgChannels.at(2) - imgChannels.at(0);
        /*imshow("1", redImage);
        蓝色
        Mat redImage = imgChannels.at(0) - imgChannels.at(2);*/
        Mat binaryImage;
        Mat binaryImagecricle;
        /*二值化*/
        threshold(redImage, binaryImagecricle, 100, 255, THRESH_BINARY);
        threshold(redImage, binaryImage, 40, 255, THRESH_BINARY);
        /*腐蚀操作*/
        Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
        Mat dstImage;
        erode(binaryImagecricle, dstImage, element);
        /*找到圆周运动的圆心——R*/
        state = retain;
        vector<vector<Point>> outlines;
        vector<Vec4i> hierarchies;
        findContours(dstImage, outlines, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
        for (int i = 0; i < outlines.size(); i++)
        {

            vector<Point> points;
            double area = contourArea(outlines[i]);
            /*面积排除噪声*/
            if (area < 10 || area > 10000)
                continue;
            /*找到没有父轮廓的轮廓*/
            if (hierarchies[i][3] >= 0 && hierarchies[i][3] < outlines.size())
                continue;
            /*找有子轮廓的*/
            if (hierarchies[i][2] < 0 || hierarchies[i][2] >= outlines.size())
                continue;
            /*控制误差范围*/
            if (area <= minArea + 10 && area >= minArea - 20)
            {
                minArea = area;
                minId = i;
                continue;
            }
            /*面积最小的轮廓*/
            if (minArea >= area)
            {
                minArea = area;
                minId = i;
            }
        }
        /*防止minId不在范围内报错*/
        if (minId >= 0 && minId < outlines.size())
        {
            /*画外接圆并找到圆心*/

            minEnclosingCircle(Mat(outlines[minId]), newcenter, newradius);
            /*减小抖动，误差*/
            if (distance(newcenter, center) < 2)
            {
            }
            else
            {
                oldcenter = center;
                center = newcenter;
                state = change;
            }
            if (fabs(newradius - radius) < 2)
            {
            }
            else
            {
                radius = newradius;
            }
            circle(test, center, radius, Scalar(0, 0, 255), 1, 8, 0);
        }
        else
        {
            continue;
        }

        /*膨胀操作*/
        element = getStructuringElement(0, Size(3, 3));
        Mat dilateImage;
        /*dilate最后一个数字是膨胀次数*/
        dilate(binaryImage, dilateImage, element, Point(-1, -1), 2);
        /*轮廓发现*/
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(dilateImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        for (int i = 0; i < contours.size(); i++)
        {
            vector<Point> points;
            double area = contourArea(contours[i]);
            /*面积排除噪声*/
            if (area < 20 || area > 10000)
                continue;
            /*找到没有父轮廓的轮廓*/
            if (hierarchy[i][3] >= 0 && hierarchy[i][3] < contours.size())
                continue;
            /*找没子轮廓的*/
            if (hierarchy[i][2] >= 0 && hierarchy[i][2] < contours.size())
                continue;
            /*找面积最大的轮廓*/
            if (maxArea <= area)
            {
                maxArea = area;
                maxId = i;
            }
            /*控制误差范围*/
            if (area <= maxArea + 50 && area >= maxArea - 50)
            {
                maxArea = area;
                maxId = i;
            }
            cout << maxArea << endl;
        }
        if (maxId >= 0 && maxId < contours.size())
        {
            /*计算矩*/
            Moments rect;
            rect = moments(contours[maxId], false);
            /*计算中心矩:*/
            Point2f rectmid;
            rectmid = Point2f(rect.m10 / rect.m00, rect.m01 / rect.m00);
            /*画出需打部位轮廓*/
            drawContours(test, contours, maxId, Scalar(0, 255, 255), 1, 8);

            /*减小抖动*/
            if (runNumber < 2)
            {
                referenceR = distance(rectmid, center);
                rectMid = rectmid;
            }
            else if (distance(rectmid, center) <= referenceR + 2 && distance(rectmid, center) >= referenceR - 2 && distance(rectmid, rectMid) < 0.5)
            {
            }
            else
            {
                referenceR = distance(rectmid, center);
                rectMid = rectmid;
            }

            /*画出样本部位中心点*/
            circle(test, rectMid, 1, Scalar(0, 255, 255), -1, 8, 0);
            /*2：1计算需打击部位,存放*/
            /*第一象限*/
            if (rectMid.x >= center.x && rectMid.y <= center.y)
            {
                target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y - (center.y - rectMid.y) * multiple);
            }
            /*第二象限*/
            if (rectMid.x <= center.x && rectMid.y <= center.y)
            {
                target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y - (center.y - rectMid.y) * multiple);
            }
            /*第三象限*/
            if (rectMid.x <= center.x && rectMid.y >= center.y)
            {
                target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y + (rectMid.y - center.y) * multiple);
            }
            /*第四象限*/
            if (rectMid.x >= center.x && rectMid.y >= center.y)
            {
                target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y + (rectMid.y - center.y) * multiple);
            }
            circle(test, target, 1, Scalar(0, 255, 255), -1, 8, 0);

            /*将几何中心点存入点集*/
            pointSet[pointNumber] = target;
            pointNumber++;
            /*实现新点替换旧点*/
            if (pointNumber == pointSetnumeber)
            {
                pointNumber = 0;
            }
        }
        else
        {
            continue;
        }
        /*算偏移*/
        if (state == change)
        {
            float xchange;
            float ychange;
            xchange = center.x - oldcenter.x;
            ychange = center.y - oldcenter.y;
            /*改变点集*/
            if (pointNumber == 0)
            {
                pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
                pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
                pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
                pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
            }
            if (pointNumber == 1)
            {
                pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
                pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
                pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
                pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
            }
            if (pointNumber == 2)
            {
                pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
                pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
                pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
                pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
            }
            if (pointNumber == 3)
            {
                pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
                pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
                pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
                pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
            }
            if (pointNumber == 4)
            {
                pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
                pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
                pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
                pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
            }
        }

        /*预测*/
        if (runNumber > pointSetnumeber)
        {

            int i = pointNumber - 1; // 取最新的点算速度
            int number1 = i;
            if (number1 < 0)
            {
                number1 += pointSetnumeber;
            }
            int number2 = i - 1;
            if (number2 < 0)
            {
                number2 += pointSetnumeber;
            }
            int number3 = i - 3;
            if (number3 < 0)
            {
                number3 += pointSetnumeber;
            }
            int number4 = i - 4;
            if (number4 < 0)
            {
                number4 += pointSetnumeber;
            }
            /*取最近四点，算速度，求加速度*/
            speed = distance(pointSet[number1], pointSet[number2]) * frame;
            speedSet[0] = speed;
            speed = distance(pointSet[number3], pointSet[number4]) * frame;
            speedSet[1] = speed;
            acceleratedSpeed = fabs((speedSet[0] - speedSet[1]) * frame);

            /* X = V0T + 1 / 2AT'2，通过距离公式，算预测的打击点距离 */
            predictdistance = 4.5 * speedSet[0] / frame + 1 / 2 * acceleratedSpeed / frame / frame * 18;

            /*算出预测时x, y需增加值的比值*/

            float xRatio, yRatio;
            xRatio = fabs(pointSet[number1].x - pointSet[number2].x) / distance(pointSet[number1], pointSet[number2]);
            yRatio = fabs(pointSet[number1].y - pointSet[number2].y) / distance(pointSet[number1], pointSet[number2]);
            /*第一象限内  顺  三逆*/
            if (pointSet[number1].x >= pointSet[number2].x && pointSet[number1].y >= pointSet[number2].y)
            {
                predictPoint = Point2f(pointSet[number1].x + predictdistance * xRatio, pointSet[number1].y + predictdistance * yRatio);
            }
            /*第二象限内  顺  四逆*/
            if (pointSet[number1].x >= pointSet[number2].x && pointSet[number1].y <= pointSet[number2].y)
            {
                predictPoint = Point2f(pointSet[number1].x + predictdistance * xRatio, pointSet[number1].y - predictdistance * yRatio);
            }
            /*第三象限内  顺  一逆*/
            if (pointSet[number1].x <= pointSet[number2].x && pointSet[number1].y <= pointSet[number2].y)
            {
                predictPoint = Point2f(pointSet[number1].x - predictdistance * xRatio, pointSet[number1].y - predictdistance * yRatio);
            }
            /*第四象限内  顺  二逆*/
            if (pointSet[number1].x <= pointSet[number2].x && pointSet[number1].y >= pointSet[number2].y)
            {
                predictPoint = Point2f(pointSet[number1].x - predictdistance * xRatio, pointSet[number1].y + predictdistance * yRatio);
            }

            /*减少预测抖动*/
            lastDistance = predictdistance;

            /*向轨迹拟合路径*/
            /*画圆*/
            circle(test, center, distance(center, target) + 3, Scalar(0, 255, 255), 1, 8, 0);
            /*预测点像圆弧靠拢*/
            /*第一象限*/
            if (predictPoint.x >= center.x && predictPoint.y <= center.y)
            {
                predictPoint = Point2f(center.x + (predictPoint.x - center.x) * distance(center, target) / distance(center, predictPoint), center.y - (center.y - predictPoint.y) * distance(center, target) / distance(center, predictPoint));
            }
            /*第二象限*/
            if (predictPoint.x <= center.x && predictPoint.y <= center.y)
            {
                predictPoint = Point2f(center.x - (center.x - predictPoint.x) * distance(center, target) / distance(center, predictPoint), center.y - (center.y - predictPoint.y) * distance(center, target) / distance(center, predictPoint));
            }
            /*第三象限*/
            if (predictPoint.x <= center.x && predictPoint.y >= center.y)
            {
                predictPoint = Point2f(center.x - (center.x - predictPoint.x) * distance(center, target) / distance(center, predictPoint), center.y + (predictPoint.y - center.y) * distance(center, target) / distance(center, predictPoint));
            }
            /*第四象限*/
            if (predictPoint.x >= center.x && predictPoint.y >= center.y)
            {
                predictPoint = Point2f(center.x + (predictPoint.x - center.x) * distance(center, target) / distance(center, predictPoint), center.y + (predictPoint.y - center.y) * distance(center, target) / distance(center, predictPoint));
            }
            /*画出预测点*/
            circle(test, predictPoint, 2, Scalar(0, 0, 255), -1, 8, 0);
        }

        imshow("1", test);
        imshow("2", dilateImage);
        runNumber++;
        clock_t end = clock();
        frame = 1 / (double)(end - start) * CLOCKS_PER_SEC;
        cout << frame << "帧" << endl;
        if (cv::waitKey(50) == 'q')
        {
            break;
        }
    }
}

