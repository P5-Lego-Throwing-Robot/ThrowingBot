// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <zbar.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include "lego_throw/camera.h"

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <typeinfo>
#include <iostream>

// Struct to store the scanned QR codes
struct Object : public cv::_InputArray {
    std::string data;
    std::vector<cv::Point> location;
    cv::Point center;
};

// Struct to store a set of frames from realsense
struct Frame {
    rs2::frameset frameset;
    rs2::frame colorFrame;
    rs2::frame depthFrame;
    uint height{};
    uint width{};
    cv::Mat matImage;
    uint32_t count = 0;
};

// Add names for QR codes here if you add more QR codes to scene
const std::vector<std::string> qrCustomNames = {
        "ID_2",
        "ID_3",
        "ID_4",
        "ID_5"
};
// Global variable to keep track of lego boxes that has been scanned
std::vector<Object> qrCustomNamesDetected;
// ROS service client
ros::ServiceClient client;


// Get a frame from realsense. Purpose of this function is to group the image collecting here.
void retrieveFrame(const rs2::pipeline &pipe, Frame *frame) {
    frame->frameset = pipe.wait_for_frames();                               // Get a frameset from realsense pipeline object. This object holds all data from the realsense camera
    frame->colorFrame = frame->frameset.get_color_frame();
    //frame->depthFrame = frame->frameset.get_depth_frame();                // We do not need depth frame for anything.. yet
    frame->width = frame->colorFrame.as<rs2::video_frame>().get_width();    // Width for OpenCV Mat object
    frame->height = frame->colorFrame.as<rs2::video_frame>().get_height();  // Height for OpenCV Mat object
    frame->matImage = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC3, (void *) frame->colorFrame.get_data(),
                              cv::Mat::AUTO_STEP);                          // Construct openCV mat object used in zBar and homogryaphy


    // Increment frame number
    frame->count++;
}


// Find and decode barcodes and QR codes
void decode(cv::Mat &im, std::vector<Object> &decodedObjects) {

    // Create zbar scanner
    zbar::ImageScanner scanner;

    // Configure scanner
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    // Convert image to grayscale
    cv::Mat imGray;
    cvtColor(im, imGray, cv::
    COLOR_BGR2GRAY);

    cv::imshow("grey", imGray);
    // Wrap image data in a zbar image
    zbar::Image image(im.cols, im.rows, "GREY", (uchar *) imGray.data, im.cols * im.rows);

    // Scan the image for barcodes and QRCodes
    int n = scanner.scan(image);

    // Print results
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        Object obj;
        obj.data = symbol->get_data();

        // Obtain location
        for (int i = 0; i < symbol->get_location_size(); i++) {
            obj.location.emplace_back(symbol->get_location_x(i), symbol->get_location_y(i));
        }

        // calculate center coords
        int sumX = 0, sumY = 0;
        for (auto &i : obj.location) {
            sumX += i.x;
            sumY += i.y;
        }
        obj.center.x = sumX / obj.location.size();
        obj.center.y = sumY / obj.location.size();

        decodedObjects.push_back(obj);

    }
}


void calculateTransformation(cv::Point2f robot, cv::Point2f legoBox, double angle) {

    // Given we know:
    // -- where the robot QR code is
    // -- the orientation of the Robot QR code
    boost::numeric::ublas::vector<double> aP(2);        // Location of LEGO box from table frame
    aP[0] = legoBox.x;
    aP[1] = legoBox.y;
    boost::numeric::ublas::vector<double> aPBorg(2);    // Location of Robot Oregon
    aPBorg[0] = robot.x;
    aPBorg[1] = robot.y;
    boost::numeric::ublas::vector<double> bP(2);        // Location of LEGO box from Robot frame

    // Rotate (angle) around z-axis.

    // Initiate 2x2 matrix
    boost::numeric::ublas::matrix<double> mat(2, 2);
    // populate rotation matrix
    mat(0, 0) = cos(angle);
    mat(0, 1) = -sin(angle);
    mat(1, 0) = sin(angle);
    mat(1, 1) = cos(angle);

    // Transpose matrix because we need the inverse
    mat = boost::numeric::ublas::trans(mat);
    // Multiply using eq: 2.17 in robotics book isolated for bP.
    bP = boost::numeric::ublas::prod(mat, aP) - boost::numeric::ublas::prod(mat, aPBorg);

    std::cout << bP << std::endl;

}

void calculateAngle(std::vector<Object> transQR, cv::Mat hMat) {
    Object orig, robot;
    if (transQR[0].data == "00") {
        orig = transQR[0];
        robot = transQR[1];
    } else {
        orig = transQR[1];
        robot = transQR[0];
    }

    // Put origin points into vector
    std::vector<cv::Point2f> origPoint(3);
    // Corner points used for line
    origPoint[0].x = orig.location[0].x;
    origPoint[0].y = orig.location[0].y;
    origPoint[1].x = orig.location[3].x;
    origPoint[1].y = orig.location[3].y;
    origPoint[2] = orig.center;
    // transform points
    perspectiveTransform(origPoint, origPoint, hMat);      // Multiply point with homography matrix


    // Put robot points into vector
    std::vector<cv::Point2f> robotPoints(3);
    robotPoints[0].x = robot.location[0].x;
    robotPoints[0].y = robot.location[0].y;
    robotPoints[1].x = robot.location[3].x;
    robotPoints[1].y = robot.location[3].y;
    // Center point
    robotPoints[2] = robot.center;

    // transform points
    perspectiveTransform(robotPoints, robotPoints, hMat);      // Multiply point with homography matrix

    // Calculate angle between planes
    // First origin line
    float m1, m2;
    float x1, x2, y1, y2;

    x1 = origPoint[0].x;
    y1 = origPoint[0].y;
    x2 = origPoint[1].x;
    y2 = origPoint[1].y;

    // Finding slope for base QR
    m1 = (y2 - y1) / (x2 - x1);
    //printf("m1 = %f \n", m1);


    x1 = robotPoints[0].x;
    y1 = robotPoints[0].y;
    x2 = robotPoints[1].x;
    y2 = robotPoints[1].y;

    //Finding slope for base QR
    m2 = (y2 - y1) / (x2 - x1);

    // return angle
    double angle = atan((m2 - m1) / (1 + m1 * m2));

    // Get the lego box location transformed
    std::vector<cv::Point2f> legoBox = {transQR[2].center};
    perspectiveTransform(legoBox, legoBox, hMat);      // Multiply point with homography matrix

    // Remember to add offset between robot and QRd
    double robotOffset = 41;
    robotPoints[2].y += -41;
    calculateTransformation(robotPoints[2], legoBox[0] , angle);
    printf("Angle: %f\n", angle);
}

void doHomography(const std::vector<Object> objects, cv::Mat colorImage) {
    // Find homography matrix needs 8 points.
    std::vector<cv::Point2f> surfaceQR(4);     // Four corners of the real world plane
    std::vector<cv::Point2f> cameraQR(4);      // Four corners of the image plane


    // The QR codes that is scanned from zBar does not come ordered.
    // Thus we want to sort the corresponding points in SurfaceQR and cameraQR such that corresponding points is at the same index.
    int amountQRCornersFound = 0;

    //ROBOT TABLE POINTS
    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(0, 0);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "01") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(71, 0);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "02") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(0, 74);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "03") {
            surfaceQR[amountQRCornersFound] = cv::Point2f(71, 74);
            cameraQR[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
    }
    /*

    // A4 PAPER TEST POINTS IN mm
    for (int i = 0; i < objects.size(); ++i) {
        if (objects[i].data == "00") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(0, 0);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "01") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(145, 0);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "02") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(0, 235);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
        if (objects[i].data == "03") {
            cornersForPlane[amountQRCornersFound] = cv::Point2f(145, 235);
            QrCamCoordinates[amountQRCornersFound] = objects[i].center;
            amountQRCornersFound++;
        }
    }
    */
    // If we didn't find 4 QR corners OR We didn't find any lego boxes
    // then stop executing and return to main loop
    if (amountQRCornersFound != 4)
        return;

    //calculate Homography matrix from 4 sets of corresponding points
    cv::Mat hMatrix = findHomography(cameraQR, surfaceQR);

    //cv::Mat homographyImage;
    //warpPerspective(colorImage, homographyImage, hMatrix, cv::Size(800, 870));


    // Check for the lego boxes QR codes.
    for (int i = 0; i < objects.size(); i++) {
        std::vector<std::string>::const_iterator it;
        it = std::find(std::begin(qrCustomNames), std::end(qrCustomNames), objects[i].data);
        if (it != std::end(qrCustomNames)) {

            // Check if the packaging QR code already exists
            for (int j = 0; j < qrCustomNamesDetected.size(); j++) {
                if (objects[i].data ==
                    qrCustomNamesDetected[j].data)   // It exists, then return to main loop, else continue with the new QR code
                    return;
            }
          //  qrCustomNamesDetected.push_back(objects[i]);                     // Save new QR code so we dont repack same lego box

            std::vector<cv::Point2f> legoBox = {
                    objects[i].center};    // Load point in as a cv::Point2f because that's perspectiveTransform's input.
            perspectiveTransform(legoBox, legoBox, hMatrix);      // Multiply point with homography matrix

            float xRobotOffset = -28;   // robot offset form real world surface plane in cm
            float yRobotOffset = 73;    // robot offset form real world surface plane in cm

            float xWithOffsetInMeters = ((legoBox[0].x + (xRobotOffset))) / 100.0f;  // Apply offset
            float yWithOffsetInMeters = ((legoBox[0].y + yRobotOffset)) / 100.0f;    // Apply offset

            // calculate transformation
            std::vector<Object> transQR;
            for (int i = 0; i < objects.size(); ++i) {
                if (objects[i].data == "00") {
                    transQR.push_back(objects[i]);
                }
                if (objects[i].data == "ID_1") {
                    transQR.push_back(objects[i]);
                }
            }
            // Push back the new detected QR code
            transQR.push_back(objects[i]);

            // If we found orego, robot and a lego box.
            if (transQR.size() == 3) {
                calculateAngle(transQR, hMatrix);
                qrCustomNamesDetected.push_back(objects[i]);

                lego_throw::camera camSrv;

                camSrv.request.x = xWithOffsetInMeters;
                camSrv.request.y = yWithOffsetInMeters;
                camSrv.request.z = 0.045;
                camSrv.request.data = *it;

                if (client.call(camSrv)) printf("Response status: %i\n", camSrv.response.status);
            }

        }
    }

}


int main(int argc, char *argv[]) {

    // -- REALSENSE SETUP --
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming from camera with default recommended configuration
    pipe.start();

    // --- ROS STUFF ---
    ros::init(argc, argv, "realsenseVision");
    ros::NodeHandle node_handle;

    // Creating the client
    client = node_handle.serviceClient<lego_throw::camera>("camera");
    //client.waitForExistence();

    // --- GROUP 563 ---
    Frame frame;                               // Place to store realsense frames
    std::vector<Object> decodedObjects;       // A vector to store scanned QR codes in


    printf("Start filming the scene\n");
    while (ros::ok()) {

        retrieveFrame(pipe,
                      &frame);                                            // Get a set of frames from realsense Camera
        decode(frame.matImage, decodedObjects);                           // Scan image for QR codes

        if (decodedObjects.size() > 3)                                          // If we have 4 or more QR codes then -
            doHomography(decodedObjects, frame.matImage);                       // -Calculate homography from QR codes

        cvtColor(frame.matImage, frame.matImage, cv::COLOR_BGR2RGB);      // Convert to RGB to display correct colors
        cv::imshow("Image", frame.matImage);                           // Display image for user feedback
        cv::imwrite("/home/magnus/ThrowingBot/camerafile.png", frame.matImage);
        if (cv::waitKey(25) == 27) break;                                // If ESC is pushed then break loop

        decodedObjects.clear();                                                // Reset vector of scanned QR codes
        //memset(&frame, 0x00, sizeof(frame));                                 // Reset frames object --> Not necessary but nice to do

    }
    return 0;
}
