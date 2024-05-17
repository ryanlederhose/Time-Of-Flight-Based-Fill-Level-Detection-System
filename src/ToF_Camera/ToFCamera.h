/**
 * @file ToFCamera.h
 * @brief Contains the definition for the ToFCamera class methods.
*/

#pragma once

#define _CRT_SECURE_NO_WARNINGS

#include "libsynexens3/libsynexens3.h"
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

// Forward declaration of Facade class
class Facade;

class ToFCamera {

    public:
        ToFCamera(Facade *facade);
        bool initialise(void);

        sy3::depth_frame *depthFrame;
        sy3::ir_frame *irFrame;
        sy3::sy3_error errorStruct;
        sy3::context *contextStruct;
        sy3::device *deviceStruct;
        sy3::pipeline *pipelineStruct;
        sy3::config *configStruct;
        sy3::frameset *framesetStruct;

        static volatile int fpsCount;
        static volatile bool cameraStarted;
        static volatile int frameCount;

        void showDepthFrame(sy3::depth_frame *frame, const char *name);
        void showIrFrame(sy3::ir_frame *frame, const char *name);
        void getDepthFrame(void);
        void getIrFrame(void);

    private:
        std::thread fpsThread;

        Facade *facade;

        void printDeviceInfo(sy3::device *dev);
        void printSupportFormat(sy3::device *dev, sy3::sy3_error &e);
        static void calculateFrameRate(void);
};
