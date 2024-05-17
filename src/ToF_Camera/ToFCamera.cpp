/**
 * @file ToFCamera.cpp
 * @brief Contains the implementation of the ToFCamera class methods. 
*/

#include "ToFCamera.h"
#include "../Facade/Facade.h"

volatile bool ToFCamera::cameraStarted = false;
volatile int ToFCamera::fpsCount = 0;
volatile int ToFCamera::frameCount = 0;

/**
 * @brief Default constructor for ToFCamera 
 * @param facade Pointer to Facade class object
*/
ToFCamera::ToFCamera(Facade *facade): facade(facade) {
}

/**
 * @brief Initialise the CS-20 ToF Camera
 * 
 * Initialises the streams for both depth and IR views, and also starts the pipeline.
 * @retval True if successfully initialised ToF Camera, False otherwise
*/
bool ToFCamera::initialise() {

    // Create the camera context and query the device
    this->contextStruct = sy3::sy3_create_context(this->errorStruct);
    this->deviceStruct = this->contextStruct->query_device(this->errorStruct);
    if (this->errorStruct != sy3::sy3_error::SUCCESS) {
        printf("Error: %s \r\n", sy3::sy3_error_to_string(this->errorStruct));
        return false;
    }
    this->printSupportFormat(this->deviceStruct, this->errorStruct);
    this->printDeviceInfo(this->deviceStruct);

    // Create the camera pipeline and config
    this->pipelineStruct = sy3::sy3_create_pipeline(this->contextStruct, this->errorStruct);
    this->configStruct = sy3::sy3_create_config(this->contextStruct, this->errorStruct);

    // Enable the depth and IR streams
    this->configStruct->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, this->errorStruct);
    this->configStruct->enable_stream(sy3::sy3_stream::SY3_STREAM_IR, DEPTH_WIDTH, DEPTH_HEIGHT, this->errorStruct);
    
    // Start the camera pipeline
    this->pipelineStruct->start(this->configStruct, this->errorStruct);
    this->cameraStarted = true;

    this->fpsThread = std::thread(calculateFrameRate);

    return true;
}

/**
 * @brief Get the depth frame from the ToF camera.
 * 
 * Waits for frames and than get the depth frame of the ToF camera. Saves to member variable.
*/
void ToFCamera::getDepthFrame() {
    for(;;) {
        this->framesetStruct = this->pipelineStruct->wait_for_frames(SY3_DEFAULT_TIMEOUT, this->errorStruct);
        this->depthFrame = this->framesetStruct->get_depth_frame();

        if ((this->framesetStruct != nullptr) && (this->depthFrame != nullptr)) {
            return;
        } 
    }
}

/**
 * @brief Get the IR frame from the ToF camera.
 * 
 * Waits for frames and than get the IR frame of the ToF camera. Saves to member variable.
*/
void ToFCamera::getIrFrame() {
    for (;;) {
        this->framesetStruct = this->pipelineStruct->wait_for_frames(SY3_DEFAULT_TIMEOUT, this->errorStruct);
        this->irFrame = this->framesetStruct->get_ir_frame();

        if ((this->framesetStruct != nullptr) && (this->irFrame != nullptr)) {
            return;
        } 
    }
}

/**
 * @brief Show the depth frame using OpenCV
 * @param frame Depth frame to show
 * @param name Name of window
*/
void ToFCamera::showDepthFrame(sy3::depth_frame *frame, const char *name) {
	if (frame) {
        frameCount++;

		cv::Mat gray16(frame->get_height(), frame->get_width(), CV_16UC1, frame->get_data());
		cv::Mat tmp;
		cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
		cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, gray8);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, gray8);

		uint8_t* depth_color = frame->apply_colormap(((uint16_t*)frame->get_data()), frame->get_width(), frame->get_height());
		cv::Mat yuvImg(frame->get_height(), frame->get_width(), CV_8UC3, depth_color);

		std::string msg = std::to_string(frame->get_width()) + "x" + std::to_string(frame->get_height()) + " fps:" + std::to_string(fpsCount);
		int font_face = cv::FONT_HERSHEY_COMPLEX;
		double font_scale = 1;
		int thickness = 2;
		int baseline;
		cv::Size text_size = cv::getTextSize(msg, font_face, font_scale, thickness, &baseline);

		cv::Point origin;
		origin.x = yuvImg.cols / 2 - text_size.width / 2;
		origin.y = 0 + text_size.height;
		cv::putText(yuvImg, msg, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 2, 0);

		cv::namedWindow("MAP_COLOR", cv::WINDOW_NORMAL);
		cv::imshow("MAP_COLOR", yuvImg);

		sy3::sy3_intrinsics intrinsics = frame->get_profile()->get_intrinsics();
	}
}

/**
 * @brief Show the IR frame using OpenCV
 * @param frame IR frame to show
 * @param name Name of window
*/
void ToFCamera::showIrFrame(sy3::ir_frame *frame, const char *name) {
	if (frame) {
		cv::Mat gray16(frame->get_height(), frame->get_width(), CV_16UC1, frame->get_data());
		cv::Mat tmp;
		cv::Mat gray8 = cv::Mat::zeros(gray16.size(), CV_8U);
		cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, gray8);
		cv::namedWindow(name, cv::WINDOW_NORMAL);
		cv::imshow(name, gray8);
	}
}

/**
 * @brief Calculate the framerate of the camera 
*/
void ToFCamera::calculateFrameRate() {
    double lastCalculationTime = cv::getTickCount() / cv::getTickFrequency() * 1000;
    while (cameraStarted) {
        double curTime = cv::getTickCount() / cv::getTickFrequency() * 1000;
        if ((curTime - lastCalculationTime) > 1000) {
            fpsCount = frameCount;
            frameCount = 0;
            lastCalculationTime = curTime;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * @brief Print the device info of the camera
 * @param dev Device structure of camera
*/
void ToFCamera::printDeviceInfo(sy3::device* dev) {
	sy3::sy3_error e;
	printf("	CAMERA NAME: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_NAME, e));
	printf("    SERIAL NUMBER: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_SERIAL_NUMBER, e));
	printf("    FIRMWARE VERSION: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_FIRMWARE_VERSION, e));
	printf("    RECOMMENDED FIRMWARE VERSION: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION, e));
	printf("    RECONSTRUCTION VERSION: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_RECONSTRUCTION_VERSION, e));
	printf("    RECOMMENDED RECONSTRUCTION VERSION: %s\n\n", sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_RECOMMENDED_RECONSTRUCTION_VERSION, e));
}

/**
 * @brief Print the support format for the camera
 * @param dev Device structure of camera
 * @param e Error struct
*/
void ToFCamera::printSupportFormat(sy3::device* dev, sy3::sy3_error& e) {

	std::vector<sy3::sy3_stream> support_stream = dev->get_support_stream(e);
	for (int i = 0; i < support_stream.size(); i++)
	{
		printf("support stream:%s \n", sy3_stream_to_string(support_stream[i]));
		std::vector<sy3::sy3_format> support_format = dev->get_support_format(support_stream[i], e);
		for (int j = 0; j < support_format.size(); j++)
		{
			printf("\t\t support format:%d x %d \n", support_format[j].width, support_format[j].height);
		}
	}
}