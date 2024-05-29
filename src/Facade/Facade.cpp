/**
 * @file Facade.cpp
 * @brief Contains the implementation for the Facade class methods.
*/

#include "Facade.h"

/**
 * @brief Default constructor for the Facade class
*/
Facade::Facade(void): tofCamera(this), 
                        volumeObj(this) {
}

/**
 * @brief Main thread for the Facade class
*/
void Facade::loop() {

    if (!this->tofCamera.initialise()) {
        std::cout << "Failed initialisation of ToFCamera." << std::endl;
        return;
    }

    int fileIdx = 0;
    std::shared_ptr<open3d::geometry::PointCloud> o3dCloud(new open3d::geometry::PointCloud);

    for (;;) {
        
        o3dCloud->Clear();
        this->tofCamera.getDepthFrame();
        this->tofCamera.showDepthFrame(this->tofCamera.depthFrame, "Depth Frame");
        PointCloud::depthFrameToPointCloud(this->tofCamera.depthFrame, this->tofCamera.pipelineStruct, o3dCloud);

        switch (cv::waitKey(50)) {
            case 'q':
                return;
            case 's': {
                std::stringstream ss;
                ss << std::setw(4) << std::setfill('0') << fileIdx;
                std::string filePath = "testingPcd_" + ss.str();
                this->debugPrintLn("Saving PointCloud as " + filePath);
                PointCloud::writePointCloudToFile(*o3dCloud, filePath);
                fileIdx++;
                break;
            }
            case 'c': {
                PointCloud::writePointCloudToFile(*o3dCloud, this->backgroundFilePath);
                this->debugPrintLn("Taken background image.");
                this->debugPrintLn("Background Depth: " + std::to_string(this->volumeObj.getBackgroundDepth(this->backgroundFilePath)) + "mm");
                open3d::visualization::DrawGeometries({o3dCloud});
                this->gotBackground = true;
                break;
            }
            case 'v': {
                if (gotBackground) {
                    auto volume = this->volumeObj.calculateStockPileVolume(o3dCloud, PointCloud::readPointCloud(this->backgroundFilePath), this->showGeometries);
                    this->debugPrintLn("Volume of Stockpile: " + std::to_string(static_cast<double>(volume / 1000000.00l)) + "L");
                } else {
                    this->debugPrintLn("Must take image of background first.");
                }
                break;
            }
            case 'e': {
                auto volume = this->volumeObj.calculateStockPileVolume(o3dCloud, PointCloud::readPointCloud(this->backgroundFilePath), this->showGeometries);
                this->debugPrintLn("Volume of Stockpile: " + std::to_string(static_cast<double>(volume / 1000000.00l)) + "L");
                std::cout << "How many towels were in the frame?" << std::endl;
                int numTowels;
                std::cin >> numTowels;
                this->singleTowelVolume = static_cast<double>(volume / (double) numTowels);
                std::cout << "Single towel volume is: " << this->singleTowelVolume << std::endl;
                this->gotSingleTowelVolume = true;
                break;
            }
            case 'r': {
                if (this->gotSingleTowelVolume && this->gotBackground) {
                    auto volume = this->volumeObj.calculateStockPileVolume(o3dCloud, PointCloud::readPointCloud(this->backgroundFilePath), this->showGeometries);
                    int numTowels = std::round(volume / this->singleTowelVolume);
                    this->debugPrintLn("Volume of Stockpile: " + std::to_string(static_cast<double>(volume / 1000000.00l)) + "L");
                    this->debugPrintLn("Number of towels: " + std::to_string(numTowels));
                } else {
                    this->debugPrintLn("Must get single towel volume and background.");
                }
                break;
            }
            default:
                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
}

/**
 * @brief Print a debug line to the console
 * @param text Debug text to print
*/
void Facade::debugPrintLn(const std::string text) {
    std::cout << "\n- " << text << " -\n" << std::endl;
}