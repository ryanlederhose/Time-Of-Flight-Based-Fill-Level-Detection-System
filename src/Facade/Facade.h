/**
 * @file Facade.h
 * @brief Contains the definition for the Facade class methods.
*/

#pragma once

#include "../ToF_Camera/ToFCamera.h"
#include "../PointCloud/PointCloud.h"
#include "../Volume/Volume.h"

/**
 * @class Facade
 * @brief Main interface for the application
*/
class Facade {

    public:
        ToFCamera tofCamera;
        Volume volumeObj;

        Facade(void);
        void loop(void);
        void debugPrintLn(const std::string text);

        const std::string backgroundFilePath = "../src/PCDWorkingDir/background";
        bool gotBackground = false;
        bool gotSingleTowelVolume = false;
        double singleTowelVolume = 0.0;
        const bool showGeometries = true;
    
    private:
};