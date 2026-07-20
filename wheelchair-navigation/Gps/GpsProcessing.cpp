#include "GpsProcessing.hpp"
#include "GpsData.hpp"
#include "GpsObject.hpp"
#include "GPSLocation.hpp"
#include <cmath>
#include <algorithm>
#include <iostream> // נועד להדפסות הדיבאג

GpsObject GpsProcessing::process(std::stringstream& input,double tss) {
    this->ts=tss;
    input.clear(); 
    input.seekg(0);
    std::cout << "[DEBUG GpsProcessing.cpp] process התחילה עם קלט." << std::endl;
    m_parser.updateFromStream(input);
    
    std::cout << "[DEBUG GpsProcessing.cpp] fillGPSLocation הסתיימה. כעת מייצר את אובייקט הפלט..." << std::endl;
    
    GpsObject GO = createObject(m_parser); 
    return GO;
}

GpsObject GpsProcessing::createObject(GPSLocation& parser) {
    GpsObject GO;
    GO.timestamp = this->ts;

    GO.x_local = parser.position.latitude;
    GO.y_local = parser.position.longitude;
    GO.z_local = parser.position.altitudeMSL;

    GO.speed   = parser.navigation.speedOverGround;
    GO.heading = parser.navigation.heading;

    parser.calculateConfidence();
    GO.confidence = parser.quality.confidenceLevel;
    GO.isValid = parser.quality.statusActive;

    // המרת lat/lon → מטרים מקומיים + בניית Z ו-H
    GpsData gd;
    gd.process(GO);

    std::cout << "[GPS][DEBUG] createObject x_local=" << GO.x_local << " y_local=" << GO.y_local << std::endl;
    return GO;
}