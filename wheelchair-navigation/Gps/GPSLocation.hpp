#pragma once
                            //דברים שצריך לעשות
#include <vector>
#include <string>
#include <iostream>
#include <string_view>

enum class FixQuality : int {
    Invalid = 0,
    GPS_Fix = 1,
    DGPS_Fix = 2,
    PPS_Fix = 3
};

enum class LockType : int {
    NoFix = 1,
    Fix2D = 2,
    Fix3D = 3
};

enum class Status : char {
    Active ='A',
    Void = 'V'
};

struct DateTimeData {
    std::string time;             
    std::string date;              
};

struct QualityData {
    bool statusActive;            
    int lockQuality;             
    int lockType;               
    double signalStrength;  
    int satellitesInView;       
    double PDOP;               
    double HDOP;                 
    double VDOP;                 
    bool selectionMode;  
    double confidenceLevel;     
};
struct BasicLocationData {
    double latitude;               // קו רוחב (GGA)
    double longitude;              // קו אורך (GGA)
    char northOrSouth;             // צפון או דרום (GGA)
    char eastOrWest;               // מזרח או מערב (GGA)
    double altitudeMSL;            // גובה מעל פני הים (GGA)
    std::string altitudeUnit;      // יחידת גובה (GGA)
};

// קטגוריית כיוון, תנועה וניווט
struct NavigationData {
    double heading;                // Heading זווית במעלות (HDT)
    bool isTrueNorth;              // True (T) מציין שזה ביחס לצפון האמיתי (HDT/VTG)
    double speedOverGround;        // מהירות מעל הקרקע בקשרים (RMC)
    double courseDegrees;          // מסלול מעלות כיוון (RMC)
    
    // שדות APB
    char warningStatus;            // סטטוס אזהרה (V/A)
    double XTE;                    // Cross Track Error - מרחק סטייה מהנתיב
    char directionToSteer;         // Direction to Steer (L/R)
    double bearingToWaypoint;      // Bearing to Waypoint - זווית מהכיסא אל היעד
    
    // שדות AAM
    bool arrivalStatus;            // Status (A/V) האם הגענו
    bool arrivalCircle;            // Arrival Circle - האם נכנסנו לרדיוס
    bool perpendicularPassing;     // Perpendicular Passing - האם חלפנו על פני הנקודה
    std::string waypointID;        // Waypoint ID - שם/מספר הנקודה
};

class GPSLocation {
public:
    DateTimeData dateTime;
    QualityData quality;
    BasicLocationData position;
    NavigationData navigation;

    static constexpr size_t MAX_SENTENCE_SIZE = 128;



GPSLocation() {
    buffer.reserve(MAX_SENTENCE_SIZE);

    dateTime.time = "";
    dateTime.date = "";

    quality.statusActive = false;
    quality.lockQuality = 0;
    quality.lockType = static_cast<int>(LockType::NoFix);
    quality.satellitesInUse = 0;
    quality.satellitesInView = 0;
    quality.signalStrength = 0.0;
    quality.PDOP = 0.0;
    quality.HDOP = 1.0; 
    quality.VDOP = 0.0;
    quality.selectionMode = false;
    quality.confidenceLevel = 0.0;

    position.latitude = 0.0;
    position.longitude = 0.0;
    position.altitudeMSL = 0.0;
    position.northOrSouth = 'N';
    position.eastOrWest = 'E';
    position.altitudeUnit = "M";

    navigation.heading = 0.0;
    navigation.isTrueNorth = false;
    navigation.speedOverGround = 0.0;
    navigation.courseDegrees = 0.0;
    
    navigation.warningStatus = 'V'; 
    navigation.XTE = 0.0;
    navigation.directionToSteer = ' ';
    navigation.bearingToWaypoint = 0.0;
    
    navigation.arrivalStatus = false;
    navigation.arrivalCircle = false;
    navigation.perpendicularPassing = false;
    navigation.waypointID = "";
}
    void calculateConfidence();
    void updateFromStream(std::istream& dataStream);
    void fillGPSLocation(const std::string& nmeaSentence);
    void displaySummary() const;
    double toDouble(const std::string_view& s); 
    int toInt(const std::string_view& s, int defaultValue = 0);
private:
    std::string buffer;
    std::vector<std::string> tokens;
    void parseGGA(const std::vector<std::string>& f);
    void parseRMC(const std::vector<std::string>& f);
    void parseGSV(const std::vector<std::string>& f);
    void parseGSA(const std::vector<std::string>& f);
    void parseHDT(const std::vector<std::string>& f);
    void parseAPB(const std::vector<std::string>& f);
    void parseAAM(const std::vector<std::string>& f);
    void parseVTG(const std::vector<std::string>& f);

    void updateIfNotEmpty(double& target, const std::string& field);
    void split(const std::string_view& s, char delimiter);
    bool validateChecksum(const std::string_view& sentence); 
};

