#include <iostream>
#include <string>
#include <sstream>  // תוקן: נוסף עבור std::stringstream
#include "Gps/GpsProcessing.hpp" 
#include "Gps/GpsObject.hpp"

#include <iostream>
#include <string>
#include <sstream>  // תוקן: נוסף עבור std::stringstream
#include "Gps/GpsProcessing.hpp" 
#include "Gps/GpsObject.hpp"

int main() {
    std::cout << "--- Starting GPS Stream Processing Test ---\n" << std::endl;

    // 1. יצירת האובייקט האחראי על עיבוד ה-GPS
    GpsProcessing gpsProcessor;

    // 2. יצירת ה-Stream והזרמת כל המשפטים שממלאים את מבני הנתונים
    std::stringstream gpsStream;
    gpsStream << "$GPGGA,123456.00,3201.2345,N,03445.6789,E,1,08,0.9,105.2,M,12.3,M,,*64\n";
    gpsStream << "$GPRMC,123456.00,A,3201.2345,N,03445.6789,E,22.4,180.5,180526,,,A*7F\n";
    gpsStream << "$GPGSA,A,3,01,02,03,04,05,06,07,08,,,,1.5,0.9,1.2*36\n";
    gpsStream << "$GPGSV,3,1,11,01,40,090,40,02,20,180,35,03,15,225,30,04,75,300,42*7B\n";
    gpsStream << "$GPHDT,180.5,T*37\n";
    gpsStream << "$GPVTG,180.5,T,,M,22.4,N,41.5,K,A*31\n";
    gpsStream << "$GPAPB,A,A,0.02,L,N,V,V,179.5,T,WP001,178.0,T,178.0,T*23\n";
    gpsStream << "$GPAAM,A,A,0.1,N,WP001*3F\n";
    GpsObject finalObject;

   finalObject=gpsProcessor.process(gpsStream);
   
        // 4. הדפסת הנתונים הסופיים והמשולבים שנאספו מכל המשפטים יחד לעיון
        std::cout << "\n------------------------------------" << std::endl;
        std::cout << "Successfully generated combined GpsObject:" << std::endl;
        std::cout << "  Timestamp:    " << finalObject.timestamp << std::endl;
        std::cout << "  Local X (Lat):" << finalObject.x_local << std::endl;
        std::cout << "  Local Y (Lon):" << finalObject.y_local << std::endl;
        std::cout << "  Local Z (Alt):" << finalObject.z_local << std::endl;
        std::cout << "  Speed:        " << finalObject.speed << std::endl;
        std::cout << "  Heading:      " << finalObject.heading << std::endl;
        std::cout << "  Confidence:   " << finalObject.confidence << std::endl;
        std::cout << "  Is Valid:     " << (finalObject.isValid ? "True" : "False") << std::endl;
        std::cout << "------------------------------------" << std::endl;


}