#include <iostream>
#include <string>

template <typename T>

struct GpsList: public SensorList<GpsList> {
  std::shared_ptr<std::GpsObject> Gps_list;
  };
