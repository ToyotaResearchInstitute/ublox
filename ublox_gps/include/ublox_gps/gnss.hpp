#ifndef UBLOX_GPS_GNSS_HPP
#define UBLOX_GPS_GNSS_HPP

#include <set>
#include <string>

namespace ublox_gps {

class GNSS final {
public:
  GNSS();
  ~GNSS();

  void add(const std::string & gnss);
  bool isSupported(const std::string & gnss);

private:
  std::set<std::string> supported_;
};

}

#endif
