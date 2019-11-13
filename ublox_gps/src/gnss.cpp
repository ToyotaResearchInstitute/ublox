#include <ublox_gps/gnss.hpp>

namespace ublox_gps {

GNSS::GNSS()
{
}

GNSS::~GNSS()
{
}

void GNSS::add(const std::string & gnss)
{
  supported_.insert(gnss);
}

bool GNSS::isSupported(const std::string & gnss)
{
  return supported_.count(gnss) > 0;
}
}
