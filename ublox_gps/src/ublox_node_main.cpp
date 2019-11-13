int main(int argc, char** argv) {
  ros::init(argc, argv, "ublox_gps");
  nh.reset(new ros::NodeHandle("~"));
  nh->param("debug", ublox_gps::debug, 1);
  if(ublox_gps::debug) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug))
     ros::console::notifyLoggerLevelsChanged();

  }
  UbloxNode node;
  return 0;
}
