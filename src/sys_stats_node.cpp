#include <ros/ros.h>
#include <shield_msgs/SysStats.h>
#include <sys_stats/sys_stats.h>
#include <sys_stats/serialization.h>

class SysStatsWrapper
{
 public:
  SysStatsWrapper()
  {
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    pub_ = nh.advertise<shield_msgs::SysStats>("sys_stats", 0);
    timer_ = nh.createTimer(ros::Rate(nhp.param("rate", 1.0)), &SysStatsWrapper::getStats, this);
  }

  void getStats(const ros::TimerEvent&)
  {
    if (stats_.update())
    {
      pub_.publish(stats_);
    }
    else
    {
      ROS_WARN("Could not get system statistics");
    }
  }

  sys_stats::SysStats stats_;
  ros::Timer timer_;
  ros::Publisher pub_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  SysStatsWrapper stats_wrapper;

  ros::spin();

  return 0;
}
