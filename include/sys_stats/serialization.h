#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <ros/serialization.h>
#include <ros/message_traits.h>
#include <sys_stats/sys_stats.h>

#define TOPIC_TRAITS_FROM_MESSAGE(serializableClass, msg) \
  ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(                      \
      serializableClass, MD5Sum<msg>::value(), DataType<msg>::value(), Definition<msg>::value())

// Associate library classes with ROS messages
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::Cpu, shield_msgs::Cpu)
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::Net, shield_msgs::Net)
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::Disk, shield_msgs::Disk)
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::Wifi, shield_msgs::Wifi)
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::Thread, shield_msgs::Thread)
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::Process, shield_msgs::Process)
TOPIC_TRAITS_FROM_MESSAGE(sys_stats::SysStats, shield_msgs::SysStats)

// Define how to read & write each message
namespace ros
{
namespace serialization
{
template <>
struct Serializer<sys_stats::Cpu>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.cpu_frequency);
    stream.next(m.cpu_temperature);
    stream.next(m.cpu_usage);
  }
};

template <>
struct Serializer<sys_stats::Net>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.interface);
    stream.next(m.read);
    stream.next(m.write);
  }
};

template <>
struct Serializer<sys_stats::Process>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.pid);
    stream.next(m.cpu_use);
    stream.next(m.mem_use);
    stream.next(m.process_name);
    stream.next(m.args);
    stream.next(m.threads);
  }
};

template <>
struct Serializer<sys_stats::Thread>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.tid);
    stream.next(m.cpu_use);
    stream.next(m.thread_name);
  }
};

template <>
struct Serializer<sys_stats::Disk>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.disk_name);
    stream.next(m.read);
    stream.next(m.write);
  }
};

template <>
struct Serializer<sys_stats::Wifi>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.ssid);
    stream.next(m.frequency);
    stream.next(m.bitrate);
    stream.next(m.link_quality);
    stream.next(m.signal_level);
  }
};

template <>
struct Serializer<sys_stats::SysStats>
{
  ROS_DECLARE_ALLINONE_SERIALIZER template <typename Stream, typename T>
  inline static void allInOne(Stream& stream, T m)
  {
    // must match order in message definition
    stream.next(m.cpu_use_total);
    stream.next(m.mem_use_total);
    stream.next(m.swap_use_total);
    stream.next(m.processes);
    stream.next(m.cpu);
    stream.next(m.interface);
    stream.next(m.disks);
    stream.next(m.wifi);
  }
};
}
}

#endif  // SERIALIZATION_H
