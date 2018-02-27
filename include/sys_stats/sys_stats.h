#ifndef _PROCLIB_H
#define _PROCLIB_H

#include <string>
#include <vector>
#include <chrono>

namespace sys_stats
{
struct seconds
{
  using Components = std::pair<unsigned long long, unsigned long long>;

  Components components;

  friend seconds operator-(const seconds& s1, const seconds& s2);
};

struct uptime
{
  seconds uptime;
  seconds idle;
};

struct Thread
{
  int tid;
  std::string thread_name;
  float cpu_use;
  Thread();

 private:
  friend struct SysStats;

  /// static interface (to reuse the same code for threads/processes)
  // cppcheck-suppress unusedPrivateFunction
  void reset(std::string name);
  // cppcheck-suppress unusedPrivateFunction
  void setCmdLine(std::string name);
  // cppcheck-suppress unusedPrivateFunction
  const std::string& name() const;
  // cppcheck-suppress unusedPrivateFunction
  int id();
  long rss;

  struct uptime
  {
    unsigned long utime;
    unsigned long stime;
  };

  uptime previous_uptime;
  uptime uptime;

  // to mark at the query - nonmarked entries will be removed
  // (dead processes)
  bool mark;

  template <typename P>
  friend bool parseProcStat(P& p, std::string proc_file, int pid, bool read_cmdline, float uptime_diff);
  template <typename P>
  friend void calculateCpu(P& p, float ticks_elapsed);
};

struct Process
{
  pid_t pid;
  std::string process_name;

  float cpu_use;
  float mem_use;

  std::vector<Thread> threads;
  std::string args;
  Process();

 private:
  friend struct SysStats;

  // cppcheck-suppress unusedPrivateFunction
  void reset(std::string name);
  // cppcheck-suppress unusedPrivateFunction
  void setCmdLine(std::string cmdline);
  // cppcheck-suppress unusedPrivateFunction
  const std::string& name() const;
  // cppcheck-suppress unusedPrivateFunction
  int id();
  long rss;

  struct uptime
  {
    unsigned long utime;
    unsigned long stime;
  };

  uptime previous_uptime;
  uptime uptime;

  // to mark at the query - nonmarked entries will be removed
  // (dead processes)
  bool mark;

  template <typename P>
  friend bool parseProcStat(P& p, std::string proc_file, int pid, bool read_cmdline, float uptime_diff);
  template <typename P>
  friend void calculateCpu(P& p, float ticks_elapsed);
};

struct Cpu
{
  float cpu_frequency;
  float cpu_temperature;
  float cpu_usage;  // per core cpu usage
};

struct Net
{
  Net()
  {
  }

  std::string interface;
  float read;
  float write;

 private:
  friend struct SysStats;
  unsigned long prev_read;
  unsigned long prev_write;
  bool mark;
};

struct Disk
{
  Disk()
  {
  }

  std::string disk_name;
  float read;                // Bytes per second
  float write;               // Bytes per second
  unsigned long free_space;  // Bytes

 private:
  friend struct SysStats;
  unsigned long prev_read;
  unsigned long prev_write;
  std::string mount_point;
  bool mark;
};

struct Wifi
{
  std::string interface;
  std::string ssid;
  float frequency;     // Hz
  float bitrate;       // b/s
  float link_quality;  // %
  float signal_level;  // dBm

  explicit Wifi(int driver_socket);

 private:
  friend struct SysStats;
  // cppcheck-suppress unusedPrivateFunction
  bool queryDriver();
  int driver_socket;
  float qual;    // from /proc/net/wireless
  int max_qual;  // from the driver
  bool mark;
};

struct SysStats
{
  std::vector<Process> processes;
  std::vector<Cpu> cpu;
  std::vector<Net> interface;
  std::vector<Disk> disks;
  std::vector<Wifi> wifi;
  float cpu_use_total;
  float mem_use_total;
  float swap_use_total;

  SysStats();
  ~SysStats();

  bool update();

 private:
  bool get_processes();
  void get_process(long int pid);

  void getuptime();
  bool getMemory();
  bool getCpuInfo();
  std::vector<float> getPerCoreUsage();
  float getCpuTemperature(int processor_id);
  bool getInterfaceData();
  bool getDiskData();
  bool getWifiData();

  uptime previous_uptime;
  uptime current_uptime;
  float uptime_diff;
  unsigned long long total_memory;
  unsigned long long free_memory;
  unsigned long long buffers;
  unsigned long long cached;
  unsigned long long shmem;
  unsigned long long sreclaimable;
  unsigned long long total_swap;
  unsigned long long free_swap;
  std::string coretemp_path;

  decltype(std::chrono::steady_clock::now()) prev_iftime;
  decltype(std::chrono::steady_clock::now()) prev_disktime;

  static void get_threads_for_process(Process& p, float uptime_diff);
  static void get_thread(Process& p, long int tid, float uptime_diff);

  std::vector<unsigned long> previous_cpu_total;
  std::vector<unsigned long> previous_cpu_idle;
  // Socket for querying the wifi driver
  int driver_socket;
};
}  // namespace sys_stats

#endif
