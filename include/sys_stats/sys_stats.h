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

struct SysStats
{
  std::vector<Process> processes;
  std::vector<Cpu> cpu;
  std::vector<Net> interface;
  std::vector<Disk> disks;
  float cpu_use_total;
  float mem_use_total;
  float swap_use_total;

  SysStats();

 private:
  friend bool get_sys_stats(SysStats*);

  // cppcheck-suppress unusedPrivateFunction
  bool get_processes();
  // cppcheck-suppress unusedPrivateFunction
  void get_process(long int pid);

  // cppcheck-suppress unusedPrivateFunction
  void getuptime();
  // cppcheck-suppress unusedPrivateFunction
  bool getMemory();
  // cppcheck-suppress unusedPrivateFunction
  bool getCpuInfo();
  std::vector<float> getPerCoreUsage();
  float getCpuTemperature(int processor_id);
  // cppcheck-suppress unusedPrivateFunction
  bool getInterfaceData();
  // cppcheck-suppress unusedPrivateFunction
  bool getDiskData();

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
};

// Upon return, `stats` will be populated, or function will return
// false if error. All stats, especially the per-second statistics,
// will be averages since this function was last called. It is acceptable
// for the first call to this function to return false.
bool get_sys_stats(SysStats* stats);
}

#endif
