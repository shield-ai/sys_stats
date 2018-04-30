#include <cstdio>
#include <cstdlib>
#include <dirent.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <cassert>
#include <fstream>
#include <chrono>
#include <sstream>
#include <cmath>
#include <limits>
#include <sys/ioctl.h>
#include <sys/statvfs.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include <nvml.h>
#include "sys_stats/sys_stats.h"

namespace sys_stats
{
/// auxilary functions
/// reads the /proc/uptime and returns the struct
uptime readuptime()
{
  uptime uptime;
  FILE* fp = fopen("/proc/uptime", "r");
  if (fp == NULL)
    return {};
  int res = fscanf(fp,
                   "%lld.%lld %lld.%lld",
                   &std::get<0>(uptime.uptime.components),
                   &std::get<1>(uptime.uptime.components),
                   &std::get<0>(uptime.idle.components),
                   &std::get<1>(uptime.idle.components));
  assert(res == 4);
  fclose(fp);
  return uptime;
}

/// Converts the value with the SI prefix to the base value
static unsigned long long prefixedUnitToValue(unsigned long long value, char unit)
{
  // clang-format off
  return value * (unit == 'k' ? 1024 :
    unit == 'M' ? 1024UL * 1024 :
    unit == 'G' ? 1024UL * 1024 * 1024 :
    unit == 'T' ? 1024UL * 1024 * 1024 * 1024 : 1);
  // clang-format on
}

/// gets the hwmon path of the coretemp sensor
std::string get_hwmon_path()
{
  std::string directory;
  struct dirent* dp;
  DIR* dirp = opendir("/sys/class/hwmon");
  if (dirp == NULL)
    return "";

  while (1)
  {
    dp = readdir(dirp);

    if (dp == NULL)
      break;

    std::string name = dp->d_name;
    if (name != "." && name != "..")
    {
      std::string path = "/sys/class/hwmon/" + name;
      std::ifstream namefile{path + "/name"};
      if (namefile.is_open() == false)
        continue;

      std::string desc;
      getline(namefile, desc);
      if (desc == "coretemp")
      {
        directory = path;
        break;
      }
    }
  }

  if (closedir(dirp) == -1)
    return "";

  return directory;
}

/// Returns the page size (as RSS is reported in pages)
long inline pageSize()
{
  static long page_size = 0;
  if (page_size == 0)
  {
    page_size = sysconf(_SC_PAGESIZE);
  }
  return page_size;
}

/// Returns the ticks per second
long ticksPerSecond()
{
  static long ticks_per_sec = 0;
  if (ticks_per_sec == 0)
  {
    ticks_per_sec = sysconf(_SC_CLK_TCK);
  }
  return ticks_per_sec;
}

/// Converts the secs (sec.cents) into ticks
float inline secondsToTicks(const seconds& secs)
{
  return std::get<0>(secs.components) * ticksPerSecond() + std::get<1>(secs.components) * ticksPerSecond() / 100.;
}

/// Returns the free space for the mount point
long inline freeSpaceForMount(const std::string& mount_point)
{
  struct statvfs stat;

  if (statvfs(mount_point.c_str(), &stat) != 0)
  {
    return 0;
  }

  return stat.f_bsize * stat.f_bavail;
}

/// Trims the string
template <typename S>
static inline S&& ltrim(S&& s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](char c) { return !std::isspace(c); }));
  return std::forward<S>(s);
}

/// Converts the part of the string to a number
unsigned long substr_to_ul(const std::string& str, int begin, int end)
{
  unsigned long val = 0;
  int i = 1;

  while (--end >= begin)
  {
    val += i * (str[end] - '0');
    i *= 10;
  }

  return val;
}

/// Calculates the CPU percentage for the process/thread
template <typename P>
void calculateCpu(P& p, float ticks_elapsed)
{
  auto prev_tot = p.previous_uptime.utime + p.previous_uptime.stime;
  auto curr_tot = p.uptime.utime + p.uptime.stime;

  p.cpu_use = (curr_tot <= prev_tot) ? 0 : (float(curr_tot - prev_tot) / ticks_elapsed * 100);
}

seconds operator-(const seconds& s1, const seconds& s2)
{
  auto cents = std::get<0>(s1.components) * 100 + std::get<1>(s1.components) - std::get<0>(s2.components) * 100 -
               std::get<1>(s2.components);

  return {{cents / 100, cents % 100}};
}

/// Parses the procStat file
template <typename P>
bool parseProcStat(P& p, std::string proc_file, int pid, bool read_cmdline, float uptime_diff)
{
  static std::stringstream ss;
  static std::string stats_line;
  static std::string cmd_line;
  static std::string process_name;
  bool calculate_cpu = true;  // don't calculate for the first seen processes

  std::ifstream stats{proc_file};
  if (stats.is_open() == false)
    return false;

  getline(stats, stats_line);
  stats.close();

  if (stats_line.length() < 50)
    return false;

  // find the program name
  auto begin = stats_line.find('(');
  begin++;

  // begin now points to (, let's find the closing one
  auto end = stats_line.rfind(')');
  process_name = stats_line.substr(begin, end - begin);
  begin = end + 1;

  // double check if we're reusing pid
  if (process_name != p.name())
  {
    p.previous_uptime = {0, 0};
    p.reset(process_name);
    calculate_cpu = false;

    if (read_cmdline)
    {
      ss.str("");
      ss << "/proc/" << pid << "/cmdline";

      std::ifstream file{ss.str()};
      // ignore on errors
      if (file.is_open() == false)
        return false;

      getline(file, cmd_line);
      file.close();

      std::replace(std::begin(cmd_line), std::end(cmd_line), '\0', ' ');

      if (cmd_line.length() > 0)
      {
        cmd_line.resize(cmd_line.length() - 1);
      }

      p.setCmdLine(cmd_line);
    }
  }

  /*
      (3) state  %c
      (14) utime  %lu
      (15) stime  %lu
      (24) rss  %ld
  */
  // skip
  for (int i = 3; i < 14; i++)
  {
    begin = stats_line.find(' ', begin + 1);
  }
  end = stats_line.find(' ', begin + 1);
  p.uptime.utime = substr_to_ul(stats_line, begin + 1, end);
  begin = end;
  end = stats_line.find(' ', begin + 1);
  p.uptime.stime = substr_to_ul(stats_line, begin + 1, end);
  begin = end;

  for (int i = 16; i < 24; i++)
  {
    begin = stats_line.find(' ', begin + 1);
  }
  end = stats_line.find(' ', begin + 1);
  p.rss = substr_to_ul(stats_line, begin + 1, end);

  if (calculate_cpu)
  {
    calculateCpu(p, uptime_diff);
  }

  return true;
}

Thread::Thread() : cpu_use{0}, previous_uptime{0, 0}, uptime{0, 0}, mark{false}
{
}

void Thread::reset(std::string name)
{
  previous_uptime = {0, 0};
  uptime = {0, 0};
  cpu_use = {0};
  thread_name = std::move(name);
}

// just satisfy the static interface,
// no cmdline for threads
void Thread::setCmdLine(std::string)
{
}

const std::string& Thread::name() const
{
  return thread_name;
}

int Thread::id()
{
  return tid;
}

Process::Process() : cpu_use{0}, mem_use{0}, previous_uptime{0, 0}, uptime{0, 0}, mark{false}
{
}

void Process::reset(std::string name)
{
  previous_uptime = {0, 0};
  uptime = {0, 0};
  cpu_use = {0};
  mem_use = {0};
  process_name = std::move(name);
}

void Process::setCmdLine(std::string cmdline)
{
  args = std::move(cmdline);
}

const std::string& Process::name() const
{
  return process_name;
}

int Process::id()
{
  return pid;
}

SysStats::SysStats()
  : mem_use_total(0)
  , swap_use_total(0)
  , previous_uptime(readuptime())
  , coretemp_path(get_hwmon_path())
  , gpu_collector(gpu_stats)
{
  this->driver_socket = socket(AF_INET, SOCK_DGRAM, 0);
}

SysStats::~SysStats()
{
  if (this->driver_socket != -1)
    close(this->driver_socket);
}

/// Queries everything and fills the structure
bool get_sys_stats(SysStats* stats)
{
  if (!stats->getCpuInfo())
    return false;
  if (!stats->getInterfaceData())
    return false;

  if (!stats->getDiskData())
    return false;

  stats->getMemory();
  stats->getuptime();
  if (!stats->get_processes())
    return false;

  if (!stats->getGpuInfo())
  {
    std::cout << "Failed to get GPU stats" << std::endl;
    return false;
  }

  std::cout << "GPU stats:\n"
  << "Total memory: " << stats->gpu_stats[0].total_mem << "\n"
  << "Total load: " << stats->gpu_stats[0].total_load << "\n"
  << "Power: " << stats->gpu_stats[0].power << "\n"
  << "Temperature: " << stats->gpu_stats[0].temperature << "\n"
  << "Clock: " << stats->gpu_stats[0].clock << std::endl;

  std::cout << "Per Process:" << stats->gpu_stats[0].process_list.size() << std::endl;
  for (uint32_t i = 0; i < stats->gpu_stats[0].process_list.size(); i++)
  {
    std::cout << "Pid: " << stats->gpu_stats[0].process_list[i].pid << "\n"
    << "Mem: " << stats->gpu_stats[0].process_list[i].memory << std::endl;
  }

  // Querying the wifi driver seems have a chance to panic the kernel and cause a hard lock up
  //if (!stats->getWifiData())
  //  return false;

  return true;
}

/// Query processes
bool SysStats::get_processes()
{
  // clear all marks
  for (auto& p : processes)
    p.mark = false;

  struct dirent* dp;
  DIR* dirp = opendir("/proc");
  if (dirp == NULL)
    return false;

  while (1)
  {
    dp = readdir(dirp);

    if (dp == NULL)
      break;

    char* endptr;
    long int pid = strtol(dp->d_name, &endptr, 10);

    if (endptr == NULL || *endptr == '\0')
    {
      get_process(pid);
    }
  }

  // remove nonmarked processes
  processes.erase(
      std::remove_if(std::begin(processes), std::end(processes), [](const Process& p) { return p.mark == false; }),
      std::end(processes));

  std::sort(std::begin(processes), std::end(processes), [](const Process& p1, const Process& p2) {
    return p1.cpu_use > p2.cpu_use;
  });
  if (closedir(dirp) == -1)
    return false;

  return true;
}

/// Fills the information for the single process
void SysStats::get_process(long int pid)
{
  static std::stringstream ss;

  // get or create the existing process
  auto pit = std::find_if(std::begin(processes), std::end(processes), [pid](const Process& p) { return p.pid == pid; });

  if (pit == std::end(processes))
  {
    Process p;
    p.pid = pid;
    pit = processes.insert(std::end(processes), p);
  }

  auto& p = *pit;
  p.previous_uptime = p.uptime;

  // read the stats
  ss.str("");
  ss << "/proc/" << pid << "/stat";

  auto parse_results = parseProcStat<Process>(p, ss.str(), pid, true, this->uptime_diff);
  if (parse_results == false)
    return;

  p.mark = true;
  get_threads_for_process(p, this->uptime_diff);

  p.rss *= pageSize();
  p.mem_use = (static_cast<float>(p.rss) / this->total_memory) * 100.;
}

/// Gets all threads for the given process
void SysStats::get_threads_for_process(Process& p, float uptime_diff)
{
  static std::stringstream ss;
  //
  // clear all marks
  for (auto& t : p.threads)
    t.mark = false;

  ss.str("");
  ss << "/proc/" << p.id() << "/task";

  struct dirent* dp;
  DIR* dirp = opendir(ss.str().c_str());
  if (dirp == NULL)
    return;

  while (1)
  {
    dp = readdir(dirp);

    if (dp == NULL)
      break;

    char* endptr;
    long int tid = strtol(dp->d_name, &endptr, 10);

    if (endptr == NULL || *endptr == '\0')
    {
      if (tid != p.pid)  // thread
      {
        get_thread(p, tid, uptime_diff);
      }
    }
  }

  // remove nonmarked threads
  p.threads.erase(
      std::remove_if(std::begin(p.threads), std::end(p.threads), [](const Thread& t) { return t.mark == false; }),
      std::end(p.threads));

  if (closedir(dirp) == -1)
    return;
}

/// Queries a single tread
void SysStats::get_thread(Process& p, long int tid, float uptime_diff)
{
  static std::stringstream ss;

  // get or create the existing process
  auto tit = std::find_if(std::begin(p.threads), std::end(p.threads), [tid](const Thread& t) { return t.tid == tid; });

  if (tit == std::end(p.threads))
  {
    Thread t;
    t.tid = tid;
    tit = p.threads.insert(std::end(p.threads), t);
  }

  auto& t = *tit;
  t.previous_uptime = t.uptime;

  // read the stats
  ss.str("");
  ss << "/proc/" << p.pid << "/task/" << tid << "/stat";

  auto parse_result = parseProcStat<Thread>(t, ss.str(), tid, true, uptime_diff);
  if (parse_result == false)
    return;

  t.mark = true;
  calculateCpu(t, uptime_diff);
}

/// Gets the uptime
void SysStats::getuptime()
{
  this->previous_uptime = this->current_uptime;
  this->current_uptime = readuptime();

  this->uptime_diff = secondsToTicks(this->current_uptime.uptime - this->previous_uptime.uptime);
}

/// Read total memory
bool SysStats::getMemory()
{
  char field[512];
  char unit[32];
  this->total_memory = 0;

  FILE* fp = fopen("/proc/meminfo", "r");
  if (fp == nullptr)
    return false;

  while (!feof(fp))
  {
    unsigned long value;
    if (fscanf(fp, "%511s %lu %31s", field, &value, unit) != 3)
      continue;

    if (strcmp(field, "MemTotal:") == 0)
    {
      this->total_memory = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "MemFree:") == 0)
    {
      this->free_memory = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "Buffers:") == 0)
    {
      this->buffers = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "Cached:") == 0)
    {
      this->cached = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "Shmem:") == 0)
    {
      this->shmem = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "SReclaimable:") == 0)
    {
      this->sreclaimable = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "SwapTotal:") == 0)
    {
      this->total_swap = prefixedUnitToValue(value, unit[0]);
    }
    else if (strcmp(field, "SwapFree:") == 0)
    {
      this->free_swap = prefixedUnitToValue(value, unit[0]);
    }
  }

  fclose(fp);

  if (this->total_memory != 0)
  {
    auto used_mem = this->total_memory - this->free_memory;
    used_mem -= (this->buffers + this->cached + this->sreclaimable - this->shmem);
    this->mem_use_total = (float(used_mem) / this->total_memory) * 100.0f;
    this->swap_use_total = 100 - (float(this->free_swap) / this->total_swap) * 100.0f;
  }

  return this->total_memory == 0;
}

/// Parses the CPU info and the temperatures
bool SysStats::getCpuInfo()
{
  std::ifstream file("/proc/cpuinfo");
  std::string line;
  if (!file.is_open())
    return false;

  unsigned int processor_id = -1;
  auto cpu_usage = getPerCoreUsage();
  this->cpu_use_total =
      cpu_usage.size() > 0 ? std::accumulate(std::begin(cpu_usage), std::end(cpu_usage), 0.0) / cpu_usage.size() : 0;

  while (getline(file, line))
  {
    auto colon = line.rfind(':');
    if (colon == std::string::npos)
      continue;

    auto tab = line.find('\t');
    auto cat = line.substr(0, tab);

    if (cat == "processor")
    {
      processor_id = std::stoi(line.substr(colon + 1));
    }
    else if (cat == "cpu MHz")
    {
      auto core_usage = cpu_usage.size() >= processor_id ? cpu_usage[processor_id] : 0.f;

      auto value = std::stof(line.substr(colon + 1));
      if (cpu.size() <= processor_id)
      {
        cpu.push_back({value * 1000000, 0, core_usage});
      }
      else
      {
        cpu[processor_id] = {value * 1000000, 0, core_usage};
      }

      cpu[processor_id].cpu_temperature = getCpuTemperature(processor_id);
    }
  }

  file.close();

  return processor_id != static_cast<decltype(processor_id)>(-1);
}

/// Gets the CPU temperature
float SysStats::getCpuTemperature(int processor_id)
{
  static std::stringstream ss;
  ss.str("");
  ss << this->coretemp_path << "/temp" << processor_id + 1 << "_input";
  std::ifstream temp_input(ss.str());
  if (temp_input.is_open() == false)
    return 0.0f;

  std::string temp;
  getline(temp_input, temp);
  temp_input.close();

  return std::stof(temp) / 1000.;
}

std::vector<float> SysStats::getPerCoreUsage()
{
  std::string line;
  std::ifstream file("/proc/stat");
  std::vector<unsigned long> cpu_total;
  std::vector<unsigned long> cpu_idle;

  if (file.is_open() == false)
    return {};

  while (getline(file, line))
  {
    unsigned int cpu_num;
    unsigned long user, nice, system, idle;

    if (line.length() < 4 || line.substr(0, 3) != "cpu")
      continue;

    if (line[3] == ' ')
      continue;

    if (sscanf(line.c_str(), "cpu%u %lu %lu %lu %lu", &cpu_num, &user, &nice, &system, &idle) == 5)
    {
      if (cpu_num >= cpu_total.size())
      {
        cpu_total.resize(cpu_num + 1);
        cpu_idle.resize(cpu_num + 1);
      }

      cpu_total[cpu_num] = user + nice + system + idle;
      cpu_idle[cpu_num] = idle;
    }
  }

  file.close();

  std::vector<float> usage(cpu_total.size(), 0);

  if (previous_cpu_total.size() == cpu_total.size())
  {
    for (unsigned i = 0; i < cpu_total.size(); i++)
    {
      auto d_idle = cpu_idle[i] - previous_cpu_idle[i];
      auto d_total = cpu_total[i] - previous_cpu_total[i];
      if (d_total > 0)
      {
        usage[i] = (((float)d_total - d_idle) / d_total) * 100.f;
      }
    }
  }

  previous_cpu_total = std::move(cpu_total);
  previous_cpu_idle = std::move(cpu_idle);
  return usage;
}

/// Gets the data for the interface
bool SysStats::getInterfaceData()
{
  for (auto& i : interface)
    i.mark = false;

  std::ifstream file("/proc/net/dev");
  std::string line;
  if (!file.is_open())
    return false;

  auto now = std::chrono::steady_clock::now();
  long duration = std::chrono::duration_cast<std::chrono::microseconds>(now - prev_disktime).count();
  prev_disktime = now;

  /*
   *   face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets errs drop fifo colls carrier
   compressed
       lo: 2776770   11307    0    0    0     0          0         0  2776770   11307    0    0    0     0       0 0
      */
  while (getline(file, line))
  {
    auto colon = line.find(':');
    if (colon == std::string::npos)
      continue;

    auto interface_name = ltrim(line.substr(0, colon));
    line = line.substr(colon + 1);
    unsigned long recv_bytes, sent_bytes, dummy;
    sscanf(line.c_str(),
           " %lu %lu %lu %lu %lu %lu %lu %lu %lu",
           &recv_bytes,
           &dummy,
           &dummy,
           &dummy,
           &dummy,
           &dummy,
           &dummy,
           &dummy,
           &sent_bytes);

    // find the inteface
    auto i = std::find_if(std::begin(interface), std::end(interface), [&interface_name](const Net& n) {
      return n.interface == interface_name;
    });

    if (i == std::end(interface))
    {
      i = interface.insert(std::end(interface), std::move(Net{}));
      (*i).interface = interface_name;
    }
    auto& iface = *i;
    iface.mark = true;

    if (duration > 0)
    {
      iface.read = (1e6 * (recv_bytes - iface.prev_read)) / duration;
      iface.write = (1e6 * (sent_bytes - iface.prev_write)) / duration;
    }
    iface.prev_read = recv_bytes;
    iface.prev_write = sent_bytes;
  }

  file.close();

  interface.erase(
      std::remove_if(std::begin(interface), std::end(interface), [](const Net& iface) { return iface.mark == false; }),
      std::end(interface));

  return true;
}

/// Gets the data for the disks (including the mountpoints and
/// the free space)
bool SysStats::getDiskData()
{
  constexpr int sector_size = 512;
  char devicename[64];
  std::string dev_name;

  for (auto& d : disks)
    d.mark = false;

  std::ifstream file("/proc/diskstats");
  std::string line;
  if (!file.is_open())
    return false;

  auto now = std::chrono::steady_clock::now();
  long duration = std::chrono::duration_cast<std::chrono::microseconds>(now - prev_iftime).count();
  prev_iftime = now;

  /* 3 - device name
   * 6 - secotors read
   * 10 -sectors written
   */
  while (getline(file, line))
  {
    unsigned long sectors_read, sectors_written, dummy;
    sscanf(line.c_str(),
           " %lu %lu %63s %lu %lu %lu %lu %lu %lu %lu",
           &dummy,
           &dummy,
           devicename,
           &dummy,
           &dummy,
           &sectors_read,
           &dummy,
           &dummy,
           &dummy,
           &sectors_written);
    dev_name = devicename;

    // find the inteface
    auto i = std::find_if(
        std::begin(disks), std::end(disks), [&dev_name](const Disk& d) { return d.disk_name == dev_name; });

    if (i == std::end(disks))
    {
      i = disks.insert(std::end(disks), Disk{});
      (*i).disk_name = dev_name;
    }
    auto& disk = *i;
    disk.mark = true;

    if (duration > 0)
    {
      disk.read = (1e6 * (sector_size * sectors_read - disk.prev_read)) / duration;
      disk.write = (1e6 * (sector_size * sectors_written - disk.prev_write)) / duration;
    }
    disk.prev_read = sectors_read * sector_size;
    disk.prev_write = sectors_written * sector_size;
  }

  file.close();

  // get the disk info for the device
  std::ifstream mounts{"/proc/mounts"};
  std::string device_name;
  if (mounts.is_open() == false)
    return false;

  while (getline(mounts, line))
  {
    auto begin = line.find(' ');
    device_name = line.substr(0, begin);

    // remove slash
    auto slash = device_name.rfind('/');
    if (slash == std::string::npos)
      continue;
    device_name = device_name.substr(slash + 1);

    auto i = std::find_if(
        std::begin(disks), std::end(disks), [&device_name](const Disk& d) { return d.disk_name == device_name; });

    if (i == std::end(disks))
      continue;

    auto end = line.find(' ', begin + 1);
    (*i).mount_point = line.substr(begin + 1, end - begin - 1);
  }
  mounts.close();

  // get the free space for every mount point
  for (auto& disk : disks)
  {
    if (disk.mount_point.size() == 0)
      continue;
    disk.free_space = freeSpaceForMount(disk.mount_point);
  }

  disks.erase(std::remove_if(std::begin(disks), std::end(disks), [](const Disk& disk) { return disk.mark == false; }),
              std::end(disks));

  return true;
}

bool SysStats::getWifiData()
{
  for (auto& i : wifi)
    i.mark = false;

  std::ifstream file("/proc/net/wireless");
  static std::string line;
  if (!file.is_open())
    return false;

  /*
   * Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
   *  face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
   *  wlp3s0: 0000   48.  -62.  -256        0      0      0      0     75        0
   */
  while (getline(file, line))
  {
    auto colon = line.find(':');
    if (colon == std::string::npos)
      continue;

    auto interface_name = ltrim(line.substr(0, colon));
    line = line.substr(colon + 1);
    float qual, level;
    int dummy;
    sscanf(line.c_str(), " %i %f %f", &dummy, &qual, &level);

    // find the inteface
    auto i = std::find_if(
        std::begin(wifi), std::end(wifi), [&interface_name](const Wifi& w) { return w.interface == interface_name; });

    if (i == std::end(wifi))
    {
      i = wifi.insert(std::end(wifi), std::move(Wifi{this->driver_socket}));
      (*i).interface = interface_name;
    }
    auto& iface = *i;
    iface.mark = true;

    // collect what we can
    if (iface.queryDriver())
    {
      iface.qual = qual;
      iface.signal_level = level;
      iface.link_quality = iface.qual / iface.max_qual * 100;
    }
    else
    {
      iface.qual = std::numeric_limits<float>::quiet_NaN();
      iface.signal_level = std::numeric_limits<float>::quiet_NaN();
      iface.link_quality = std::numeric_limits<float>::quiet_NaN();
    }
  }

  for (auto& w : wifi)
  {
    if (w.mark == false)
    {
      w.ssid = "";
      w.qual = std::numeric_limits<float>::quiet_NaN();
      w.signal_level = std::numeric_limits<float>::quiet_NaN();
      w.link_quality = std::numeric_limits<float>::quiet_NaN();
      w.frequency = std::numeric_limits<float>::quiet_NaN();
      w.bitrate = 0;
    }
  }

  return true;
}

bool SysStats::getGpuInfo()
{
  return this->gpu_collector.queryDevices();
}

bool Wifi::queryDriver()
{
  static iwreq driver_rq;
  this->ssid = "";

  char essid[IW_ESSID_MAX_SIZE + 1] = {0};
  strncpy(driver_rq.ifr_name, this->interface.c_str(), this->interface.size());
  driver_rq.u.essid.pointer = (caddr_t)essid;
  driver_rq.u.essid.length = IW_ESSID_MAX_SIZE + 1;
  driver_rq.u.essid.flags = 0;

  if (ioctl(this->driver_socket, SIOCGIWESSID, &driver_rq) != -1)
  {
    // copies to the member
    this->ssid = essid;
  }
  else
  {
    return false;
  }

  if (ssid.size() == 0)
  {
    return false;
  }

  // this doesn't change per device, let's just load it once
  if (this->max_qual == 0)
  {
    // read the range info, needed for the max quality
    char range_buffer[sizeof(iw_range) * 2];
    driver_rq.u.data.pointer = (caddr_t)range_buffer;
    driver_rq.u.data.length = sizeof(range_buffer);
    driver_rq.u.data.flags = 0;

    if (ioctl(this->driver_socket, SIOCGIWRANGE, &driver_rq) != -1)
    {
      iw_range range;
      memcpy(&range, range_buffer, sizeof(iw_range));

      if (!(range.avg_qual.updated & IW_QUAL_QUAL_INVALID))
      {
        this->max_qual = (int)range.max_qual.qual;
      }
    }
  }

  // get bitrate and frequency
  if (ioctl(this->driver_socket, SIOCGIWRATE, &driver_rq) != -1)
  {
    this->bitrate = driver_rq.u.bitrate.value;
  }

  if (ioctl(this->driver_socket, SIOCGIWFREQ, &driver_rq) != -1)
  {
    this->frequency = (float)driver_rq.u.freq.m * pow(10, driver_rq.u.freq.e);
  }

  return true;
}

Wifi::Wifi(int driver_sock) : driver_socket(driver_sock), max_qual(0)
{
}

GpuQuery::GpuQuery(std::vector<Gpu>& gpu_stats) : gpu_stats{gpu_stats}
{
  this->initialized = nvmlInit() == NVML_SUCCESS;

  if (!this->initialized)
    return;

  unsigned int device_count;
  this->initialized = nvmlDeviceGetCount(&device_count) == NVML_SUCCESS;

  if (!this->initialized)
  {
    nvmlShutdown();
    return;
  }

  for (unsigned int i = 0; i < device_count; i++)
  {
    nvmlDevice_t device;
    if (nvmlDeviceGetHandleByIndex(i, &device) != NVML_SUCCESS)
    {
      this->initialized = false;
      nvmlShutdown();
      break;
    }

    this->devices.push_back(device);
  }

  this->gpu_stats.resize(this->devices.size());
}

GpuQuery::~GpuQuery()
{
  // although documentation indicates that for the
  // backwards compatibility we're can call nvmlShutdown
  // when nvmlInit failed, there's a little point anyway
  if (this->initialized)
    static_cast<void>(nvmlShutdown());
}

bool GpuQuery::queryDevices()
{
  int i = 0;
  for (auto it = std::begin(this->devices); it != std::end(this->devices); i++, it++)
  {
    // gather as much as you can, do not fail, just continue
    static_cast<void>(this->getProcessesForDevice(*it, this->gpu_stats[i].process_list));

    static_cast<void>(this->getDeviceStats(*it, this->gpu_stats[i]));
  }

  return true;
}

bool GpuQuery::getDeviceStats(nvmlDevice_t device, Gpu& stats)
{
  stats.power = 0;
  unsigned int power_mw;
  auto ret = nvmlDeviceGetPowerUsage(device, &power_mw);

  if (ret == NVML_SUCCESS)
  {
    // DeviceGetPowerUsage returns power in mwatts
    stats.power = power_mw / 1000.0;
  }

  nvmlMemory_t memory;
  stats.total_mem = 0;

  ret = nvmlDeviceGetMemoryInfo(device, &memory);
  if (ret == NVML_SUCCESS)
  {
    stats.total_mem = (static_cast<float>(memory.used) / memory.total) * 100.0f;
  }

  unsigned int temperature;
  stats.temperature = 0;
  ret = nvmlDeviceGetTemperature(device, NVML_TEMPERATURE_GPU, &temperature);
  if (ret == NVML_SUCCESS)
  {
    stats.temperature = temperature;
  }

  ret = nvmlDeviceGetClock(device, NVML_CLOCK_GRAPHICS, NVML_CLOCK_ID_CURRENT, &stats.clock);
  if (ret != NVML_SUCCESS)
  {
    stats.clock = 0;
  }

  stats.total_load = 0;
  nvmlUtilization_t utilization;
  ret = nvmlDeviceGetUtilizationRates(device, &utilization);

  if (ret == NVML_SUCCESS)
  {
    stats.total_load = utilization.gpu;
  }

  return true;
}

bool GpuQuery::getProcessesForDevice(nvmlDevice_t device, std::vector<GpuProcess>& process_list)
{
  if (!this->initialized)
    return true;  // ignore if the nvml failed to initialize

  // get the number of the processes running
  auto collectMethod = [this, device, &process_list](decltype(nvmlDeviceGetComputeRunningProcesses) func) {
    unsigned int numProcesses;
    auto ret = func(device, &numProcesses, nullptr);

    if (ret != NVML_ERROR_INSUFFICIENT_SIZE && ret != NVML_SUCCESS)
    {
      // this shouldn't happen
      return false;
    }

    // always allocate more in case new processes are spawned
    do
    {
      if (numProcesses * 2 > this->process_infos.capacity())
      {
        this->process_infos.reserve(numProcesses * 2);
      }

      this->process_infos.resize(numProcesses * 2);

      numProcesses = this->process_infos.size();
      ret = func(device, &numProcesses, &this->process_infos[0]);
    } while (ret == NVML_ERROR_INSUFFICIENT_SIZE);  // resize and retry on error

    if (ret != NVML_SUCCESS)
      return false;

    // transform them into our list
    // TODO: stl tranform here
    for (const auto& info : this->process_infos)
      process_list.push_back(GpuProcess{info.pid, info.usedGpuMemory});

    return true;
  };

  process_list.clear();

  if (!collectMethod(nvmlDeviceGetComputeRunningProcesses))
    return false;

  if (!collectMethod(nvmlDeviceGetGraphicsRunningProcesses))
    return false;

  // Remove duplicates
  std::sort(std::begin(process_list), std::end(process_list), [](const GpuProcess& p1, const GpuProcess& p2) {
    return p1.pid < p2.pid;
  });

  process_list.erase(std::unique(std::begin(process_list),
                                 std::end(process_list),
                                 [](const GpuProcess& p1, const GpuProcess& p2) { return p1.pid == p2.pid; }),
                     std::end(process_list));

  return true;
}
}  // namespace sys_stats
