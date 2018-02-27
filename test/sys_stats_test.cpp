#include <unistd.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <atomic>
#include <thread>
#include <pthread.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "sys_stats/sys_stats.h"

TEST(SysStatsTest, checksCpuUsage)
{
  sys_stats::SysStats s;
  s.update();
  ASSERT_GT(s.cpu.size(), 0);
  //
  // wait until there has been some CPU work
  // or timeout
  while (s.cpu[0].cpu_usage == 0)
  {
    usleep(10000);
    s.update();
  }

  ASSERT_GT(s.cpu[0].cpu_usage, 0);
  ASSERT_GT(s.cpu[0].cpu_frequency, 0);
  ASSERT_GT(s.cpu[0].cpu_temperature, 0);
  ASSERT_GT(s.cpu_use_total, 0);
}

TEST(SysStatsTest, checkMemUsage)
{
  sys_stats::SysStats s;
  s.update();
  ASSERT_GT(s.mem_use_total, 0);
  ASSERT_GE(s.swap_use_total, 0);
}

/// Thread for testing the thread count
std::atomic<bool> exit_thread{false};

void cpu_thread()
{
  while (!exit_thread)
  {
    float n = 1.7;
    while (n < 10000)
      n *= 2;
    usleep(1000);
  }
}

TEST(SysStatsTest, checkProcParsing)
{
  sys_stats::SysStats s;
  s.update();

  ASSERT_GT(s.processes.size(), 0);

  int pid = getpid();
  auto self = std::find_if(
      std::begin(s.processes), std::end(s.processes), [pid](const sys_stats::Process& p) { return p.pid == pid; });

  ASSERT_NE(self, std::end(s.processes));
  ASSERT_GT((*self).process_name.size(), 0);

  // generate some CPU usage in this process
  do
  {
    float n = 1.7;
    while (n < 10000)
      n *= 2;

    s.update();
    self = std::find_if(
        std::begin(s.processes), std::end(s.processes), [pid](const sys_stats::Process& p) { return p.pid == pid; });
    ASSERT_NE(self, std::end(s.processes));
  } while ((*self).cpu_use == 0);

  ASSERT_GT((*self).mem_use, 0);
  ASSERT_EQ((*self).threads.size(), 0);

  std::thread t(cpu_thread);
  pthread_setname_np(t.native_handle(), "test thread");
  s.update();
  self = std::find_if(
      std::begin(s.processes), std::end(s.processes), [pid](const sys_stats::Process& p) { return p.pid == pid; });
  ASSERT_NE(self, std::end(s.processes));
  ASSERT_EQ((*self).threads.size(), 1);

  while ((*self).threads[0].cpu_use == 0)
  {
    usleep(10000);
    s.update();
    self = std::find_if(
        std::begin(s.processes), std::end(s.processes), [pid](const sys_stats::Process& p) { return p.pid == pid; });
    ASSERT_NE(self, std::end(s.processes));
    ASSERT_EQ((*self).threads[0].thread_name, "test thread");
  }

  exit_thread = true;
  t.join();
  s.update();
  self = std::find_if(
      std::begin(s.processes), std::end(s.processes), [pid](const sys_stats::Process& p) { return p.pid == pid; });
  ASSERT_NE(self, std::end(s.processes));
  ASSERT_EQ((*self).threads.size(), 0);
}

TEST(SysStatsTest, checkDiskSpace)
{
  sys_stats::SysStats s;
  s.update();

  ASSERT_GT(s.disks.size(), 0);
  for (const auto& d : s.disks)
  {
    ASSERT_GT(s.disks[0].disk_name.size(), 0);
    if (d.free_space > 0)
      return;
  }
  // flaky, depends on single disk with some space
  ASSERT_EQ(true, false);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
