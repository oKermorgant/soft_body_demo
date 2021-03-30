#include "ThreadPool/ThreadPool.h"

#include <thread>
#include <iostream>
#include <map>
#include <cstdlib>

std::mutex mutex;

std::map<decltype(std::this_thread::get_id()), int> thread_id;

struct Worker
{
 void computeSquareAndWait(int val)
 {
   auto id(std::this_thread::get_id());
   mutex.lock();
   // add thread_id just for display
   if(thread_id.find(id) == thread_id.end())
     thread_id[id] = thread_id.size()+1;

   // colored output
   std::cout << "\033[1;" << 30+thread_id[id] << "m"
             << "[thread #" << thread_id[id] << "]: adding " << val << "^2"
             << "\033[0m" << std::endl;
   total += val*val;

   mutex.unlock();

   std::this_thread::sleep_for(std::chrono::milliseconds(100 + rand()%50));
 }
  int total = 0;
};



int main()
{
  nbsdx::concurrent::ThreadPool<4> pool;

  Worker worker;


  for(int i = 0; i < 100; ++i)
  {
    pool.AddJob([&worker,i](){worker.computeSquareAndWait(i);});
  }

  while(pool.JobsRemaining())
  {
    mutex.lock();
    std::cout << pool.JobsRemaining() << " remaining" << std::endl;
    mutex.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  pool.WaitAll();

  std::cout << "total: " << worker.total << std::endl;





}
