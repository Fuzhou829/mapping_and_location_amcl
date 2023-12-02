/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-01 08:26:15
 * @LastEditTime: 2023-06-28 04:32:57
 */

#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <thread>  // NOLINT
#include <vector>
#include <unordered_map>

#include "common/mutex.h"
#include "common/task.h"

namespace internal_common {

class Task;

class ThreadPoolInterface {
 public:
  ThreadPoolInterface() {}
  virtual ~ThreadPoolInterface() {}

  virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

 protected:
  void Execute(Task* task);
  void SetThreadPool(Task* task);

 private:
  friend class Task;
  virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
class ThreadPool : public ThreadPoolInterface {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  // When the returned weak pointer is expired, 'task' has certainly completed,
  // so dependants no longer need to add it as a dependency.
  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task)
    EXCLUDES(mutex_) override;

 private:
  void DoWork();

  void NotifyDependenciesCompleted(Task* task) EXCLUDES(mutex_) override;

  Mutex mutex_;
  bool running_ GUARDED_BY(mutex_) = true;
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  std::deque<std::shared_ptr<Task>> task_queue_ GUARDED_BY(mutex_);
  std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_
      GUARDED_BY(mutex_);
};





}  // namespace internal_common





