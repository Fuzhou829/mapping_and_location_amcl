/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-01 08:26:15
 * @LastEditTime: 2023-06-28 04:32:50
 */

#pragma once

#include <memory>
#include <functional>
#include <set>

#include "common/thread_pool.h"
#include "glog/logging.h"

#include "common/mutex.h"
#include "common/logger.h"

namespace internal_common {

class ThreadPoolInterface;

class Task {
 public:
  friend class ThreadPoolInterface;

  using WorkItem = std::function<void()>;
  /**
    NEW：新建任务, 还未schedule到线程池
    DISPATCHED： 任务已经schedule 到线程池
    DEPENDENCIES_COMPLETED： 任务依赖已经执行完成
    RUNNING： 任务执行中
    COMPLETED： 任务完成

    对任一个任务的状态转换顺序为：
    NEW->DISPATCHED->DEPENDENCIES_COMPLETED->RUNNING->COMPLETED
  */
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

  Task() = default;
  ~Task();

  State GetState() EXCLUDES(mutex_);

  // State must be 'NEW'.
  void SetWorkItem(const WorkItem& work_item) EXCLUDES(mutex_);

  // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
  // assumed completed.
  void AddDependency(std::weak_ptr<Task> dependency) EXCLUDES(mutex_);

 private:
  // Allowed in all states.
  void AddDependentTask(Task* dependent_task);

  // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
  void Execute() EXCLUDES(mutex_);

  // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
  void SetThreadPool(ThreadPoolInterface* thread_pool) EXCLUDES(mutex_);

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
  // 'DEPENDENCIES_COMPLETED'.
  void OnDependenyCompleted();

  WorkItem work_item_ GUARDED_BY(mutex_);
  ThreadPoolInterface* thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
  State state_ GUARDED_BY(mutex_) = NEW;
  unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
  std::set<Task*> dependent_tasks_ GUARDED_BY(mutex_);

  Mutex mutex_;
};






}  // namespace internal_common
