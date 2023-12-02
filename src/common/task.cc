/*
 * @Description: Copyright (C) 2022 山东亚历山大智能科技有限公司
 * @Version: 1.2
 * @Author: renjy
 * @Date: 2023-06-01 08:26:15
 * @LastEditTime: 2023-06-03 08:44:56
 */

#include "common/task.h"


namespace internal_common {

Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  if (state_ != NEW && state_ != COMPLETED) {
    SLAM_WARN("Delete Task between dispatch and completion.");
  }
}

Task::State Task::GetState() {
  MutexLocker locker(&mutex_);
  return state_;
}

void Task::SetWorkItem(const WorkItem& work_item) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;
}

void Task::AddDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, NEW);
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  if (shared_dependency) {
    shared_dependency->AddDependentTask(this);
  }
}

void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  MutexLocker locker(&mutex_);
  CHECK_EQ(state_, NEW);
  state_ = DISPATCHED;
  thread_pool_to_notify_ = thread_pool;
  if (uncompleted_dependencies_ == 0) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

void Task::AddDependentTask(Task* dependent_task) {
  MutexLocker locker(&mutex_);
  if (state_ == COMPLETED) {
    dependent_task->OnDependenyCompleted();
    return;
  }
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

void Task::OnDependenyCompleted() {
  MutexLocker locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  --uncompleted_dependencies_;
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

void Task::Execute() {
  {
    MutexLocker locker(&mutex_);
    CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
    state_ = RUNNING;
  }

  // Execute the work item.
  if (work_item_) {
    work_item_();
  }

  MutexLocker locker(&mutex_);
  state_ = COMPLETED;
  for (Task* dependent_task : dependent_tasks_) {
    dependent_task->OnDependenyCompleted();
  }
}


}  // namespace internal_common



