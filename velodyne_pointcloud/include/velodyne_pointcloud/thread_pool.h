#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool
{
  public:
    ThreadPool() = default;

    explicit ThreadPool(int num_threads_)
    {
        init(num_threads_);
    }

    ~ThreadPool()
    {
        if (!terminate_pool_)
            shutdown();
    }

    void init(int num_threads)
    {
        num_threads_ = num_threads;
        for (int i = 0; i < num_threads; i++)
            threads_.emplace_back(&ThreadPool::infiniteLoop, this);
    }

    void infiniteLoop()
    {
        while (true)
        {
            std::function<void()> job;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                condition_.wait(lock, [this]() { return !jobs_.empty() || terminate_pool_; });
                if (terminate_pool_)
                    return;
                job = jobs_.front();
                jobs_.pop();
            }

            job();

            {
                std::lock_guard<std::mutex> lg(jobs_finished_mutex_);
                num_jobs_finished++;
            }
            condition_jobs_finished_.notify_one();
        }
    }

    void addJob(const std::function<void()>& job)
    {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            jobs_.push(job);
        }

        condition_.notify_one();
    }

    void waitUntilNumJobsFinished(int num_jobs)
    {
        {
            std::unique_lock<std::mutex> lock(jobs_finished_mutex_);
            condition_jobs_finished_.wait(lock, [this, num_jobs]() { return num_jobs_finished == num_jobs; });
            num_jobs_finished = 0;
        }
    }

    template<class T>
    void shareWork(int num_bins, void (T::*job)(int, int), T* obj)
    {
        int num_bins_per_thread = static_cast<int>(std::ceil(static_cast<double>(num_bins) / num_threads_));
        for (int i = 0; i < num_threads_; i++)
        {
            int start_bin_idx = i * num_bins_per_thread;
            int end_bin_idx = std::min(num_bins, start_bin_idx + num_bins_per_thread);
            auto f = std::bind(job, obj, start_bin_idx, end_bin_idx);
            addJob(f);
        }
        waitUntilNumJobsFinished(num_threads_);
    }

    void shutdown()
    {
        terminate_pool_ = true;
        condition_.notify_all(); // wake up all threads.

        // Join all threads.
        for (std::thread& th : threads_)
            th.join();

        threads_.clear();
    }

    std::vector<std::thread> threads_;
    int num_threads_{0};
    std::queue<std::function<void()>> jobs_;
    std::mutex queue_mutex_;
    std::mutex jobs_finished_mutex_;
    int num_jobs_finished{0};
    std::condition_variable condition_;
    std::condition_variable condition_jobs_finished_;
    bool terminate_pool_{};
};
