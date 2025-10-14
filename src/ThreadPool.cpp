#include "ThreadPool.h"

ThreadPool::ThreadPool(size_t numThreads) : stop(false) {
    for (size_t i = 0; i < numThreads; ++i) {
        workers.emplace_back(&ThreadPool::workerThread, this);
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread &worker : workers) {
        worker.join();
    }
}

void ThreadPool::enqueueTask(std::function<void()> task) {
    {
        std::unique_lock<std::mutex> lock(queueMutex);
        tasks.emplace(std::move(task));
    }
    condition.notify_one();
}

void ThreadPool::workerThread() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            condition.wait(lock, [this]() { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) return;
            task = std::move(tasks.front());
            tasks.pop();
        }
        task(); // Execute the task
    }
}

/*
    below commented out is a more complex version with error handling and debug printing
    potentially causes delay errors while waiting for tasks
*/


// #include "ThreadPool.h"

// #pragma once

// #include <iostream>
// #include <mutex>
// #include <sstream>
// #include <chrono>
// #include <iomanip>
// #include <thread>

// inline std::mutex safe_print_mutex;

// template<typename... Args>
// void safe_print(Args&&... args) {
//     std::lock_guard<std::mutex> lock(safe_print_mutex);

//     // Get current time
//     auto now = std::chrono::system_clock::now();
//     auto in_time = std::chrono::system_clock::to_time_t(now);
//     auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
//                   now.time_since_epoch()) % 1000;

//     std::stringstream ss;

//     ss << "[" << std::put_time(std::localtime(&in_time), "%H:%M:%S")
//        << "." << std::setw(3) << std::setfill('0') << ms.count()
//        << "][Thread " << std::this_thread::get_id() << "] ";

//     // Fold arguments into stream
//     (ss << ... << std::forward<Args>(args));

//     std::cout << ss.str() << std::endl;
// }




// ThreadPool::ThreadPool(size_t numThreads) : stop(false) {
//     for (size_t i = 0; i < numThreads; ++i) {
//         workers.emplace_back(&ThreadPool::workerThread, this);
//         safe_print("Started worker thread ", i + 1, " of ", numThreads);
//     }
// }

// ThreadPool::~ThreadPool() {
//     {
//         std::unique_lock<std::mutex> lock(queueMutex);
//         stop = true;
//     }
//     condition.notify_all();
//     for (std::thread &worker : workers) {
//         worker.join();
//     }
// }

// void ThreadPool::enqueueTask(std::function<void()> task) {
//     {
//         std::unique_lock<std::mutex> lock(queueMutex);
//         if (stop)
//             throw std::runtime_error("enqueue on stopped ThreadPool");
//         tasks.emplace(std::move(task));
//     }
//     condition.notify_one();
// }

// void ThreadPool::workerThread() {
//     while (true) {
//         safe_print("Worker thread waiting for tasks...");
//         std::function<void()> task;
//         {
//             std::unique_lock<std::mutex> lock(queueMutex);
//             condition.wait(lock, [this]() { return stop || !tasks.empty(); });
//             if (stop && tasks.empty()) return;
//             task = std::move(tasks.front());
//             tasks.pop();
//         }
//         try {
//             task();
//             safe_print("Finished ThreadPool task.");
//         } catch (const std::exception& e) {
//             safe_print(std::string("Exception in ThreadPool task: ") + e.what());
//         } catch (...) {
//             safe_print("Unknown exception in ThreadPool task.");
//         }
//     }
// }