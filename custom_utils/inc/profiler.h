#pragma once
#include <chrono>
#include <string>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <iostream>
#include <memory>
#include <vector>

namespace custom_utils {
namespace profiler {

// Lightweight, thread-safe statistic for timed events
struct Stat {
    std::atomic<uint64_t> count{0};          // number of events
    std::atomic<uint64_t> total_ns{0};       // accumulated time in nanoseconds

    void add(uint64_t ns) {
        total_ns.fetch_add(ns, std::memory_order_relaxed);
        count.fetch_add(1, std::memory_order_relaxed);
    }
    void add_count(uint64_t c) {
        count.fetch_add(c, std::memory_order_relaxed);
    }
    void reset() {
        count.store(0);
        total_ns.store(0);
    }
};

// Central profiler that stores named stats. Minimal dependencies so it can be
// used from many modules. Thread-safe for updates.
class Profiler {
public:
    Profiler() = default;

    // Add an observation (ns) to the named stat
    void add(const std::string& name, uint64_t ns) {
        auto s = get_or_create(name);
        s->add(ns);
    }

    // Increment count-only metric
    void inc(const std::string& name, uint64_t cnt = 1) {
        auto s = get_or_create(name);
        s->add_count(cnt);
    }

    // Reset all stored stats
    void reset() {
        std::lock_guard<std::mutex> lg(m_);
        for (auto &p : stats_) p.second->reset();
    }

    // Print a short report to the provided stream (defaults to cerr)
    void report(std::ostream& os = std::cerr) const {
        std::lock_guard<std::mutex> lg(m_);
        os << "[Profiler] report:\n";
        os << " name, count, total_ms, avg_ms\n";
        for (const auto &p : stats_) {
            const std::string &name = p.first;
            const Stat &s = *p.second;
            uint64_t cnt = s.count.load();
            double total_ms = double(s.total_ns.load()) / 1e6;
            double avg_ms = cnt ? (total_ms / double(cnt)) : 0.0;
            os << " " << name << ", " << cnt << ", " << total_ms << ", " << avg_ms << "\n";
        }
        os << std::flush;
    }

private:
    std::shared_ptr<Stat> get_or_create(const std::string& name) {
        {
            std::lock_guard<std::mutex> lg(m_);
            auto it = stats_.find(name);
            if (it != stats_.end()) return it->second;
        }
        // create
        auto s = std::make_shared<Stat>();
        //std::lock_guard<std::mutex> lg(m_);
        auto[it, inserted] = stats_.emplace(name, s);
        return it->second;
    }

    mutable std::mutex m_;
    std::unordered_map<std::string, std::shared_ptr<Stat>> stats_;
};

// RAII scoped timer that records elapsed time (ns) to a Profiler under a name.
class ScopedTimer {
public:
    ScopedTimer(Profiler& prof, const std::string& name) : prof_(prof), name_(name), start_(std::chrono::steady_clock::now()) {}
    ~ScopedTimer() {
        auto end = std::chrono::steady_clock::now();
        uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count();
        prof_.add(name_, ns);
    }
    // non-copyable
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
private:
    Profiler& prof_;
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};

// Convenience module-level profiler wrapper. Modules can define their own
// names and provide typed accessors for common timers/counters.
class ModuleProfiler {
public:
    explicit ModuleProfiler(Profiler& p, const std::string& module_prefix = "module") : prof(p), prefix(module_prefix) {}
    // Start a scoped timer for a named event under this module
    ScopedTimer scoped(const std::string& name) { return ScopedTimer(prof, prefix + ":" + name); }
    void inc(const std::string& name, uint64_t cnt = 1) { prof.inc(prefix + ":" + name, cnt); }
    void add_ns(const std::string& name, uint64_t ns) { prof.add(prefix + ":" + name, ns); }
private:
    Profiler& prof;
    std::string prefix;
};

} // namespace profiler
} // namespace custom_utils
