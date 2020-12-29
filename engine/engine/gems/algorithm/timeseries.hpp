/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <deque>
#include <iterator>
#include <utility>

#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"

namespace isaac {

// Stores a list of timestamped values in timestamp order
template <typename State, typename Stamp = double>
class Timeseries {
 public:
  static constexpr ssize_t kInvalid = -1;

  // A pair of timestamp and state
  struct Entry {
    Stamp stamp;
    State state;
  };

  // Queries the number of elements in the timeseries
  bool empty() const { return entries_.empty(); }
  size_t size() const { return entries_.size(); }

  // Empties the time series
  void clear() {
    entries_.clear();
  }

  // Adds an entry to the end of the timeseries. Will assert if timestamp are out of order.
  void push(Stamp stamp, State state) {
    const bool ok = tryPush(std::move(stamp), std::move(state));
    ASSERT(ok, "Timestamps out of order: %f !<= %f", entries_.back().stamp, stamp);
  }
  // Same as `push` but returns false if timestamps are out of order instead of asserting.
  bool tryPush(Stamp stamp, State state) {
    if (!empty() && stamp < entries_.back().stamp) {
      return false;
    }
    entries_.emplace_back(Entry{std::move(stamp), std::move(state)});
    return true;
  }
  // Inserts an element at the correct time. This always succeeds, is slower than pushing push and
  // invalidates previously computed indices for more recent entries.
  void insert(Stamp stamp, State state) {
    entries_.insert(std::next(entries_.begin(), upper_index(stamp)),
        Entry{std::move(stamp), std::move(state)});
  }
  // Erases the element at the given time. Will return false if no item with this timestamp.
  bool erase(Stamp stamp) {
    const ssize_t index = find(stamp);
    if (index == -1) return false;
    entries_.erase(std::next(entries_.begin(), index));
    return true;
  }

  // Removes all entries with timestamps smaller than the given timestamp. Returns the number of
  // elements which were removed.
  size_t rejuvenate(const Stamp& stamp) {
    const ssize_t index = lower_index(stamp);
    if (index <= 0) {
      return 0;
    }
    entries_.erase(entries_.begin(), std::next(entries_.begin(), index));
    return index;
  }

  // Gets the oldest entry (smallest timestamp)
  // This will assert if the timeseries is empty.
  const Entry& oldest() const {
    ASSERT(!empty(), "Can not get oldest entry in empty timeseries.");
    return entries_.front();
  }
  // Gets the youngest entry (largest timestamp)
  // This will assert if the timeseries is empty.
  const Entry& youngest() const {
    ASSERT(!empty(), "Can not get youngest entry in empty timeseries.");
    return entries_.back();
  }
  // Gets the entry at the given index
  // This will assert if the requested element is out of bounds.
  const Entry& at(size_t index) const {
    ASSERT(index < size(), "Index out of bounds: %zd !<= %zd", index, size());
    return entries_[index];
  }
  // Gets the state at the given index
  // This will assert if the requested element is out of bounds.
  const State& state(size_t index) const {
    return at(index).state;
  }
  State& state(size_t index) {
    ASSERT(index < size(), "Index out of bounds: %zd !<= %zd", index, size());
    return entries_[index].state;
  }

  // Forgets the beginning of history if it contains too many elements
  void forgetBySize(size_t max_size) {
    if (entries_.size() <= max_size) {
      return;
    }
    entries_.erase(entries_.begin(), std::next(entries_.begin(), entries_.size() - max_size));
  }

  // Returns the index of an element with exactly the given timestamp, or -1 otherwise
  ssize_t find(const Stamp& stamp) const {
    const ssize_t index = lower_index(stamp);
    return (index >= 0 && at(index).stamp == stamp) ? index : -1;
  }

  // The index of the entry with the greatest timestamp smaller or equal to the given timestamp.
  // Returns -1 if no such element exists.
  // If a valid index is returned the following condition is true:
  //    stamp(lower_index(t)) <= t < stamp(upper_index(t))
  ssize_t lower_index(const Stamp& t) const {
    return upper_index(t) - 1;
  }

  // The index of the entry with the smallest timestamp greater than the given timestamp. Returns
  // number-of-elements if no such element exists.
  // If a valid index is returned the following condition is true:
  //    stamp(lower_index(t)) <= t < stamp(upper_index(t))
  ssize_t upper_index(const Stamp& t) const {
    auto it = std::upper_bound(entries_.begin(), entries_.end(), t, CompareEntryToStamp{});
    return std::distance(entries_.begin(), it);
  }

  // Returns an index i such at(i) <= t < at(i+1).
  // Border cases are handled as follows:
  //   * returns  -1 if the timeseries is empty
  //   * returns   0 if t <= at(0), i.e. the asked time is before recorded history
  //   * returns n-1 if t >= at(n-1), i.e. the asked time is in the future
  ssize_t interpolate_2_index(const Stamp& t) const {
    if (entries_.empty()) {
      return -1;
    }
    if (t <= entries_.front().stamp) {
      return 0;
    }
    if (t >= entries_.back().stamp) {
      return static_cast<ssize_t>(size()) - 1;
    }
    const ssize_t b = upper_index(t);
    const ssize_t a = b - 1;
    ASSERT(0 <= a && b < static_cast<ssize_t>(size()), "Logic error %zd / %zd / %zd", a, b, size());
    return a;
  }

  // Samples a value for the given timestamp using the given two-point interpolation function.
  // If the timestamp is out of range the first or last state will be returned. If the timeseries
  // is empty std::nullopt is returned.
  // f: (const Stamp&, const Entry&, const Entry&) -> State,
  //    (t, a, b) -> (1.0 - p)*a_x + p*b_x, where p = (t - a_t) / (b_t - a_t).
  // The interpolation function is only called for a_t < t < b_t.
  template <typename F>
  std::optional<State> interpolate_2(const Stamp& stamp, F f) const {
    const ssize_t a = interpolate_2_index(stamp);
    if (a == -1) {
      return std::nullopt;
    }
    return interpolate_2(stamp, a, std::move(f));
  }
  // Same as `interpolate_2` but with pre-computed index, e.g. from interpolate_2_index
  template <typename F>
  State interpolate_2(const Stamp& stamp, ssize_t a, F f) const {
    ASSERT(0 <= a && a < static_cast<ssize_t>(size()), "Index out of range: %lld / %llu", a,
           size());
    // This check guarantees that the interpolate function is called with a_t < t < b_t and that
    // We return the correct value in case we don't have enough room to interpolate but have a
    // value for exactly the given time.
    if (stamp <= entries_[a].stamp || a == static_cast<ssize_t>(size()) - 1) {
      return entries_[a].state;
    }
    // interpolate
    return f(stamp, entries_[a], entries_[a + 1]);
  }

  // Similar to `interpolate_2`, but with the following interpolate function:
  // f: (K, const State&, const State&) -> State,
  //    (p, a_x, b_x) -> (1.0 - p)*a_x + p*b_x.
  // p is computed from timestamps as p = (t - a_t) / (b_t - a_t) assuming that differences between
  // timestamp are convertible to K. The interpolation function is only called for 0 < p < 1.
  template <typename F, typename K = double>
  std::optional<State> interpolate_2_p(const Stamp& stamp, F f) const {
    return interpolate_2(stamp,
        [&] (const Stamp& t, const Entry& a, const Entry& b) {
          const K p = static_cast<K>(t - a.stamp) / static_cast<K>(b.stamp - a.stamp);
          return f(p, a.state, b.state);
        });
  }
  template <typename F, typename K = double>
  State interpolate_2_p(const Stamp& stamp, ssize_t a, F f) const {
    return interpolate_2(stamp, a,
        [&] (const Stamp& t, const Entry& a, const Entry& b) {
          const K p = static_cast<K>(t - a.stamp) / static_cast<K>(b.stamp - a.stamp);
          return f(p, a.state, b.state);
        });
  }

 private:
  struct CompareEntryToStamp {
    bool operator()(const Stamp& lhs, const Entry& rhs) const {
      return lhs < rhs.stamp;
    }
  };

  // TODO This is not the most optimal data type, but good enough for now.
  std::deque<Entry> entries_;
};

}  // namespace isaac
