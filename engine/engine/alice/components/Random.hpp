/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <random>
#include <type_traits>
#include <vector>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/math/types.hpp"

namespace isaac {
namespace alice {

// Helper component to generate random numbers.
// FIXME(dweikersdorf) This should be a component, however currently configuration is not setup
// yet when the call to initialize happens.
class Random : public Codelet {
 public:
  void start() override;

  // Return a random number using the given distribution.
  template <typename T, template<typename> typename Distribution>
  T sample(Distribution<T>& dist) {
    std::unique_lock<std::mutex> lock(mutex_);
    return dist(rng_);
  }

  // Return a random real using the range (min and max are included).
  template <typename T>
  T sampleUniformReal(T min, T max) {
    static_assert(std::is_floating_point<T>::value, "sampleUniformReal expects a real number type");
    std::unique_lock<std::mutex> lock(mutex_);
    return std::uniform_real_distribution<T>(min, max)(rng_);
  }
  template <typename T>
  T sampleUniformReal(const Vector2<T>& range) {
    return sampleUniformReal(range[0], range[1]);
  }

  // Return a random integer using the range (min and max are included).
  template <typename I>
  I sampleUniformInt(I min, I max) {
    static_assert(std::is_integral<I>::value, "sampleUniformInt expects an integer type");
    std::unique_lock<std::mutex> lock(mutex_);
    return std::uniform_int_distribution<I>(min, max)(rng_);
  }
  template <typename I>
  I sampleUniformInt(const Vector2<I>& range) {
    return sampleUniformInt(range[0], range[1]);
  }

  // Returns a random integer `index` such that: 0 <= index < count.
  template <typename I>
  I sampleUniformIndex(I count) {
    static_assert(std::is_integral<I>::value, "sampleUniformInt expects an integer type");
    ASSERT(count > 0, "`count` (%s) must be positive", std::to_string(count).c_str());
    std::unique_lock<std::mutex> lock(mutex_);
    return std::uniform_int_distribution<I>(0, count - 1)(rng_);
  }

  // Samples a vector in which element is within the corresponding interval defined by the given
  // vectors min and max.
  template <typename K, int N>
  Vector<K, N> sampleUniformRealVector(const Vector<K, N>& min, const Vector<K, N>& max) {
    Vector<K, N> result;
    for (int i = 0; i < N; i++) {
      result[i] = sampleUniformReal(min[i], max[i]);
    }
    return result;
  }
  // Samples a vector in which element is within the corresponding interval defined by
  // [-range|range]. In different words this is identical to
  //   sampleUniformRealVector(-range, +range)
  template <typename K, int N>
  Vector<K, N> sampleUniformRealVector(const Vector<K, N>& range) {
    Vector<K, N> result;
    for (int i = 0; i < N; i++) {
      result[i] = sampleUniformReal(-range[i], +range[i]);
    }
    return result;
  }

  // Shuffles the given object using the range iterator
  template <typename Iterator>
  void shuffle(Iterator start, Iterator end) {
    std::unique_lock<std::mutex> lock(mutex_);
    std::shuffle(start, end, rng_);
  }

  // Returns a random unsigned 32 bits integer
  uint32_t sampleSeed() {
    std::unique_lock<std::mutex> lock(mutex_);
    return rng_();
  }

  // Returns a sample from a normal distribution
  template <typename K>
  K sampleGaussian(K standard_deviation) {
    static_assert(std::is_floating_point<K>::value,
                  "sampleGaussianVector expects a real number type");
    std::unique_lock<std::mutex> lock(mutex_);
    return std::normal_distribution<K>(K(0), standard_deviation)(rng_);
  }

  // Returns a vector of the same side as the one provided by std_deviation containing random value
  // following a gaussian distribution of the given standard deviation.
  template <typename K, int N>
  Vector<K, N> sampleGaussianVector(const Vector<K, N>& std_deviation) {
    static_assert(std::is_floating_point<K>::value,
                  "sampleGaussianVector expects a real number type");
    static_assert(N > 0, "sampleGaussianVector expects a compile time sized Vector");
    std::unique_lock<std::mutex> lock(mutex_);
    Vector<K, N> samples;
    for (int i = 0; i < N; ++i) {
      samples(i) = std::normal_distribution<K>(K(0), std_deviation(i))(rng_);
    }
    return samples;
  }

  // Samples a coin flip, i.e. returns a random Boolean
  bool sampleCoin() {
    return sampleUniformInt(0, 1) == 0;
  }

  // Samples a coin flip for a biased coin, i.e. returns true if a random sample from unit interval
  // is smaller than the given probability.
  bool sampleCoin(double probability) {
    return sampleUniformReal(0.0, 1.0) < probability;
  }

  // Picks a random element from a list with equal probability
  template <typename K>
  const K& sampleChoice(const std::vector<K>& choices) {
    ASSERT(!choices.empty(), "must have at least one option");
    return choices[sampleUniformIndex(choices.size())];
  }

  // Weighted random sampling of an element from a discrete probability density function.
  // The index of the chosen element is returned. This functions needs a copy of `pdf` to compute
  // the CDF. Use `sampleChoiceCdf` instead if you have the CDF available.  It is an error to pass
  // an empty PDF.
  template <typename K>
  size_t sampleDiscretePdf(std::vector<K> pdf) {
    // Compute CDF
    for (size_t i = 1; i < pdf.size(); i++) {
      pdf[i] += pdf[i - 1];
    }
    return sampleDiscreteCdf(pdf);
  }

  // Weighted random sampling of an element from a cumulative distribution function.
  // The index of the chosen element is returned. It is an error to pass an empty CDF.
  template <typename K>
  size_t sampleDiscreteCdf(const std::vector<K>& cdf) {
    ASSERT(!cdf.empty(), "cdf must have at least one element");
    const K value = sampleUniformReal(K(0), cdf.back());
    const auto it = std::lower_bound(cdf.begin(), cdf.end(), value);
    if (it == cdf.end()) return cdf.size() - 1;
    return std::distance(cdf.begin(), it);
  }

  // Returns the underlying random number generator
  std::mt19937& rng() { return rng_; }

  // The seed used by the random engine. If use_random_seed is set to true, this seed will be
  // ignored.
  ISAAC_PARAM(int, seed, 0);
  // Whether or not using the default seed or use a random seed that will change from one execution
  // to another.
  ISAAC_PARAM(bool, use_random_seed, false);

 private:
  std::mt19937 rng_;
  std::mutex mutex_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::alice::Random)
