/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/serialization/base64.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace serialization {

void TestBuffer(const std::vector<uint8_t>& buffer, const std::string& expected) {
  EXPECT_EQ(Base64Encode(buffer.data(), buffer.size()), expected);
}

TEST(Serialization, SmallBuffer1) {
  TestBuffer({0}, "AA==");
  TestBuffer({0,0}, "AAA=");
  TestBuffer({0,0,0}, "AAAA");
  TestBuffer({0,0,0,0}, "AAAAAA==");
  TestBuffer({17}, "EQ==");
  TestBuffer({17,13}, "EQ0=");
  TestBuffer({17,13,11}, "EQ0L");
  TestBuffer({17,13,11,7}, "EQ0LBw==");
}

TEST(Serialization, Base64Image) {
  const std::string expected = "Qk02CQAAAAAAADYAAAAoAAAAIAAAABgAAAABABgAAAAAAAAJAAAAAAAAAAAAAAAAAAA"
      "AAAAAoqGgpaSjqKemq6qprq2ssbCvtLOyt7a1urm4vby7wL++w8LBxsXEycjHzMvKz87N0tHQ1dTT2NfW29rZ3t3c4eD"
      "f5OPi5+bl6uno7ezr8O/u8/Lx9vX0+fj3/Pv6//79QkFARURDSEdGS0pJTk1MUVBPVFNSV1ZVWllYXVxbYF9eY2JhZmV"
      "kaWhnbGtqb25tcnFwdXRzeHd2e3p5fn18gYB/hIOCh4aFiomIjYyLkI+Ok5KRlpWUmZiXnJuan56d4uHg5eTj6Ofm6+r"
      "p7u3s8fDv9PPy9/b1+vn4/fz7AP/+AwIBBgUECQgHDAsKDw4NEhEQFRQTGBcWGxoZHh0cISAfJCMiJyYlKikoLSwrMC8"
      "uMzIxNjU0OTg3PDs6Pz49goGAhYSDiIeGi4qJjo2MkZCPlJOSl5aVmpmYnZyboJ+eo6KhpqWkqainrKuqr66tsrGwtbS"
      "zuLe2u7q5vr28wcC/xMPCx8bFysnIzczL0M/O09LR1tXU2djX3Nva397dIiEgJSQjKCcmKyopLi0sMTAvNDMyNzY1Ojk"
      "4PTw7QD8+Q0JBRkVESUhHTEtKT05NUlFQVVRTWFdWW1pZXl1cYWBfZGNiZ2ZlamlobWxrcG9uc3JxdnV0eXh3fHt6f35"
      "9wsHAxcTDyMfGy8rJzs3M0dDP1NPS19bV2tnY3dzb4N/e4+Lh5uXk6ejn7Ovq7+7t8vHw9fTz+Pf2+/r5/v38AQD/BAM"
      "CBwYFCgkIDQwLEA8OExIRFhUUGRgXHBsaHx4dYmFgZWRjaGdma2ppbm1scXBvdHNyd3Z1enl4fXx7gH9+g4KBhoWEiYi"
      "HjIuKj46NkpGQlZSTmJeWm5qZnp2coaCfpKOip6alqqmorayrsK+us7KxtrW0ubi3vLu6v769AgEABQQDCAcGCwoJDg0"
      "MERAPFBMSFxYVGhkYHRwbIB8eIyIhJiUkKSgnLCsqLy4tMjEwNTQzODc2Ozo5Pj08QUA/RENCR0ZFSklITUxLUE9OU1J"
      "RVlVUWVhXXFtaX15doqGgpaSjqKemq6qprq2ssbCvtLOyt7a1urm4vby7wL++w8LBxsXEycjHzMvKz87N0tHQ1dTT2Nf"
      "W29rZ3t3c4eDf5OPi5+bl6uno7ezr8O/u8/Lx9vX0+fj3/Pv6//79QkFARURDSEdGS0pJTk1MUVBPVFNSV1ZVWllYXVx"
      "bYF9eY2JhZmVkaWhnbGtqb25tcnFwdXRzeHd2e3p5fn18gYB/hIOCh4aFiomIjYyLkI+Ok5KRlpWUmZiXnJuan56d4uH"
      "g5eTj6Ofm6+rp7u3s8fDv9PPy9/b1+vn4/fz7AP/+AwIBBgUECQgHDAsKDw4NEhEQFRQTGBcWGxoZHh0cISAfJCMiJyY"
      "lKikoLSwrMC8uMzIxNjU0OTg3PDs6Pz49goGAhYSDiIeGi4qJjo2MkZCPlJOSl5aVmpmYnZyboJ+eo6KhpqWkqainrKu"
      "qr66tsrGwtbSzuLe2u7q5vr28wcC/xMPCx8bFysnIzczL0M/O09LR1tXU2djX3Nva397dIiEgJSQjKCcmKyopLi0sMTA"
      "vNDMyNzY1Ojk4PTw7QD8+Q0JBRkVESUhHTEtKT05NUlFQVVRTWFdWW1pZXl1cYWBfZGNiZ2ZlamlobWxrcG9uc3JxdnV"
      "0eXh3fHt6f359wsHAxcTDyMfGy8rJzs3M0dDP1NPS19bV2tnY3dzb4N/e4+Lh5uXk6ejn7Ovq7+7t8vHw9fTz+Pf2+/r"
      "5/v38AQD/BAMCBwYFCgkIDQwLEA8OExIRFhUUGRgXHBsaHx4dYmFgZWRjaGdma2ppbm1scXBvdHNyd3Z1enl4fXx7gH9"
      "+g4KBhoWEiYiHjIuKj46NkpGQlZSTmJeWm5qZnp2coaCfpKOip6alqqmorayrsK+us7KxtrW0ubi3vLu6v769AgEABQQ"
      "DCAcGCwoJDg0MERAPFBMSFxYVGhkYHRwbIB8eIyIhJiUkKSgnLCsqLy4tMjEwNTQzODc2Ozo5Pj08QUA/RENCR0ZFSkl"
      "ITUxLUE9OU1JRVlVUWVhXXFtaX15doqGgpaSjqKemq6qprq2ssbCvtLOyt7a1urm4vby7wL++w8LBxsXEycjHzMvKz87"
      "N0tHQ1dTT2NfW29rZ3t3c4eDf5OPi5+bl6uno7ezr8O/u8/Lx9vX0+fj3/Pv6//79QkFARURDSEdGS0pJTk1MUVBPVFN"
      "SV1ZVWllYXVxbYF9eY2JhZmVkaWhnbGtqb25tcnFwdXRzeHd2e3p5fn18gYB/hIOCh4aFiomIjYyLkI+Ok5KRlpWUmZi"
      "XnJuan56d4uHg5eTj6Ofm6+rp7u3s8fDv9PPy9/b1+vn4/fz7AP/+AwIBBgUECQgHDAsKDw4NEhEQFRQTGBcWGxoZHh0"
      "cISAfJCMiJyYlKikoLSwrMC8uMzIxNjU0OTg3PDs6Pz49goGAhYSDiIeGi4qJjo2MkZCPlJOSl5aVmpmYnZyboJ+eo6K"
      "hpqWkqainrKuqr66tsrGwtbSzuLe2u7q5vr28wcC/xMPCx8bFysnIzczL0M/O09LR1tXU2djX3Nva397dIiEgJSQjKCc"
      "mKyopLi0sMTAvNDMyNzY1Ojk4PTw7QD8+Q0JBRkVESUhHTEtKT05NUlFQVVRTWFdWW1pZXl1cYWBfZGNiZ2ZlamlobWx"
      "rcG9uc3JxdnV0eXh3fHt6f359wsHAxcTDyMfGy8rJzs3M0dDP1NPS19bV2tnY3dzb4N/e4+Lh5uXk6ejn7Ovq7+7t8vH"
      "w9fTz+Pf2+/r5/v38AQD/BAMCBwYFCgkIDQwLEA8OExIRFhUUGRgXHBsaHx4dYmFgZWRjaGdma2ppbm1scXBvdHNyd3Z"
      "1enl4fXx7gH9+g4KBhoWEiYiHjIuKj46NkpGQlZSTmJeWm5qZnp2coaCfpKOip6alqqmorayrsK+us7KxtrW0ubi3vLu"
      "6v769AgEABQQDCAcGCwoJDg0MERAPFBMSFxYVGhkYHRwbIB8eIyIhJiUkKSgnLCsqLy4tMjEwNTQzODc2Ozo5Pj08QUA"
      "/RENCR0ZFSklITUxLUE9OU1JRVlVUWVhXXFtaX15d";
  Image3ub image(24, 32);
  for (int i = 0; i < image.num_elements(); i++) {
    image.element_wise_begin()[i] = static_cast<unsigned char>(i % 256);
  }
  const std::string actual = Base64Encode(image);
  EXPECT_EQ(actual, expected);
}

}  // namespace serialization
}  // namespace isaac
