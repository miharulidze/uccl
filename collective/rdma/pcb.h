#pragma once

#include "eqds.h"
#include "pcm_vm.hpp"
#include "swift.h"
#include "timely.h"
#include "timing_wheel.h"
#include "util/util.h"
#include <glog/logging.h>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>

namespace uccl {

/**
 * @brief Protocol Control Block.
 */
inline uint64_t tsc_to_ns(uint64_t tsc) { return tsc / freq_ghz; }

// Expose a plain function pointer of signature `uint64_t (*)()` that the
// PCM VM expects. Initialize it from a non-capturing lambda (convertible
// to function pointer).
static uint64_t (*ns_rdtsc)() = []() -> uint64_t { return tsc_to_ns(rdtsc()); };

struct PCB {
  static constexpr std::size_t kSackBitmapBucketSize = sizeof(uint64_t) * 8;

  PCB() {}

  PCB(double link_bandwidth)
      : timely_cc(freq_ghz, link_bandwidth),
        swift_cc(freq_ghz, link_bandwidth),
        pcm_library{open_pcm_algo()} {
    if (pcm_library.first && kSenderCCA == SENDER_CCA_PCM) {
      pcm_cc = std::unique_ptr<pcm_vm::PcmHandlerVmDesc>(pcm_library.second()),
      pcm_cc->add_get_time_source(ns_rdtsc);
      pcm_io_slab = &pcm_cc->get_signal_io_slab();
    } else if (!pcm_library.first && kSenderCCA == SENDER_CCA_PCM) {
      throw std::runtime_error{"Failed to load PCM algorithm."};
    }
  }

  timely::TimelyCC timely_cc;

  swift::SwiftCC swift_cc;

  using pcm_factory_fn_ptr = pcm_vm::PcmHandlerVmDesc* (*)();
  using pcm_factory_so_ptr = std::shared_ptr<void>;
  std::pair<pcm_factory_so_ptr, pcm_factory_fn_ptr> pcm_library;
  std::unique_ptr<pcm_vm::PcmHandlerVmDesc> pcm_cc;
  // Pointer to the PCM VM's IO slab (points into the object owned by pcm_cc)
  pcm_vm::PcmHandlerVmIoSlab* pcm_io_slab = nullptr;

  eqds::EQDSCC eqds_cc;

  // Next sequence number to be sent.
  UINT_CSN snd_nxt{0};
  // Oldest unacknowledged sequence number.
  UINT_CSN snd_una{0};
  // Next expected sequence number to be received.
  UINT_CSN rcv_nxt{0};

  // SACK bitmap at the receiver side.
  uint64_t sack_bitmap[kSackBitmapSize / kSackBitmapBucketSize]{};
  uint8_t sack_bitmap_count{0};
  // SACK bitmap at the sender side.
  uint64_t tx_sack_bitmap[kSackBitmapSize / kSackBitmapBucketSize]{};
  uint8_t tx_sack_bitmap_count{0};
  // The starting CSN of the copy of SACK bitmap.
  uint32_t tx_sack_bitmap_base{0};

  // Timestamp of the last received data.
  uint64_t t_remote_nic_rx{0};

  // Incremented when a bitmap is shifted left by 1.
  // Even if increment every one microsecond, it will take 584542 years to
  // overflow.
  uint64_t shift_count;
  uint16_t duplicate_acks{0};
  uint16_t rto_rexmits_consectutive{0};
  UINT_CSN snd_ooo_acks{0};

  // Stats
  uint32_t stats_fast_rexmits{0};
  uint32_t stats_rto_rexmits{0};
  uint32_t stats_accept_retr{0};
  uint32_t stats_accept_barrier{0};
  uint32_t stats_chunk_drop{0};
  uint32_t stats_barrier_drop{0};
  uint32_t stats_retr_chunk_drop{0};
  uint32_t stats_ooo{0};
  uint32_t stats_real_ooo{0};
  uint32_t stats_maxooo{0};

  UINT_CSN seqno() const { return snd_nxt; }
  UINT_CSN get_snd_nxt() {
    UINT_CSN seqno = snd_nxt;
    snd_nxt += 1;
    return seqno;
  }

  UINT_CSN ackno() const { return rcv_nxt; }
  UINT_CSN get_rcv_nxt() const { return rcv_nxt; }
  void advance_rcv_nxt(UINT_CSN n) { rcv_nxt += n; }
  void advance_rcv_nxt() { rcv_nxt += 1; }

  void sack_bitmap_shift_left_one() {
    constexpr size_t sack_bitmap_bucket_max_idx =
        kSackBitmapSize / kSackBitmapBucketSize - 1;

    for (size_t i = 0; i < sack_bitmap_bucket_max_idx; i++) {
      // Shift the current each bucket to the left by 1 and take the most
      // significant bit from the next bucket
      uint64_t& sack_bitmap_left_bucket = sack_bitmap[i];
      uint64_t const sack_bitmap_right_bucket = sack_bitmap[i + 1];

      sack_bitmap_left_bucket =
          (sack_bitmap_left_bucket >> 1) | (sack_bitmap_right_bucket << 63);
    }

    // Special handling for the right most bucket
    uint64_t& sack_bitmap_right_most_bucket =
        sack_bitmap[sack_bitmap_bucket_max_idx];
    sack_bitmap_right_most_bucket >>= 1;

    sack_bitmap_count--;
  }

  // Check if the bit at the given index is set.
  bool sack_bitmap_bit_is_set(size_t const index) const {
    size_t const sack_bitmap_bucket_idx = index / kSackBitmapBucketSize;
    size_t const sack_bitmap_idx_in_bucket = index % kSackBitmapBucketSize;
    return sack_bitmap[sack_bitmap_bucket_idx] &
           (1ULL << sack_bitmap_idx_in_bucket);
  }
  // Set the bit at the given index.
  void sack_bitmap_bit_set(size_t const index) {
    size_t const sack_bitmap_bucket_idx = index / kSackBitmapBucketSize;
    size_t const sack_bitmap_idx_in_bucket = index % kSackBitmapBucketSize;

    LOG_IF(FATAL, index >= kSackBitmapSize) << "Index out of bounds: " << index;

    sack_bitmap[sack_bitmap_bucket_idx] |= (1ULL << sack_bitmap_idx_in_bucket);

    sack_bitmap_count++;
  }

 private:
  static std::pair<pcm_factory_so_ptr, pcm_factory_fn_ptr> open_pcm_algo() {
    char const* pcm_algo_env = std::getenv("UCCL_PCM_ALGO");
    if (!pcm_algo_env) {
      return {nullptr, nullptr};
    }
    std::string pcm_algo_name{pcm_algo_env};
    // Open shared object that contains PCM VM factory function
    std::string spec_lib_name = "lib" + std::string{pcm_algo_name} + "_spec.so";
    std::string vm_factory_fn_name =
        "__" + std::string{pcm_algo_name} + "_spec_get";

    auto symbol_result =
        pcm_vm::util::shared_symbol_open(spec_lib_name, vm_factory_fn_name);
    if (!symbol_result.has_value()) {
      throw std::runtime_error{"Failed to load PCM algorithm library: " +
                               spec_lib_name + " or find factory function."};
    }

    auto [so_handle, raw_fn_ptr] = symbol_result.value();

    // Store the shared object handle for cleanup
    auto spec_so_handle_ptr =
        std::shared_ptr<void>(so_handle, [](void* handle) {
          if (handle) {
            pcm_vm::util::shared_symbol_close(handle);
          }
        });

    // Cast the function pointer to the expected factory function type
    auto vm_factory_fn_ptr = reinterpret_cast<pcm_factory_fn_ptr>(raw_fn_ptr);
    std::cout << "Successfully opened PCM algorithm " << pcm_algo_name
              << std::endl;
    return {spec_so_handle_ptr, vm_factory_fn_ptr};
  }
};

}  // namespace uccl
