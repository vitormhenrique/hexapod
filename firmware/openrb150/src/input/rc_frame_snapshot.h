#pragma once

// ===========================================================================
// Latest complete RC-frame handoff (portable, heap-free).
//
// rcTask is the sole publisher and controlTask is the sole control consumer.
// The task adapter serializes publish/copy with a short critical section. This
// portable mailbox then stores one complete decoded frame at a time without a
// scheduler or Arduino dependency.
// ===========================================================================

#include <stdint.h>

#include "controller_bridge.h"
#include "crsf_parser.h"

namespace controller {

struct RcFrameSnapshot {
  ControllerCommand command{};
  crsf::RcStatus status{};
  uint32_t frame_sequence = 0;
  uint32_t published_ms = 0;
};

class RcFrameMailbox {
 public:
  void reset() {
    snapshot_ = RcFrameSnapshot{};
    sequence_ = 0;
    published_ = false;
  }

  uint32_t publish(const RcFrameSnapshot& snapshot) {
    snapshot_ = snapshot;
    published_ = true;
    return ++sequence_;
  }

  bool copy(RcFrameSnapshot& out, uint32_t* sequence = nullptr) const {
    if (!published_) return false;
    out = snapshot_;
    if (sequence != nullptr) *sequence = sequence_;
    return true;
  }

 private:
  RcFrameSnapshot snapshot_{};
  uint32_t sequence_ = 0;
  bool published_ = false;
};

}  // namespace controller