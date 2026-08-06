// Deduplicated error journal implementation. See error_journal.h.

#include "error_journal.h"

namespace safety {

namespace {

// Unsigned-safe elapsed time (handles the 32-bit millisecond wrap).
inline uint32_t elapsed(uint32_t now_ms, uint32_t then_ms) {
  return now_ms - then_ms;
}

}  // namespace

void ErrorJournal::reset() {
  for (uint8_t i = 0; i < kMaxErrorEntries; ++i) {
    entries_[i] = ErrorEntry{};
    pending_[i] = false;
  }
  latest_ = ErrorEntry{};
  count_ = 0;
  pending_count_ = 0;
  next_sequence_ = 1;
  suppressed_ = 0;
}

int8_t ErrorJournal::find(ErrorCode code, uint8_t detail) const {
  for (uint8_t i = 0; i < count_; ++i) {
    if (entries_[i].code == code && entries_[i].detail == detail) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

int8_t ErrorJournal::recycleSlot(ErrorSeverity incoming,
                                 uint32_t now_ms) const {
  int8_t best = -1;
  for (uint8_t i = 0; i < count_; ++i) {
    // Never evict something that has not been transmitted yet unless the
    // incoming error is strictly more severe; losing an unsent critical fault
    // to a chatty warning would defeat the point of the journal.
    if (pending_[i] && entries_[i].severity >= incoming) continue;
    if (best < 0) {
      best = static_cast<int8_t>(i);
      continue;
    }
    const ErrorEntry& candidate = entries_[i];
    const ErrorEntry& current = entries_[best];
    if (candidate.severity != current.severity) {
      if (candidate.severity < current.severity) best = static_cast<int8_t>(i);
    } else if (elapsed(now_ms, candidate.last_ms) >
               elapsed(now_ms, current.last_ms)) {
      best = static_cast<int8_t>(i);
    }
  }
  return best;
}

bool ErrorJournal::note(ErrorCode code, uint8_t detail,
                        ErrorSeverity severity, uint32_t now_ms) {
  if (code == ErrorCode::None) return false;

  int8_t slot = find(code, detail);
  bool announce = false;

  if (slot < 0) {
    if (count_ < kMaxErrorEntries) {
      slot = static_cast<int8_t>(count_++);
    } else {
      slot = recycleSlot(severity, now_ms);
      if (slot < 0) {
        // Table full of unsent, at-least-as-severe errors: count the drop but
        // never block the producer.
        ++suppressed_;
        return false;
      }
      if (pending_[slot]) {
        pending_[slot] = false;
        if (pending_count_ > 0) --pending_count_;
      }
    }
    entries_[slot] = ErrorEntry{};
    entries_[slot].code = code;
    entries_[slot].detail = detail;
    entries_[slot].first_ms = now_ms;
    announce = true;
  } else if (elapsed(now_ms, entries_[slot].last_ms) >= kClearAfterMs) {
    // Quiet long enough to count as resolved: restart the incident.
    entries_[slot].count = 0;
    entries_[slot].first_ms = now_ms;
    announce = true;
  } else if (elapsed(now_ms, entries_[slot].first_ms) >= kRepeatIntervalMs &&
             !pending_[slot]) {
    // Still failing: re-announce once per interval with the running count so
    // the operator sees severity without the transport seeing a flood.
    entries_[slot].first_ms = now_ms;
    announce = true;
  }

  ErrorEntry& entry = entries_[slot];
  entry.severity = severity;
  entry.last_ms = now_ms;
  if (entry.count < 0xFFFFu) ++entry.count;

  if (!announce) {
    ++suppressed_;
    return false;
  }

  entry.sequence = next_sequence_++;
  if (next_sequence_ == 0) next_sequence_ = 1;  // 0 means "never announced"
  if (!pending_[slot]) {
    pending_[slot] = true;
    ++pending_count_;
  }
  return true;
}

bool ErrorJournal::takePending(ErrorEntry* out) {
  if (pending_count_ == 0) return false;
  int8_t best = -1;
  for (uint8_t i = 0; i < count_; ++i) {
    if (!pending_[i]) continue;
    if (best < 0) {
      best = static_cast<int8_t>(i);
      continue;
    }
    const ErrorEntry& candidate = entries_[i];
    const ErrorEntry& current = entries_[best];
    if (candidate.severity > current.severity) {
      best = static_cast<int8_t>(i);
    } else if (candidate.severity == current.severity &&
               candidate.first_ms < current.first_ms) {
      best = static_cast<int8_t>(i);
    }
  }
  if (best < 0) {
    pending_count_ = 0;
    return false;
  }
  pending_[best] = false;
  --pending_count_;
  latest_ = entries_[best];
  if (out != nullptr) *out = entries_[best];
  return true;
}

}  // namespace safety
