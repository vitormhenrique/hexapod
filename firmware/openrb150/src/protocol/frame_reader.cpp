#include "frame_reader.h"

namespace protocol {

void FrameReader::reset() {
  len_ = 0;
  overflow_ = false;
  ready_ = false;
}

bool FrameReader::push(uint8_t byte) {
  // If the previous push returned a frame, body_/len_ are still exposed to the
  // caller. Start a fresh frame now, before processing this byte.
  if (ready_) {
    ready_ = false;
    len_ = 0;
    overflow_ = false;
  }

  if (byte == 0x00) {
    // Delimiter: end of the current frame (if any data was collected).
    if (len_ > 0 && !overflow_) {
      ready_ = true;  // expose buf_/len_ until the next push()
      return true;
    }
    // Empty frame, leading delimiter, or a dropped overflow frame: restart.
    len_ = 0;
    overflow_ = false;
    return false;
  }

  if (overflow_) {
    // Already too long; keep dropping until the next delimiter.
    return false;
  }
  if (len_ >= sizeof(buf_)) {
    overflow_ = true;
    len_ = 0;
    return false;
  }
  buf_[len_++] = byte;
  return false;
}

}  // namespace protocol
