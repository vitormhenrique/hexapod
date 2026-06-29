// DYNAMIXEL maintenance command group (portable, host-tested).
// See dxl_job_api.h for the queue + gating contract.

#include "dxl_job_api.h"

namespace protocol {
namespace {

int32_t readI32(const uint8_t* p) {
  return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                              (static_cast<uint32_t>(p[1]) << 8) |
                              (static_cast<uint32_t>(p[2]) << 16) |
                              (static_cast<uint32_t>(p[3]) << 24));
}

}  // namespace

// --- DxlJobQueue -----------------------------------------------------------

uint8_t DxlJobQueue::nextJobId() {
  // Monotonic, skipping 0 (reserved for "no job"), wrapping 255 -> 1.
  ++next_id_;
  if (next_id_ == 0) next_id_ = 1;
  return next_id_;
}

void DxlJobQueue::reset() {
  slot_ = dxljob::Slot::Empty;
  job_id_ = 0;
  next_id_ = 0;
  req_ = DxlJobRequest{};
  result_ = DxlJobResult{};
}

bool DxlJobQueue::submit(const DxlJobRequest& req, uint8_t& job_id_out) {
  // Only free or already-collected slots accept a new job; a job in flight
  // (Pending/Running) must finish first.
  if (slot_ != dxljob::Slot::Empty && slot_ != dxljob::Slot::Done) {
    return false;
  }
  req_ = req;
  job_id_ = nextJobId();
  job_id_out = job_id_;
  // Publish payload before advancing the lifecycle so the consumer never reads
  // a half-written request.
  slot_ = dxljob::Slot::Pending;
  return true;
}

dxljob::Slot DxlJobQueue::poll(uint8_t job_id, DxlJobResult& out) const {
  if (job_id == 0 || job_id != job_id_) {
    return dxljob::Slot::Empty;  // unknown/stale job
  }
  const dxljob::Slot s = slot_;
  if (s == dxljob::Slot::Done) {
    out = result_;
  }
  return s;
}

bool DxlJobQueue::claim(DxlJobRequest& req, uint8_t& job_id) {
  if (slot_ != dxljob::Slot::Pending) {
    return false;
  }
  req = req_;
  job_id = job_id_;
  slot_ = dxljob::Slot::Running;
  return true;
}

void DxlJobQueue::complete(uint8_t job_id, dxljob::Code code,
                           const uint8_t* data, uint8_t len) {
  if (slot_ != dxljob::Slot::Running || job_id != job_id_) {
    return;
  }
  result_.code = code;
  if (len > dxljob::kMaxResult) len = dxljob::kMaxResult;
  result_.len = len;
  for (uint8_t i = 0; i < len; ++i) {
    result_.data[i] = data ? data[i] : 0;
  }
  // Publish the result before advancing the lifecycle.
  slot_ = dxljob::Slot::Done;
}

// --- DxlJobApi -------------------------------------------------------------

void DxlJobApi::reset() {
  queue_.reset();
  live_state_ = 0;
  lock_held_ = false;
  raw_register_enabled_ = false;
}

bool DxlJobApi::writeSubmit(DxlSubmit r, uint8_t job_id, dxljob::Slot slot,
                            uint8_t* out, size_t out_cap, uint16_t* out_len,
                            uint8_t* out_flags) const {
  if (out_cap < 3) {
    out[0] = static_cast<uint8_t>(DxlSubmit::BadRequest);
    *out_len = 1;
    *out_flags = 0x02;  // flag::kError
    return true;
  }
  out[0] = static_cast<uint8_t>(r);
  out[1] = job_id;
  out[2] = static_cast<uint8_t>(slot);
  *out_len = 3;
  // Rejected/Busy/BadRequest are reported with the error flag set so the host
  // does not mistake them for a queued job.
  *out_flags = (r == DxlSubmit::Accepted) ? 0x00 : 0x02;
  return true;
}

bool DxlJobApi::handle(uint8_t msg_id, const uint8_t* req, uint16_t req_len,
                       uint8_t* out, size_t out_cap, uint16_t* out_len,
                       uint8_t* out_flags) {
  if (!dxlmsg::isDxlMsg(msg_id)) return false;

  // DXL_GET_RESULT is a pure poll and is always allowed (even after the state
  // leaves maintenance) so the host can still collect an in-flight result.
  if (msg_id == dxlmsg::kGetResult) {
    if (req_len < 1) {
      out[0] = static_cast<uint8_t>(dxljob::Slot::Empty);
      out[1] = static_cast<uint8_t>(dxljob::Code::Ok);
      out[2] = 0;
      *out_len = 3;
      *out_flags = 0x02;  // flag::kError (malformed)
      return true;
    }
    const uint8_t job_id = req[0];
    DxlJobResult res;
    const dxljob::Slot s = queue_.poll(job_id, res);
    out[0] = static_cast<uint8_t>(s);
    if (s == dxljob::Slot::Done) {
      out[1] = static_cast<uint8_t>(res.code);
      uint8_t n = res.len;
      const size_t max_data = (out_cap > 3) ? (out_cap - 3) : 0;
      if (n > max_data) n = static_cast<uint8_t>(max_data);
      out[2] = n;
      for (uint8_t i = 0; i < n; ++i) out[3 + i] = res.data[i];
      *out_len = static_cast<uint16_t>(3 + n);
    } else {
      out[1] = static_cast<uint8_t>(dxljob::Code::Ok);
      out[2] = 0;
      *out_len = 3;
    }
    *out_flags = 0x00;
    return true;
  }

  // All submit commands are maintenance-gated.
  if (!gateOpen()) {
    return writeSubmit(DxlSubmit::Rejected, 0, dxljob::Slot::Empty, out, out_cap,
                       out_len, out_flags);
  }

  DxlJobRequest job;
  switch (msg_id) {
    case dxlmsg::kScan: {
      // [first_id, last_id]; default to the full unicast range if omitted.
      uint8_t first = 1;
      uint8_t last = 252;
      if (req_len >= 2) {
        first = req[0];
        last = req[1];
      } else if (req_len == 1) {
        first = req[0];
        last = req[0];
      }
      if (first == 0 || last < first) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::Scan;
      job.arg0 = first;
      job.arg1 = last;
      break;
    }
    case dxlmsg::kPing: {
      if (req_len < 1 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::Ping;
      job.arg0 = req[0];
      break;
    }
    case dxlmsg::kTorque: {
      if (req_len < 1) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::Torque;
      job.arg0 = (req[0] != 0) ? 1 : 0;
      break;
    }
    case dxlmsg::kGetServoProfile: {
      if (req_len < 1 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::GetProfile;
      job.arg0 = req[0];
      break;
    }
    case dxlmsg::kGetParam: {
      // [id, param]; reads one logical parameter from a single servo.
      if (req_len < 2 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::GetParam;
      job.arg0 = req[0];
      job.param = req[1];
      break;
    }
    case dxlmsg::kSetParam: {
      // [id, param, value(i32)]; writes one logical parameter (torque-off +
      // read-back verify handled by the executor for EEPROM params).
      if (req_len < 6 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::SetParam;
      job.arg0 = req[0];
      job.param = req[1];
      job.val_a = readI32(&req[2]);
      break;
    }
    case dxlmsg::kSetServoLimits: {
      // [id, min_tick(i32), max_tick(i32)]; table-aware joint travel bounds.
      if (req_len < 9 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      const int32_t lo = readI32(&req[1]);
      const int32_t hi = readI32(&req[5]);
      if (hi < lo) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::SetLimits;
      job.arg0 = req[0];
      job.val_a = lo;
      job.val_b = hi;
      break;
    }
    case dxlmsg::kReadRegister: {
      // [id, addr(u16), len]; raw read, expert-gated diagnostics only.
      if (!raw_register_enabled_) {
        return writeSubmit(DxlSubmit::Rejected, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      if (req_len < 4 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      const uint8_t len = req[3];
      if (len != 1 && len != 2 && len != 4) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::ReadReg;
      job.arg0 = req[0];
      job.val_a = static_cast<int32_t>(req[1] | (req[2] << 8));  // addr
      job.param = len;
      break;
    }
    case dxlmsg::kWriteRegister: {
      // [id, addr(u16), len, value(i32), flags]; flags bit0 = EEPROM region
      // (executor disables torque before writing). Expert-gated diagnostics.
      if (!raw_register_enabled_) {
        return writeSubmit(DxlSubmit::Rejected, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      if (req_len < 9 || req[0] == 0) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      const uint8_t len = req[3];
      if (len != 1 && len != 2 && len != 4) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::WriteReg;
      job.arg0 = req[0];
      job.val_a = static_cast<int32_t>(req[1] | (req[2] << 8));  // addr
      job.param = len;
      job.val_b = readI32(&req[4]);                              // value
      job.arg1 = req[8];                                         // flags
      break;
    }
    case dxlmsg::kPower: {
      // [on(0/1)]; toggles the DXL power FET. The executor (dxlTask) owns the
      // board power line, so the toggle runs there. Maintenance-gated like every
      // other submit (gateOpen() already enforced above), so power can only be
      // energized while the bench maintenance lock is held.
      if (req_len < 1) {
        return writeSubmit(DxlSubmit::BadRequest, 0, dxljob::Slot::Empty, out,
                           out_cap, out_len, out_flags);
      }
      job.type = dxljob::Type::Power;
      job.arg0 = (req[0] != 0) ? 1 : 0;
      break;
    }
    default:
      // An unassigned id inside the reserved DXL block: decline so the
      // dispatcher falls through to UnknownMsg.
      return false;
  }

  uint8_t job_id = 0;
  if (!queue_.submit(job, job_id)) {
    return writeSubmit(DxlSubmit::Busy, 0, queue_.slotState(), out, out_cap,
                       out_len, out_flags);
  }
  return writeSubmit(DxlSubmit::Accepted, job_id, dxljob::Slot::Pending, out,
                     out_cap, out_len, out_flags);
}

}  // namespace protocol
