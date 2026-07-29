#include "rc_input_conditioner.h"

#include <math.h>

namespace controller {

namespace {

constexpr float kX255Scale = 1.0f / 255.0f;

}  // namespace

void RcInputConditioner::configure(
    const config::RcInputCalibration& calibration) {
  if (!config::validateRcInputCalibration(calibration)) return;
  calibration_ = calibration;
  reset();
}

void RcInputConditioner::reset() {
  for (uint8_t index = 0; index < config::kNumRcAnalogInputs; ++index) {
    channels_[index] = ChannelState{};
  }
}

bool RcInputConditioner::setMode(InputFilterMode mode,
                                 bool diagnostic_mode_allowed) {
  if (mode == InputFilterMode::NoneDiagnostic && !diagnostic_mode_allowed) {
    return false;
  }
  if (mode == mode_) return true;
  mode_ = mode;
  // Seed every median history from the current filtered result. A subsequent
  // Median3+EMA sample therefore starts from the old output, not raw input.
  for (uint8_t index = 0; index < config::kNumRcAnalogInputs; ++index) {
    ChannelState& state = channels_[index];
    for (uint8_t sample = 0; sample < 3; ++sample) {
      state.median[sample] = state.output;
    }
    state.median_count = state.initialized ? 3 : 0;
    state.median_next = 0;
  }
  return true;
}

float RcInputConditioner::update(uint8_t source, float input, uint32_t dt_ms) {
  if (source == 0 || source > config::kNumRcAnalogInputs) return 0.0f;
  const config::RcChannelCalibration* channel = find(source);
  if (channel == nullptr) return 0.0f;
  const uint8_t index = static_cast<uint8_t>(source - 1u);
  ChannelState& state = channels_[index];
  const bool centered = channel->type ==
      static_cast<uint8_t>(config::RcChannelType::CenteredAnalog);
  input = clamp(input, centered ? -1.0f : 0.0f, 1.0f);

  if (!state.initialized) {
    state.output = input;
    for (uint8_t sample = 0; sample < 3; ++sample) state.median[sample] = input;
    state.median_count = 3;
    state.initialized = true;
    return state.output;
  }

  if (channel->type ==
      static_cast<uint8_t>(config::RcChannelType::RelativeEncoder)) {
    // Encoder accumulation already resolves wraps and should not be delayed.
    state.output = input;
    return state.output;
  }

  if (mode_ == InputFilterMode::NoneDiagnostic) {
    state.output = input;
    return state.output;
  }

  float filtered_input = input;
  if (mode_ == InputFilterMode::Median3Ema) {
    filtered_input = medianInput(state, input);
  }

  if (channel->filter_tau_ms == 0 || dt_ms == 0) {
    if (channel->filter_tau_ms == 0) state.output = filtered_input;
    return state.output;
  }
  const float tau_ms = static_cast<float>(channel->filter_tau_ms);
  const float alpha = 1.0f - expf(-static_cast<float>(dt_ms) / tau_ms);
  state.output += (filtered_input - state.output) * clamp(alpha, 0.0f, 1.0f);
  return clamp(state.output, centered ? -1.0f : 0.0f, 1.0f);
}

bool RcInputConditioner::isCentered(uint8_t source) const {
  const config::RcChannelCalibration* channel = find(source);
  return channel != nullptr && channel->type ==
      static_cast<uint8_t>(config::RcChannelType::CenteredAnalog);
}

float RcInputConditioner::deadband(uint8_t source) const {
  const config::RcChannelCalibration* channel = find(source);
  return channel == nullptr ? 0.0f
                            : static_cast<float>(channel->deadband_x255) *
                                  kX255Scale;
}

float RcInputConditioner::expo(uint8_t source) const {
  const config::RcChannelCalibration* channel = find(source);
  return channel == nullptr ? 0.0f
                            : static_cast<float>(channel->expo_x255) *
                                  kX255Scale;
}

float RcInputConditioner::applyDeadband(float value, float deadband) {
  deadband = clamp(deadband, 0.0f, 0.999f);
  const float magnitude = value < 0.0f ? -value : value;
  if (magnitude <= deadband) return 0.0f;
  const float scaled = (magnitude - deadband) / (1.0f - deadband);
  return value < 0.0f ? -scaled : scaled;
}

float RcInputConditioner::applyExpo(float value, float expo) {
  expo = clamp(expo, 0.0f, 1.0f);
  return (1.0f - expo) * value + expo * value * value * value;
}

const config::RcChannelCalibration* RcInputConditioner::find(
    uint8_t source) const {
  for (uint8_t index = 0; index < config::kNumRcAnalogInputs; ++index) {
    if (calibration_.channels[index].source == source) {
      return &calibration_.channels[index];
    }
  }
  return nullptr;
}

float RcInputConditioner::median3(float a, float b, float c) {
  if (a > b) {
    const float swap = a;
    a = b;
    b = swap;
  }
  if (b > c) {
    const float swap = b;
    b = c;
    c = swap;
  }
  if (a > b) b = a;
  return b;
}

float RcInputConditioner::clamp(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float RcInputConditioner::medianInput(ChannelState& state, float input) {
  state.median[state.median_next] = input;
  state.median_next = static_cast<uint8_t>((state.median_next + 1u) % 3u);
  if (state.median_count < 3) ++state.median_count;
  return median3(state.median[0], state.median[1], state.median[2]);
}

uint8_t RcTriSwitchDebouncer::update(uint8_t raw, uint32_t now_ms,
                                     uint16_t debounce_ms) {
  if (raw > 2) raw = 1;
  if (!initialized_) {
    stable_ = raw;
    candidate_ = raw;
    candidate_since_ms_ = now_ms;
    initialized_ = true;
    return stable_;
  }
  if (raw != candidate_) {
    candidate_ = raw;
    candidate_since_ms_ = now_ms;
  }
  if (candidate_ != stable_ &&
      (now_ms - candidate_since_ms_) >= debounce_ms) {
    stable_ = candidate_;
  }
  return stable_;
}

}  // namespace controller