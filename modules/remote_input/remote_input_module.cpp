/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include <modules/remote_input/remote_input_module.h>
#include <modules/thread_utils.h>
#include <protocols/remote_input/dr16_protocol.h>
#include <protocols/remote_input/vt03_protocol.h>
#include <protocols/remote_input/wfly_sbus_protocol.h>

LOG_MODULE_REGISTER(remote_input_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_remote_input_module_stack, 1024);

constexpr float kWflySbusMid = 1024.0f;
constexpr float kWflySbusScale = 670.0f;
constexpr uint16_t kWflySwitchLowMidThreshold = 689U;
constexpr uint16_t kWflySwitchMidHighThreshold = 1359U;

enum class WflySwitchPosition : uint8_t {
  kLow = 1,
  kMid = 2,
  kHigh = 3,
};

float ClampNormalized(float value) {
  if (value > 1.0f) {
    return 1.0f;
  }

  if (value < -1.0f) {
    return -1.0f;
  }

  return value;
}

float NormalizeWflyChannel(uint16_t raw) {
  return ClampNormalized((static_cast<float>(raw) - kWflySbusMid) /
                         kWflySbusScale);
}

float PositiveOnly(float value) {
  return (value > 0.0f) ? value : 0.0f;
}

WflySwitchPosition DecodeWflySwitch(uint16_t raw) {
  if (raw < kWflySwitchLowMidThreshold) {
    return WflySwitchPosition::kLow;
  }

  if (raw < kWflySwitchMidHighThreshold) {
    return WflySwitchPosition::kMid;
  }

  return WflySwitchPosition::kHigh;
}

void SetCommonActiveDefaults(channels::RemoteInputState *input) {
  input->run = true;
  input->robot_enable = true;
  input->leg_length = 0.0f;
  input->leg_length_delta = 0.0f;
  input->friction_speed = 0.0f;
  input->plucker = 0.0f;
  input->fast_spin = false;
}

void SetDisabled(channels::RemoteInputState *input) {
  input->chassis_x = 0.0f;
  input->chassis_rotate = 0.0f;
  input->yaw_angle = 0.0f;
  input->pitch_angle = 0.0f;
  input->leg_length = 0.0f;
  input->leg_length_delta = 0.0f;
  input->friction_speed = 0.0f;
  input->plucker = 0.0f;
  input->run = false;
  input->robot_enable = false;
  input->fast_spin = false;
}

} // namespace

namespace modules::remote_input {

int RemoteInputModule::Initialize() {
  publish_sequence_ = 0U;
  started_ = false;
  line_pos_ = 0U;
  line_buf_[0] = '\0';
  binary_len_ = 0U;
  return 0;
}

int RemoteInputModule::Start() {
  if (started_) {
    return 0;
  }

  ::modules::StartMemberThread<RemoteInputModule, &RemoteInputModule::RunLoop>(
      &thread_, g_remote_input_module_stack,
      K_THREAD_STACK_SIZEOF(g_remote_input_module_stack), this,
      K_PRIO_PREEMPT(8), "remote_input_module");

  started_ = true;
  return 0;
}

void RemoteInputModule::RunLoop() {
  LOG_INF("remote_input module started");

  while (true) {
    DecodeUartBytesFromRing();
  }
}

int RemoteInputModule::ParseLine(const char *line,
                                 channels::RemoteInputState *out) {
  if ((line == nullptr) || (out == nullptr)) {
    return -EINVAL;
  }

  char type[8] = {0};
  float chassis_x = 0.0f;
  float chassis_rotate = 0.0f;
  float yaw_angle = 0.0f;
  float pitch_angle = 0.0f;

  if (sscanf(line, "%7s %f %f %f %f", type, &chassis_x, &chassis_rotate,
             &yaw_angle, &pitch_angle) != 5) {
    return -EINVAL;
  }

  if (strcmp(type, "dr16") == 0) {
    out->source = channels::kRemoteInputDr16;
    SetCommonActiveDefaults(out);
    out->chassis_x = chassis_x;
    out->chassis_rotate = chassis_rotate;
    out->yaw_angle = yaw_angle;
    out->pitch_angle = pitch_angle;
  } else if (strcmp(type, "vt03") == 0) {
    out->source = channels::kRemoteInputVt03;
    SetCommonActiveDefaults(out);
    out->chassis_x = chassis_x;
    out->chassis_rotate = chassis_rotate;
    out->yaw_angle = yaw_angle;
    out->pitch_angle = pitch_angle;
  } else if (strcmp(type, "wfly") == 0) {
    out->source = channels::kRemoteInputWfly;
    SetCommonActiveDefaults(out);
    out->chassis_x = chassis_x;
    out->chassis_rotate = chassis_rotate;
    out->yaw_angle = yaw_angle;
    out->pitch_angle = pitch_angle;
  } else {
    return -EINVAL;
  }

  return 0;
}

void RemoteInputModule::ConsumeBinary(size_t bytes) {
  if ((bytes == 0U) || (binary_len_ == 0U)) {
    return;
  }

  if (bytes >= binary_len_) {
    binary_len_ = 0U;
    return;
  }

  memmove(binary_buf_, binary_buf_ + bytes, binary_len_ - bytes);
  binary_len_ -= bytes;
}

void RemoteInputModule::TryDecodeBinaryFrames() {
  while (binary_len_ > 0U) {
    channels::RemoteInputState input = {};

    if (binary_buf_[0] == protocols::remote_input::wfly_sbus::kStartByte) {
      if (binary_len_ < protocols::remote_input::wfly_sbus::kFrameLength) {
        break;
      }

      protocols::remote_input::wfly_sbus::WflySbusFrame wfly_frame = {};
      if (protocols::remote_input::wfly_sbus::DecodeFrame(
              binary_buf_, binary_len_, &wfly_frame)) {
        input.source = channels::kRemoteInputWfly;
        SetDisabled(&input);

        if (!wfly_frame.frame_lost && !wfly_frame.failsafe) {
          const float right_x = NormalizeWflyChannel(wfly_frame.channels[0]);
          const float right_y = NormalizeWflyChannel(wfly_frame.channels[1]);
          const float left_y = NormalizeWflyChannel(wfly_frame.channels[2]);
          const float left_x = NormalizeWflyChannel(wfly_frame.channels[3]);
          const WflySwitchPosition ch4 =
              DecodeWflySwitch(wfly_frame.channels[4]);
          const WflySwitchPosition ch5 =
              DecodeWflySwitch(wfly_frame.channels[5]);

          if (ch4 == WflySwitchPosition::kLow) {
            if (ch5 == WflySwitchPosition::kMid) {
              input.run = true;
              input.robot_enable = true;
              input.chassis_x = left_y;
              input.chassis_rotate = left_x;
              input.yaw_angle = right_x;
              input.pitch_angle = right_y;
            } else if (ch5 == WflySwitchPosition::kHigh) {
              input.run = true;
              input.robot_enable = true;
              input.friction_speed = PositiveOnly(left_y);
              input.plucker = right_y;
            }
          } else if (ch4 == WflySwitchPosition::kMid) {
            input.run = true;
            input.robot_enable = true;
            input.chassis_x = left_y;
            input.chassis_rotate = left_x;
            input.leg_length_delta = right_y;
            input.leg_length = input.leg_length_delta;
          }
        }
        PublishRemoteState(&input);
        ConsumeBinary(protocols::remote_input::wfly_sbus::kFrameLength);
        continue;
      }

      ConsumeBinary(1U);
      continue;
    }

    if ((binary_len_ >= protocols::remote_input::vt03::kRemoteFrameLength) &&
        (binary_buf_[0] == 0xa9U) && (binary_buf_[1] == 0x53U)) {
      protocols::remote_input::vt03::Vt03Frame vt03_frame = {};
      if (protocols::remote_input::vt03::DecodeRemoteFrame(
              binary_buf_, binary_len_, &vt03_frame)) {
        input.source = channels::kRemoteInputVt03;
        SetCommonActiveDefaults(&input);
        input.chassis_x = vt03_frame.left_y;
        input.chassis_rotate = vt03_frame.left_x;
        input.yaw_angle = vt03_frame.right_x;
        input.pitch_angle = vt03_frame.right_y;
        PublishRemoteState(&input);
        ConsumeBinary(protocols::remote_input::vt03::kRemoteFrameLength);
        continue;
      }
      ConsumeBinary(1U);
      continue;
    }

    if ((binary_len_ >= protocols::remote_input::vt03::kCustomFrameLength) &&
        (binary_buf_[0] == 0xa5U)) {
      protocols::remote_input::vt03::Vt03CustomFrame custom = {};
      if (protocols::remote_input::vt03::DecodeCustomFrame(
              binary_buf_, binary_len_, &custom)) {
        input.source = channels::kRemoteInputVt03;
        SetCommonActiveDefaults(&input);
        input.yaw_angle = custom.joystick_x;
        input.pitch_angle = custom.joystick_y;
        PublishRemoteState(&input);
        ConsumeBinary(protocols::remote_input::vt03::kCustomFrameLength);
        continue;
      }
    }

    if (binary_len_ >= protocols::remote_input::dr16::kFrameLength) {
      protocols::remote_input::dr16::Dr16Frame dr16_frame = {};
      if (protocols::remote_input::dr16::DecodeFrame(binary_buf_, binary_len_,
                                                     &dr16_frame)) {
        input.source = channels::kRemoteInputDr16;
        SetCommonActiveDefaults(&input);
        input.chassis_x = dr16_frame.left_stick_y;
        input.chassis_rotate = dr16_frame.left_stick_x;
        input.yaw_angle = dr16_frame.right_stick_x;
        input.pitch_angle = dr16_frame.right_stick_y;
        PublishRemoteState(&input);
        ConsumeBinary(protocols::remote_input::dr16::kFrameLength);
        continue;
      }
    }

    if (binary_len_ < protocols::remote_input::dr16::kFrameLength) {
      break;
    }

    ConsumeBinary(1U);
  }
}

void RemoteInputModule::DecodeUartBytesFromRing() {
  while (true) {
    uint8_t data[64] = {};
    uint32_t read_len =
        ring_buf_get(&channels::uart_raw_frame_queue::remote_input_ring_buf,
                     data, static_cast<uint32_t>(sizeof(data)));
    if (read_len == 0U) {
      if (k_sem_take(&channels::uart_raw_frame_queue::remote_input_sem,
                     K_MSEC(20)) != 0) {
        break;
      }

      read_len =
          ring_buf_get(&channels::uart_raw_frame_queue::remote_input_ring_buf,
                       data, static_cast<uint32_t>(sizeof(data)));
    }

    const size_t len = static_cast<size_t>(read_len);
    if (len == 0U) {
      break;
    }

    for (size_t i = 0U; i < len; ++i) {
      const uint8_t byte = data[i];

      if (binary_len_ < kBinaryBufSize) {
        binary_buf_[binary_len_++] = byte;
      } else {
        memmove(binary_buf_, binary_buf_ + 1, kBinaryBufSize - 1U);
        binary_buf_[kBinaryBufSize - 1U] = byte;
      }

      TryDecodeBinaryFrames();

      if (byte == '\r') {
        continue;
      }

      if (byte == '\n') {
        line_buf_[line_pos_] = '\0';
        if (line_pos_ > 0U) {
          channels::RemoteInputState input = {};
          if (ParseLine(line_buf_, &input) == 0) {
            PublishRemoteState(&input);
          }
        }
        line_pos_ = 0U;
        line_buf_[0] = '\0';
        continue;
      }

      if (isprint(byte) == 0) {
        continue;
      }

      if (line_pos_ < (kLineBufSize - 1U)) {
        line_buf_[line_pos_++] = static_cast<char>(byte);
      } else {
        line_pos_ = 0U;
        line_buf_[0] = '\0';
      }
    }
  }
}

void RemoteInputModule::PublishRemoteState(channels::RemoteInputState *input) {
  if (input == nullptr) {
    return;
  }

  input->sequence = ++publish_sequence_;
  latest_remote_state.write(*input);
}

} // namespace modules::remote_input
