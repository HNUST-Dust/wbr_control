/* SPDX-License-Identifier: Apache-2.0 */

#include <algorithm>
#include <cmath>
#include <errno.h>

#include <zephyr/logging/log.h>

#include <channels/serialservo_send_raw.hpp>
#include <modules/gimbal/gimbal_module.h>
#include <modules/thread_utils.h>
#include <platform/drivers/devices/actuators/serial_servo.h>

LOG_MODULE_REGISTER(gimbal_module, LOG_LEVEL_INF);

namespace {

K_THREAD_STACK_DEFINE(g_gimbal_module_stack, 1024);
constexpr bool kBaselineTraceEnabled = false;
constexpr uint32_t kBaselineTracePeriod = 500U;
constexpr uint8_t kSerialServoFrameHead = 0x55U;

enum class ServoTxSlot : uint8_t {
  kYaw,
  kPitch,
};

uint16_t ServoAngleToRaw(float degrees) {
  float clamped = degrees;
  if (clamped < 0.0f) {
    clamped = 0.0f;
  }
  if (clamped > 240.0f) {
    clamped = 240.0f;
  }

  return static_cast<uint16_t>(clamped * (1000.0f / 240.0f) + 0.5f);
}

int WriteServoPacket(ServoTxSlot slot, uint8_t id, uint8_t cmd,
                     const uint8_t *params, uint8_t params_len) {
  if ((params == nullptr) && (params_len != 0U)) {
    return -EINVAL;
  }

  SerialServoSendRawFrame value = {};
  value.valid = 1U;
  const uint8_t length = static_cast<uint8_t>(params_len + 3U);
  value.data[0] = kSerialServoFrameHead;
  value.data[1] = kSerialServoFrameHead;
  value.data[2] = id;
  value.data[3] = length;
  value.data[4] = cmd;

  uint16_t sum = static_cast<uint16_t>(id + length + cmd);
  for (uint8_t i = 0U; i < params_len; ++i) {
    value.data[5U + i] = params[i];
    sum = static_cast<uint16_t>(sum + params[i]);
  }

  value.data[5U + params_len] = static_cast<uint8_t>(~(sum & 0xffU));
  value.len = static_cast<uint8_t>(6U + params_len);
  if (slot == ServoTxSlot::kYaw) {
    yaw_servo_send_raw.write(value);
  } else {
    pitch_servo_send_raw.write(value);
  }

  return 0;
}

int WriteServoStop(ServoTxSlot slot, uint8_t id) {
  return WriteServoPacket(slot, id, 12U, nullptr, 0U);
}

int WriteServoMove(ServoTxSlot slot, uint8_t id, float angle_deg,
                   uint16_t time_ms) {
  const uint16_t raw = ServoAngleToRaw(angle_deg);
  const uint8_t params[4] = {
      static_cast<uint8_t>(raw & 0xffU),
      static_cast<uint8_t>((raw >> 8) & 0xffU),
      static_cast<uint8_t>(time_ms & 0xffU),
      static_cast<uint8_t>((time_ms >> 8) & 0xffU),
  };
  return WriteServoPacket(slot, id, 1U, params, sizeof(params));
}

} // namespace

namespace modules::gimbal {

int GimbalModule::Initialize() {
  started_ = false;
  servo_ready_ =
      (platform::drivers::devices::actuators::serial_servo::Initialize() == 0);
  servo_stopped_ = false;
  yaw_servo_online_ = false;
  pitch_servo_online_ = false;
  command_enable_ = 0U;
  idle_ticks_ = 0U;
  yaw_servo_id_ = kDefaultYawServoId;
  pitch_servo_id_ = kDefaultPitchServoId;
  yaw_angle_deg_ = 0.0f;
  pitch_angle_deg_ = 0.0f;
  yaw_last_sent_angle_deg_ = 0.0f;
  pitch_last_sent_angle_deg_ = 0.0f;
  yaw_sent_once_ = false;
  pitch_sent_once_ = false;
  next_servo_send_ms_ = 0;
  state_sequence_ = 0U;
  last_remote_sequence_ = 0U;

  if (servo_ready_) {
    uint8_t found = 0U;
    if (platform::drivers::devices::actuators::serial_servo::ReadId(
            yaw_servo_id_, &found, 120U) == 0) {
      yaw_servo_id_ = found;
      yaw_servo_online_ = true;
    }
    if (platform::drivers::devices::actuators::serial_servo::ReadId(
            pitch_servo_id_, &found, 120U) == 0) {
      pitch_servo_id_ = found;
      pitch_servo_online_ = true;
    }

    if (!yaw_servo_online_ && !pitch_servo_online_) {
      if (platform::drivers::devices::actuators::serial_servo::ReadId(
              0xFEU, &found, 200U) == 0) {
        yaw_servo_id_ = found;
        yaw_servo_online_ = true;
      }
    }
  }
  return 0;
}

int GimbalModule::Start() {
  if (started_) {
    return 0;
  }

  ::modules::StartMemberThread<GimbalModule, &GimbalModule::RunLoop>(
      &thread_, g_gimbal_module_stack,
      K_THREAD_STACK_SIZEOF(g_gimbal_module_stack), this, K_PRIO_PREEMPT(8),
      "gimbal_module");
  started_ = true;
  return 0;
}

void GimbalModule::SetYawAngle(float degrees) {
  yaw_angle_deg_ = std::clamp(degrees, kYawMin, kYawMax);
}

void GimbalModule::SetPitchAngle(float degrees) {
  pitch_angle_deg_ = std::clamp(degrees, kPitchMin, kPitchMax);
}

void GimbalModule::HandleRemoteState(const channels::RemoteInputState &state) {
  idle_ticks_ = 0U;

  if ((state.source == channels::kRemoteInputUnknown) || !state.robot_enable) {
    command_enable_ = 0U;
    return;
  }

  command_enable_ = 1U;
  SetYawAngle(yaw_angle_deg_ + (state.yaw_angle * 1.2f));
  SetPitchAngle(pitch_angle_deg_ + (state.pitch_angle * 0.8f));
}

void GimbalModule::RunLoop() {
  LOG_INF("gimbal module started (servo_ready=%d)", servo_ready_ ? 1 : 0);

  while (true) {
    channels::RemoteInputState remote_state = {};
    latest_remote_state.read(remote_state);
    if ((remote_state.sequence != 0U) &&
        (remote_state.sequence != last_remote_sequence_)) {
      last_remote_sequence_ = remote_state.sequence;
      HandleRemoteState(remote_state);
    } else {
      ++idle_ticks_;
      if (idle_ticks_ > kNoCommandStopTicks) {
        command_enable_ = 0U;
      }
    }

    if (servo_ready_) {
      if (command_enable_ == 0U) {
        if (!servo_stopped_) {
          if (yaw_servo_online_) {
            (void)WriteServoStop(ServoTxSlot::kYaw, yaw_servo_id_);
          }
          if (pitch_servo_online_) {
            (void)WriteServoStop(ServoTxSlot::kPitch, pitch_servo_id_);
          }
          servo_stopped_ = true;
          yaw_sent_once_ = false;
          pitch_sent_once_ = false;
        }
      } else {
        const int64_t now_ms = k_uptime_get();
        if (now_ms >= next_servo_send_ms_) {
          const float yaw_servo_angle = yaw_angle_deg_ + 120.0f;
          const float pitch_servo_angle = pitch_angle_deg_ + 90.0f;

          if (yaw_servo_online_) {
            if (!yaw_sent_once_ ||
                (std::fabs(yaw_servo_angle - yaw_last_sent_angle_deg_) >=
                 kServoAngleEpsilonDeg)) {
              (void)WriteServoMove(ServoTxSlot::kYaw, yaw_servo_id_,
                                   yaw_servo_angle, kServoMoveTimeMs);
              yaw_last_sent_angle_deg_ = yaw_servo_angle;
              yaw_sent_once_ = true;
            }
          }

          if (pitch_servo_online_) {
            if (!pitch_sent_once_ ||
                (std::fabs(pitch_servo_angle - pitch_last_sent_angle_deg_) >=
                 kServoAngleEpsilonDeg)) {
              (void)WriteServoMove(ServoTxSlot::kPitch, pitch_servo_id_,
                                   pitch_servo_angle, kServoMoveTimeMs);
              pitch_last_sent_angle_deg_ = pitch_servo_angle;
              pitch_sent_once_ = true;
            }
          }

          next_servo_send_ms_ = now_ms + kServoUpdatePeriodMs;
        }
        servo_stopped_ = false;
      }
    }

    ++state_sequence_;

    if (kBaselineTraceEnabled &&
        ((state_sequence_ % kBaselineTracePeriod) == 0U)) {
      LOG_INF("baseline yaw=%.2f pitch=%.2f en=%u idle=%u yid=%u pid=%u yon=%u pon=%u",
              static_cast<double>(yaw_angle_deg_),
              static_cast<double>(pitch_angle_deg_),
              static_cast<unsigned int>(command_enable_),
              static_cast<unsigned int>(idle_ticks_),
              static_cast<unsigned int>(yaw_servo_id_),
              static_cast<unsigned int>(pitch_servo_id_),
              yaw_servo_online_ ? 1U : 0U, pitch_servo_online_ ? 1U : 0U);
    }
    k_sleep(K_MSEC(2));
  }
}

} // namespace modules::gimbal
