// Copyright 2026 Dynamixel Hardware contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using Bytes = std::vector<uint8_t>;

uint16_t word(const Bytes & bytes, size_t offset)
{
  return bytes.at(offset) | (bytes.at(offset + 1) << 8);
}

uint16_t crc(const Bytes & bytes)
{
  uint16_t result = 0;
  for (const auto byte : bytes) {
    result ^= static_cast<uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      result = (result & 0x8000) ? (result << 1) ^ 0x8005 : result << 1;
    }
  }
  return result;
}

// Two XL430-W250 servos. Only a pty is opened: never a physical serial device.
// Workbench probes Protocol 1 first (ignored), then Protocol 2. Init writes
// Operating_Mode; reads use the X-series block at 126..137. Sync writes have
// no status reply, just as on a real bus.
class ServoBus
{
public:
  struct Write
  {
    uint8_t id;
    uint16_t address;
    Bytes data;
  };

  ServoBus()
  {
    char name[128]{};
    if (openpty(&master_, &slave_, name, nullptr, nullptr) != 0) {
      throw std::runtime_error("openpty failed");
    }
    path_ = name;
    for (auto & table : tables_) {
      table[0] = 1060 & 0xff;
      table[1] = 1060 >> 8;
      table[11] = 3;
    }
    set_state(1, 2048, 10, 20);
    set_state(2, 1024, -5, -10);
    thread_ = std::thread([this]() {run();});
  }

  ~ServoBus()
  {
    stop_ = true;
    thread_.join();
    close(master_);
    close(slave_);
  }

  const std::string & path() const {return path_;}

  void set_state(uint8_t id, int32_t position, int32_t velocity, int16_t effort)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto & table = tables_.at(id - 1);
    for (int byte = 0; byte < 4; ++byte) {
      table[132 + byte] = static_cast<uint32_t>(position) >> (8 * byte);
      table[128 + byte] = static_cast<uint32_t>(velocity) >> (8 * byte);
    }
    table[126] = effort & 0xff;
    table[127] = static_cast<uint16_t>(effort) >> 8;
  }

  void drop_reads(int count, uint8_t only_id = 0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    drop_reads_ = count;
    drop_id_ = only_id;
  }

  std::vector<Write> writes()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

  bool wait_for_writes(uint16_t address, size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, std::chrono::seconds(1), [this, address, count]() {
               return std::count_if(writes_.begin(), writes_.end(), [address](const Write & write) {
                        return write.address == address;
      }) >= static_cast<int>(count);
    });
  }

  int read_count()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return read_count_;
  }

  bool healthy() const {return healthy_;}

  void reject_torque_enable()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reject_torque_enable_ = true;
  }

private:
  void reply(uint8_t id, const Bytes & data, uint8_t error = 0)
  {
    Bytes body{0x55, error};
    body.insert(body.end(), data.begin(), data.end());
    Bytes stuffed;
    for (const auto byte : body) {
      stuffed.push_back(byte);
      const size_t n = stuffed.size();
      if (n >= 3 && stuffed[n - 3] == 0xff && stuffed[n - 2] == 0xff && byte == 0xfd) {
        stuffed.push_back(0xfd);
      }
    }
    const auto length = stuffed.size() + 2;
    Bytes packet{0xff, 0xff, 0xfd, 0, id,
      static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8)};
    packet.insert(packet.end(), stuffed.begin(), stuffed.end());
    const auto checksum = crc(packet);
    packet.push_back(checksum & 0xff);
    packet.push_back(checksum >> 8);
    if (::write(master_, packet.data(), packet.size()) != static_cast<ssize_t>(packet.size())) {
      healthy_ = false;
    }
  }

  Bytes read_register(uint8_t id, uint16_t address, uint16_t length)
  {
    const auto & table = tables_.at(id - 1);
    if (address + length > table.size()) {
      throw std::runtime_error("read outside simulated control table");
    }
    return Bytes(table.begin() + address, table.begin() + address + length);
  }

  void write_register(uint8_t id, uint16_t address, const Bytes & data)
  {
    auto & table = tables_.at(id - 1);
    if (address + data.size() > table.size()) {
      throw std::runtime_error("write outside simulated control table");
    }
    std::copy(data.begin(), data.end(), table.begin() + address);
    writes_.push_back({id, address, data});
    changed_.notify_all();
  }

  void handle(uint8_t id, const Bytes & body)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    switch (body.at(0)) {
      case 0x01:  // Ping: model 1060, firmware 1.
        reply(id, {0x24, 0x04, 1});
        break;
      case 0x02:
        reply(id, read_register(id, word(body, 1), word(body, 3)));
        break;
      case 0x03:
        if (reject_torque_enable_ && word(body, 1) == 64 && body.at(3) == 1) {
          reply(id, {}, 0x04);  // Data range error, no write applied.
          break;
        }
        write_register(id, word(body, 1), Bytes(body.begin() + 3, body.end()));
        reply(id, {});
        break;
      case 0x82: {
          ++read_count_;
          const bool drop = drop_reads_ > 0;
          if (drop) {
            --drop_reads_;
          }
          for (size_t i = 5; i < body.size(); ++i) {
            if (!drop || (drop_id_ != 0 && drop_id_ != body[i])) {
              reply(body[i], read_register(body[i], word(body, 1), word(body, 3)));
            }
          }
          break;
        }
      case 0x83: {
          const auto length = word(body, 3);
          for (size_t i = 5; i < body.size(); i += length + 1) {
            write_register(
            body[i], word(body, 1), Bytes(body.begin() + i + 1, body.begin() + i + 1 + length));
          }
          break;
        }
      default:
        throw std::runtime_error("unexpected servo instruction");
    }
  }

  void run()
  {
    try {
      Bytes input;
      const Bytes header{0xff, 0xff, 0xfd, 0};
      while (!stop_) {
        pollfd descriptor{master_, POLLIN, 0};
        if (poll(&descriptor, 1, 10) <= 0 || !(descriptor.revents & POLLIN)) {
          continue;
        }
        uint8_t buffer[1024];
        const auto received = ::read(master_, buffer, sizeof(buffer));
        if (received <= 0) {
          continue;
        }
        input.insert(input.end(), buffer, buffer + received);
        while (input.size() >= 7) {
          if (!std::equal(header.begin(), header.end(), input.begin())) {
            input.erase(input.begin());
            continue;
          }
          const size_t size = 7 + word(input, 5);
          if (input.size() < size) {
            break;
          }
          const Bytes packet(input.begin(), input.begin() + size - 2);
          if (crc(packet) != word(input, size - 2)) {
            throw std::runtime_error("invalid instruction CRC");
          }
          Bytes body;
          for (size_t i = 7; i < size - 2; ++i) {
            body.push_back(input[i]);
            const size_t n = body.size();
            if (n >= 3 && body[n - 3] == 0xff && body[n - 2] == 0xff && body[n - 1] == 0xfd &&
              i + 1 < size - 2 && input[i + 1] == 0xfd)
            {
              ++i;
            }
          }
          handle(input[4], body);
          input.erase(input.begin(), input.begin() + size);
        }
      }
    } catch (const std::exception &) {
      healthy_ = false;
    }
  }

  int master_{-1};
  int slave_{-1};
  std::string path_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> healthy_{true};
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::array<std::array<uint8_t, 256>, 2> tables_{};
  std::vector<Write> writes_;
  int drop_reads_{0};
  uint8_t drop_id_{0};
  int read_count_{0};
  bool reject_torque_enable_{false};
};

class DynamixelHardwareTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override {EXPECT_TRUE(bus_.healthy());}

  CallbackReturn init(
    const std::string & parameters = "", const std::string & joint_parameters = "")
  {
    std::string urdf =
      "<robot name='test'><link name='base'/><link name='link1'/><link name='link2'/>"
      "<joint name='joint1' type='continuous'><parent link='base'/><child link='link1'/></joint>"
      "<joint name='joint2' type='continuous'><parent link='link1'/><child link='link2'/></joint>"
      "<ros2_control name='servos' type='system'>"
      "<hardware><plugin>dynamixel_hardware/DynamixelHardware</plugin>"
      "<param name='usb_port'>" + bus_.path() + "</param>"
      "<param name='baud_rate'>1000000</param>" + parameters + "</hardware>";
    for (int id = 1; id <= 2; ++id) {
      urdf += "<joint name='joint" + std::to_string(id) + "'><param name='id'>" +
        std::to_string(id) + "</param>" + joint_parameters + "<command_interface name='position'/>"
        "<command_interface name='velocity'/><state_interface name='position'/>"
        "<state_interface name='velocity'/><state_interface name='effort'/></joint>";
    }
    urdf += "</ros2_control></robot>";
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf).at(0);
    params.clock = std::make_shared<rclcpp::Clock>();
    return hardware_.init(params);
  }

  return_type read_ms(int milliseconds)
  {
    return hardware_.read(rclcpp::Time{},
        rclcpp::Duration::from_nanoseconds(milliseconds * 1000000LL));
  }

  std::vector<double> state()
  {
    std::vector<double> result;
    for (const auto & interface : hardware_.export_state_interfaces()) {
      result.push_back(interface.get_optional().value());
    }
    return result;
  }

  CallbackReturn activate() {return hardware_.on_activate(rclcpp_lifecycle::State{});}

  ServoBus bus_;
  dynamixel_hardware::DynamixelHardware hardware_;
};

TEST_F(DynamixelHardwareTest, FeedforwardGainsUseEManualRegisters)
{
  ASSERT_EQ(
    init(
      "", "<param name='Feedforward_1st_Gain'>4660</param>"
      "<param name='Feedforward_2nd_Gain'>9029</param>"),
    CallbackReturn::SUCCESS);
  for (uint8_t id = 1; id <= 2; ++id) {
    int first_gain_writes = 0;
    int second_gain_writes = 0;
    for (const auto & write : bus_.writes()) {
      if (write.id != id) {
        continue;
      }
      if (write.address == 90) {
        ++first_gain_writes;
        EXPECT_EQ(write.data, (Bytes{0x34, 0x12}));
      }
      if (write.address == 88) {
        ++second_gain_writes;
        EXPECT_EQ(write.data, (Bytes{0x45, 0x23}));
      }
    }
    EXPECT_EQ(first_gain_writes, 1);
    EXPECT_EQ(second_gain_writes, 1);
  }
}

TEST_F(DynamixelHardwareTest, ValidReadPublishesConvertedState)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  ASSERT_EQ(read_ms(10), return_type::OK);
  const auto values = state();
  ASSERT_EQ(values.size(), 6u);
  EXPECT_DOUBLE_EQ(values[0], 0.0);
  EXPECT_NEAR(values[1], 10 * 0.229 * 0.104719755, 1e-6);
  EXPECT_NEAR(values[2], 20 * 2.69, 1e-5);
  EXPECT_NEAR(values[3], -1.5707963268, 1e-6);
  EXPECT_NEAR(values[4], -5 * 0.229 * 0.104719755, 1e-6);
  EXPECT_NEAR(values[5], -10 * 2.69, 1e-5);
}

TEST_F(DynamixelHardwareTest, FailedReadPreservesEveryStateAndRecoveryResetsTimeout)
{
  ASSERT_EQ(init("<param name='read_error_timeout_ms'>30</param>"), CallbackReturn::SUCCESS);
  ASSERT_EQ(read_ms(10), return_type::OK);
  const auto valid = state();
  bus_.set_state(1, 3072, 30, 40);
  bus_.drop_reads(1);
  EXPECT_EQ(read_ms(20), return_type::OK);
  EXPECT_EQ(state(), valid);
  ASSERT_EQ(read_ms(10), return_type::OK);
  EXPECT_NE(state(), valid);
  bus_.drop_reads(2);
  EXPECT_EQ(read_ms(20), return_type::OK);
  const auto recovered = state();
  EXPECT_EQ(read_ms(10), return_type::ERROR);
  EXPECT_EQ(state(), recovered);
}

TEST_F(DynamixelHardwareTest, PartialBusReplyDoesNotPublishPartialState)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  ASSERT_EQ(read_ms(10), return_type::OK);
  const auto valid = state();
  bus_.set_state(1, 3072, 30, 40);
  bus_.drop_reads(1, 2);
  EXPECT_EQ(read_ms(10), return_type::OK);
  EXPECT_EQ(state(), valid);
}

TEST_F(DynamixelHardwareTest, DefaultTimeoutAndInitialNanState)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  bus_.drop_reads(3);
  EXPECT_EQ(read_ms(100), return_type::OK);
  EXPECT_EQ(read_ms(149), return_type::OK);
  EXPECT_EQ(read_ms(1), return_type::ERROR);
  for (const auto value : state()) {
    EXPECT_TRUE(std::isnan(value));
  }
}

TEST_F(DynamixelHardwareTest, TimeoutAtExactly1001Milliseconds)
{
  ASSERT_EQ(init("<param name='read_error_timeout_ms'>1001</param>"), CallbackReturn::SUCCESS);
  bus_.drop_reads(1);
  EXPECT_EQ(read_ms(1001), return_type::ERROR);
}

TEST_F(DynamixelHardwareTest, FractionalNanosecondTimeoutRoundsUp)
{
  ASSERT_EQ(init("<param name='read_error_timeout_ms'>0.0000015</param>"), CallbackReturn::SUCCESS);
  bus_.drop_reads(2);
  const auto period = rclcpp::Duration::from_nanoseconds(1);
  EXPECT_EQ(hardware_.read(rclcpp::Time{}, period), return_type::OK);
  EXPECT_EQ(hardware_.read(rclcpp::Time{}, period), return_type::ERROR);
}

class InvalidReadTimeoutTest : public DynamixelHardwareTest,
  public ::testing::WithParamInterface<const char *>
{
};

TEST_P(InvalidReadTimeoutTest, InvalidTimeoutRejectedBeforeServoPackets)
{
  EXPECT_EQ(
    init("<param name='read_error_timeout_ms'>" + std::string(GetParam()) + "</param>"),
    CallbackReturn::ERROR);
  EXPECT_TRUE(bus_.writes().empty());
  EXPECT_EQ(bus_.read_count(), 0);
}

INSTANTIATE_TEST_SUITE_P(
  InvalidValues, InvalidReadTimeoutTest,
  ::testing::Values("0", "-1", "nan", "inf", "no", "250ms", "", "1e999", "1e20"));

TEST_F(DynamixelHardwareTest, ActivationWithoutValidReadWritesNoGoal)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  bus_.drop_reads(3);
  EXPECT_EQ(activate(), CallbackReturn::ERROR);
  EXPECT_EQ(bus_.read_count(), 3);
  for (const auto & write : bus_.writes()) {
    EXPECT_NE(write.address, 116);
    EXPECT_FALSE(write.address == 64 && write.data.at(0) == 1);
  }
  for (const auto value : state()) {
    EXPECT_TRUE(std::isnan(value));
  }
}

TEST_F(DynamixelHardwareTest, ActivationRetriesTransientReadFailure)
{
  ASSERT_EQ(init("<param name='read_error_timeout_ms'>1</param>"), CallbackReturn::SUCCESS);
  bus_.drop_reads(1);
  EXPECT_EQ(activate(), CallbackReturn::SUCCESS);
  EXPECT_EQ(bus_.read_count(), 2);
  ASSERT_TRUE(bus_.wait_for_writes(116, 2));
  for (const auto & write : bus_.writes()) {
    if (write.address == 116) {
      EXPECT_EQ(word(write.data, 0), write.id == 1 ? 2048 : 1024);
    }
  }
  bus_.drop_reads(1);
  EXPECT_EQ(read_ms(0), return_type::OK);
}

TEST_F(DynamixelHardwareTest, ReactivationInVelocityModeTransmitsZeroVelocity)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
  auto commands = hardware_.export_command_interfaces();
  // The mode switch resets commands when restoring torque. Then command motion.
  ASSERT_TRUE(commands.at(1).set_value(1.0));
  ASSERT_EQ(hardware_.write(rclcpp::Time{}, rclcpp::Duration(0, 0)), return_type::OK);
  ASSERT_TRUE(commands.at(1).set_value(1.0));
  ASSERT_EQ(hardware_.write(rclcpp::Time{}, rclcpp::Duration(0, 0)), return_type::OK);
  ASSERT_TRUE(bus_.wait_for_writes(104, 4));
  const auto before = bus_.writes();
  auto velocity = std::find_if(before.rbegin(), before.rend(), [](const ServoBus::Write & write) {
        return write.id == 1 && write.address == 104;
  });
  ASSERT_NE(velocity, before.rend());
  EXPECT_EQ(velocity->data, (Bytes{41, 0, 0, 0}));

  ASSERT_EQ(hardware_.on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
  ASSERT_TRUE(bus_.wait_for_writes(104, 6));
  const auto after = bus_.writes();
  int zero_velocities = 0;
  int measured_goals = 0;
  for (size_t i = before.size(); i < after.size(); ++i) {
    if (after[i].address == 104) {
      ++zero_velocities;
      EXPECT_EQ(after[i].data, (Bytes{0, 0, 0, 0}));
    }
    if (after[i].address == 116) {
      ++measured_goals;
      EXPECT_EQ(word(after[i].data, 0), after[i].id == 1 ? 2048 : 1024);
    }
    EXPECT_NE(after[i].address, 11);  // Reactivation must not switch control modes.
  }
  EXPECT_EQ(zero_velocities, 2);
  EXPECT_EQ(measured_goals, 2);
}

TEST_F(DynamixelHardwareTest, MeasuredGoalsPrecedeTorqueEnable)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  for (const auto & write : bus_.writes()) {
    EXPECT_FALSE(write.address == 64 && write.data.at(0) == 1);
  }
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
  int goals = 0;
  int enables = 0;
  for (const auto & write : bus_.writes()) {
    if (write.address == 116) {
      ++goals;
      ASSERT_EQ(write.data.size(), 4u);
      EXPECT_EQ(word(write.data, 0), write.id == 1 ? 2048 : 1024);
      EXPECT_EQ(word(write.data, 2), 0);
    }
    if (write.address == 64 && write.data.at(0) == 1) {
      ++enables;
      EXPECT_EQ(goals, 2);
    }
  }
  EXPECT_EQ(goals, 2);
  EXPECT_EQ(enables, 2);
}

TEST_F(DynamixelHardwareTest, DisabledTorqueIsNeverEnabled)
{
  ASSERT_EQ(init("<param name='enable_torque'>false</param>"), CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
  ASSERT_TRUE(bus_.wait_for_writes(116, 2));
  for (const auto & write : bus_.writes()) {
    EXPECT_FALSE(write.address == 64 && write.data.at(0) == 1);
  }
}

TEST_F(DynamixelHardwareTest, TorqueEnableFailureAbortsActivationAfterSeeding)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  bus_.reject_torque_enable();
  EXPECT_EQ(activate(), CallbackReturn::ERROR);
  ASSERT_TRUE(bus_.wait_for_writes(116, 2));
  for (const auto & write : bus_.writes()) {
    EXPECT_FALSE(write.address == 64 && write.data.at(0) == 1);
  }
}

TEST_F(DynamixelHardwareTest, StatusByteStuffingIsDecoded)
{
  ASSERT_EQ(init(), CallbackReturn::SUCCESS);
  // Present velocity contains the Protocol 2 header marker ff ff fd.
  bus_.set_state(1, 2048, 0x00fdffff, 0);
  EXPECT_EQ(read_ms(10), return_type::OK);
  EXPECT_NEAR(state()[1], 0x00fdffff * 0.229 * 0.104719755, 0.1);
}
}  // namespace
