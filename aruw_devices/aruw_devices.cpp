// ARUW-specific hw-emulator devices, loaded by the hub as a plugin
// (devices.yaml `plugins: [.../libaruw_devices.so]`). They live here rather
// than in hw-emulator because they're ARUW's own boards or RMUL's referee:
//
//   aruw_referee                 the RMUL referee system on its UART (DJI
//                                serial framing): game status, robot status,
//                                power and heat, position. The game model
//                                itself waits on the RMUL rules: until then
//                                the match is held "in progress" at fixed
//                                values, and that is reported as a sim gap.
//   aruw_cap_bank                the capacitor bank (CAN 0x1EC): a stub. It
//                                answers the MCB's commands with a full,
//                                healthy bank; no charge model.
//   aruw_voltage_current_sensor  the chassis supply sensor (CAN 0x1C5): bus
//                                voltage, and the battery current the bound
//                                motors draw, from their sim currents and
//                                speeds (P = sum V_motor * I) / V_bus.
//
// Each reports what it doesn't model, once, as a sim gap (hwemu/gaps.hpp).
#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "hwemu/devices/device.hpp"
#include "hwemu/gaps.hpp"
#include "hwemu/protocol/dji_serial.hpp"

namespace aruw {
namespace {

using hwemu::devices::DeviceSpec;
using hwemu::devices::Every;
using hwemu::devices::IDevice;
using hwemu::devices::Params;
using hwemu::devices::SimTime;
using hwemu::devices::Transports;
using hwemu::protocol::CanFrame;

std::string Context(const DeviceSpec& spec) { return spec.type + " '" + spec.name + "'"; }

void PutU16(std::vector<std::uint8_t>* b, std::size_t at, std::uint16_t v) {
  (*b)[at] = static_cast<std::uint8_t>(v & 0xFF);
  (*b)[at + 1] = static_cast<std::uint8_t>(v >> 8);
}
void PutU32(std::vector<std::uint8_t>* b, std::size_t at, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) (*b)[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
}
void PutF32(std::vector<std::uint8_t>* b, std::size_t at, float v) { std::memcpy(b->data() + at, &v, 4); }

// ---- referee ----------------------------------------------------------------
// The messages, as taproot's RefSerial decodes them (ref_serial.cpp).
constexpr std::uint16_t kGameStatus = 0x0001;    // 11 B
constexpr std::uint16_t kAllRobotHp = 0x0003;    // 32 B
constexpr std::uint16_t kRobotStatus = 0x0201;   // 13 B
constexpr std::uint16_t kPowerAndHeat = 0x0202;  // 14 B
constexpr std::uint16_t kRobotPosition = 0x0203; // 12 B

class Referee final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override {
    SetIdentity(spec);
    const std::string ctx = Context(spec);
    const Params transport(spec.transport, ctx + " transport");
    const Params params(spec.params, ctx + " params");
    port_spec_ = transport.Text("port", "referee");
    robot_id_ = static_cast<std::uint8_t>(params.Integer("robot_id", 107));  // 107: blue sentry
    max_hp_ = static_cast<std::uint16_t>(params.Integer("max_hp", 400));
    power_limit_ = static_cast<std::uint16_t>(params.Integer("chassis_power_limit", 100));
    heat_limit_ = static_cast<std::uint16_t>(params.Integer("heat_limit", 400));
    cooling_rate_ = static_cast<std::uint16_t>(params.Integer("cooling_rate", 80));
    stage_ = static_cast<std::uint8_t>(params.Integer("game_stage", 4));  // 4: in match
    game_type_ = static_cast<std::uint8_t>(params.Integer("game_type", 3));  // 3: RMUL
    match_seconds_ = params.Number("match_seconds", 420);
    status_.SetRate(params.Number("status_rate_hz", 10));
    power_.SetRate(params.Number("power_heat_rate_hz", 50));
  }

  void Bind(hwemu::sim::ISim&, Transports& t) override {
    port_ = t.OpenSerial(port_spec_, 115200);
    hwemu::gaps::Report("referee", "game-model-pending",
                        "the RMUL game model isn't written yet (waiting on the rules): the match is held "
                        "in progress with fixed HP, power and heat; no damage, no buffs, no RFID");
  }

  void Tick(const SimTime& now) override {
    std::vector<std::uint8_t> in;
    port_->Read(&in);
    parser_.Feed(in);
    while (auto frame = parser_.Next()) {
      // The MCB's transmissions (client UI, robot interaction): parsed and
      // counted, not yet acted on.
      ++received_;
      hwemu::gaps::Report("referee", "rx:" + std::to_string(frame->cmd),
                          "the MCB sent referee message 0x" + Hex(frame->cmd) +
                              " (client UI or robot interaction); the sim parses it but doesn't act on it");
    }
    if (status_.Due(now)) {
      std::vector<std::uint8_t> g(11, 0);
      g[0] = static_cast<std::uint8_t>((game_type_ & 0xF) | ((stage_ & 0xF) << 4));
      const double left = std::max(0.0, match_seconds_ - now.seconds);
      PutU16(&g, 1, static_cast<std::uint16_t>(left));
      Send(kGameStatus, g);

      std::vector<std::uint8_t> r(13, 0);
      r[0] = robot_id_;
      r[1] = 1;  // level
      PutU16(&r, 2, max_hp_);
      PutU16(&r, 4, max_hp_);
      PutU16(&r, 6, cooling_rate_);
      PutU16(&r, 8, heat_limit_);
      PutU16(&r, 10, power_limit_);
      r[12] = 0b111;  // gimbal, chassis and shooter power outputs on
      Send(kRobotStatus, r);

      std::vector<std::uint8_t> hp(32, 0);
      for (std::size_t i = 0; i < 16; ++i) PutU16(&hp, 2 * i, max_hp_);
      Send(kAllRobotHp, hp);
    }
    if (power_.Due(now)) {
      std::vector<std::uint8_t> p(14, 0);
      PutU16(&p, 8, 60);  // energy buffer, J
      Send(kPowerAndHeat, p);
      std::vector<std::uint8_t> pos(12, 0);
      Send(kRobotPosition, pos);
    }
  }

  std::string Status() const override {
    return "sent=" + std::to_string(sent_) + " received=" + std::to_string(received_) +
           " port=" + (port_ ? port_->Path() : "");
  }

 private:
  static std::string Hex(std::uint16_t v) {
    char b[8];
    std::snprintf(b, sizeof(b), "%04X", v);
    return b;
  }
  void Send(std::uint16_t cmd, const std::vector<std::uint8_t>& data) {
    const auto frame = hwemu::protocol::EncodeDjiFrame(cmd, data, seq_++);
    port_->Write(frame);
    ++sent_;
  }

  std::string port_spec_;
  std::unique_ptr<hwemu::transport::ISerialPort> port_;
  hwemu::protocol::DjiFrameParser parser_;
  std::uint8_t robot_id_ = 107, stage_ = 4, game_type_ = 3, seq_ = 0;
  std::uint16_t max_hp_ = 400, power_limit_ = 100, heat_limit_ = 400, cooling_rate_ = 80;
  double match_seconds_ = 420;
  Every status_, power_;
  std::uint64_t sent_ = 0, received_ = 0;
};

// ---- capacitor bank (stub) ----------------------------------------------------
constexpr std::uint32_t kCapBankId = 0x1EC;

class CapBank final : public IDevice {
 public:
  void Configure(const DeviceSpec& spec) override {
    SetIdentity(spec);
    const Params transport(spec.transport, Context(spec) + " transport");
    const Params params(spec.params, Context(spec) + " params");
    bus_name_ = transport.Text("bus");
    voltage_ = params.Number("voltage", 24.0);
    status_.SetRate(params.Number("status_rate_hz", 100));
  }

  void Bind(hwemu::sim::ISim&, Transports& t) override {
    bus_ = t.Can(bus_name_);
    t.Router().Subscribe(bus_, kCapBankId, false, [this](const CanFrame& f) {
      if (f.len >= 2 && f.data[0] == 0x28) mode_ = f.data[1];  // CAP_COMMAND: mode
      if (f.len >= 7 && f.data[0] == 0x28) limit_raw_ = f.data[6];
      ++commands_;
    });
    hwemu::gaps::Report("cap_bank", "stub",
                        "the capacitor bank is a stub: always full and healthy, no charge or discharge model");
  }

  void Tick(const SimTime& now) override {
    if (!status_.Due(now)) return;
    // The state its mode asks for: OFF/IDLE -> STANDBY, CHARGE -> CHARGING,
    // DISCHARGE -> BOOST.
    static constexpr std::uint8_t kState[] = {1, 1, 2, 3};
    CanFrame f;
    f.id = kCapBankId;
    f.len = 8;
    f.data[0] = 0x05;  // STATUS
    f.data[1] = kState[mode_ & 3];
    const auto mv = static_cast<std::uint16_t>(voltage_ * 1000);
    f.data[4] = static_cast<std::uint8_t>(mv & 0xFF);
    f.data[5] = static_cast<std::uint8_t>(mv >> 8);
    f.data[6] = 100;         // energy %
    f.data[7] = limit_raw_;  // supply power: what the MCB allows
    bus_->Send(f);
  }

  std::string Status() const override {
    return "mode=" + std::to_string(mode_) + " commands=" + std::to_string(commands_);
  }

 private:
  std::string bus_name_;
  hwemu::transport::ICanBus* bus_ = nullptr;
  double voltage_ = 24.0;
  std::uint8_t mode_ = 1, limit_raw_ = 25;
  std::uint64_t commands_ = 0;
  Every status_;
};

// ---- chassis voltage/current sensor ------------------------------------------
constexpr std::uint32_t kChassisSensorId = 0x1C5;

class VoltageCurrentSensor final : public IDevice {
 public:
  struct Load {
    std::string motor, encoder;
    double resistance = 0, back_emf = 0, reduction = 1;
    hwemu::sim::MotorHandle m;
    hwemu::sim::EncoderHandle e;
  };

  void Configure(const DeviceSpec& spec) override {
    SetIdentity(spec);
    const std::string ctx = Context(spec);
    bus_name_ = Params(spec.transport, ctx + " transport").Text("bus");
    const Params params(spec.params, ctx + " params");
    bus_voltage_ = params.Number("bus_voltage", 24.0);
    rate_.SetRate(params.Number("rate_hz", 500));
    // bind.loads: [{motor, encoder, resistance, back_emf, reduction}] --
    // each motor's winding resistance (ohm), back-EMF constant at the rotor
    // (V per rad/s) and rotor turns per joint turn.
    for (const auto& l : spec.bind["loads"]) {
      const Params p(l, ctx + " bind.loads");
      loads_.push_back({p.Text("motor"), p.Text("encoder"), p.Number("resistance"), p.Number("back_emf"),
                        p.Number("reduction", 1.0), nullptr, nullptr});
    }
    if (loads_.empty()) throw std::invalid_argument(ctx + ": bind.loads needs at least one motor");
  }

  void Bind(hwemu::sim::ISim& sim, Transports& t) override {
    bus_ = t.Can(bus_name_);
    for (auto& l : loads_) {
      l.m = sim.Motor(l.motor);
      l.e = sim.Encoder(l.encoder);
    }
    hwemu::gaps::Report("voltage_current_sensor", "ideal-supply",
                        "the chassis supply is ideal: fixed bus voltage, battery current from the bound "
                        "motors' electrical power only (no other loads, no converter losses)");
  }

  void Tick(const SimTime& now) override {
    if (!rate_.Due(now)) return;
    double power = 0;
    for (const auto& l : loads_) {
      const double i = l.m->Read().current;
      const double w = l.e->Read().velocity * l.reduction;  // rotor rad/s
      power += i * (i * l.resistance + l.back_emf * w);
    }
    const double amps = std::max(0.0, power / bus_voltage_);
    CanFrame f;
    f.id = kChassisSensorId;
    f.len = 4;
    const auto mv = static_cast<std::uint16_t>(std::lround(bus_voltage_ * 1000));
    const auto ma = static_cast<std::uint16_t>(std::min(65535.0, std::round(amps * 1000)));
    f.data[0] = static_cast<std::uint8_t>(mv & 0xFF);
    f.data[1] = static_cast<std::uint8_t>(mv >> 8);
    f.data[2] = static_cast<std::uint8_t>(ma & 0xFF);
    f.data[3] = static_cast<std::uint8_t>(ma >> 8);
    bus_->Send(f);
    last_amps_ = amps;
  }

  std::string Status() const override { return "current=" + std::to_string(last_amps_) + "A"; }

 private:
  std::string bus_name_;
  hwemu::transport::ICanBus* bus_ = nullptr;
  double bus_voltage_ = 24.0, last_amps_ = 0;
  std::vector<Load> loads_;
  Every rate_;
};

}  // namespace
}  // namespace aruw

extern "C" void hwemu_register_devices(hwemu::devices::DeviceRegistry* registry) {
  registry->Register("aruw_referee", [] { return std::make_unique<aruw::Referee>(); });
  registry->Register("aruw_cap_bank", [] { return std::make_unique<aruw::CapBank>(); });
  registry->Register("aruw_voltage_current_sensor", [] { return std::make_unique<aruw::VoltageCurrentSensor>(); });
}
