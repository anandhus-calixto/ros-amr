#include <libserial/SerialPort.h>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>

// Binary MCU<->MPU packet protocol (standardized 2026-09-29). Same wire
// format as calixto-amr-imx-rt-app/src/mcu_frame/mcu_frame_protocol.h on the
// MCU side and remote-control/remote_bridge.py - see
// calixto-amr-imx-rt-app/calixto-amr-info.md's "MCU↔MPU Binary Packet
// Protocol" section for the full spec/history. Replaces the earlier ASCII
// protocol ("<left>,<right>\r" / "e\r" -> "<left>,<right>\n").
//
// Framing: each packet starts with a unique, fixed sync byte and is a fixed
// size - no length field needed. CRC-8 (polynomial 0x07, init 0x00, no
// reflection) must match the MCU side exactly.
//
// Only the Command Packet and the Telemetry Request/Telemetry Packet
// round-trip are used here - read()/write() in diffbot_system.cpp keep the
// same request/reply shape as before, just binary now instead of ASCII.

namespace calixto_mcu_protocol
{

inline uint8_t crc8(const uint8_t *data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++)
    {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

constexpr uint8_t SYNC_TELEMETRY_REQUEST = 0xE5;  // i.MX 95 -> i.MX RT
constexpr uint8_t SYNC_COMMAND           = 0xA5;  // i.MX 95 -> i.MX RT
constexpr uint8_t SYNC_TELEMETRY         = 0x5A;  // i.MX RT -> i.MX 95

constexpr uint8_t CMD_ID_DRIVE_VELOCITY = 0x01;

// status_flags bits (Telemetry Packet, Byte 19) - must match
// mcu_frame_protocol.h's MCU_STATUS_* exactly.
constexpr uint8_t STATUS_FRONT_ESTOP    = 1u << 0;
constexpr uint8_t STATUS_BACK_ESTOP     = 1u << 1;
constexpr uint8_t STATUS_BUMPER         = 1u << 2;  // raw switch level right now
constexpr uint8_t STATUS_COMMS_TIMEOUT  = 1u << 3;
constexpr uint8_t STATUS_CAN_FAULT      = 1u << 4;
constexpr uint8_t STATUS_BUMPER_LATCHED = 1u << 5;  // the actual stop condition - see safety.c

#pragma pack(push, 1)

struct TelemetryRequestPacket
{
  uint8_t header;  // Fixed: SYNC_TELEMETRY_REQUEST
  uint8_t crc8;    // CRC-8 over header
};

struct CommandPacket
{
  uint8_t  header;         // Fixed: SYNC_COMMAND
  uint8_t  cmd_id;         // CMD_ID_DRIVE_VELOCITY, etc.
  uint16_t sequence_num;   // Incremental packet counter (little-endian)
  float    cmd_vel_left;   // rad/s
  float    cmd_vel_right;  // rad/s
  uint8_t  control_flags;  // bit0: enable drives, bit1: reset fault, bit2: e-brake
  uint8_t  crc8;           // CRC-8 over bytes 0-12
  uint8_t  padding;        // 0x00
};

struct TelemetryPacket
{
  uint8_t  header;               // Fixed: SYNC_TELEMETRY
  uint16_t sequence_num;         // Mirrors the last received Command Packet's sequence_num
  int32_t  encoder_ticks_left;
  int32_t  encoder_ticks_right;
  float    measured_vel_left;    // rad/s
  float    measured_vel_right;   // rad/s
  uint8_t  status_flags;         // bit0: front e-stop, bit1: back e-stop,
                                  // bit2: bumper (raw switch level right now),
                                  // bit3: comms timeout, bit4: CAN fault,
                                  // bit5: bumper latch active (stays 1 after the
                                  // bumper itself releases, until an e-stop is
                                  // pressed and released - see safety.c)
  uint8_t  drive_fault_code;
  uint8_t  battery_soc;          // 0xFF = not implemented
  uint8_t  crc8;                 // CRC-8 over bytes 0-21
  uint8_t  padding;               // 0x00
};

#pragma pack(pop)

static_assert(sizeof(TelemetryRequestPacket) == 2, "TelemetryRequestPacket size mismatch");
static_assert(sizeof(CommandPacket) == 15, "CommandPacket size mismatch");
static_assert(sizeof(TelemetryPacket) == 24, "TelemetryPacket size mismatch");

}  // namespace calixto_mcu_protocol


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class McuComms
{

public:

  McuComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  // Sends a Telemetry Request and blocks for the Telemetry Packet reply
  // (matches the old "e\r"-request shape, just binary now). On timeout, a
  // bad sync byte, or a CRC mismatch, left/right are set to 0 (same
  // effective behaviour as the old ASCII path's atoi("") on an empty/timed
  // out response) and false is returned.
  bool read_encoder_values(int &left, int &right)
  {
    using namespace calixto_mcu_protocol;

    TelemetryRequestPacket req{};
    req.header = SYNC_TELEMETRY_REQUEST;
    req.crc8 = crc8(reinterpret_cast<const uint8_t *>(&req), offsetof(TelemetryRequestPacket, crc8));

    LibSerial::DataBuffer out(reinterpret_cast<const uint8_t *>(&req),
                               reinterpret_cast<const uint8_t *>(&req) + sizeof(req));
    serial_conn_.Write(out);

    LibSerial::DataBuffer in;
    try
    {
      serial_conn_.Read(in, sizeof(TelemetryPacket), timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
      std::cerr << "Telemetry read timed out." << std::endl;
      left = 0;
      right = 0;
      return false;
    }

    if (in.size() != sizeof(TelemetryPacket))
    {
      std::cerr << "Telemetry packet wrong size: " << in.size() << std::endl;
      left = 0;
      right = 0;
      return false;
    }

    TelemetryPacket pkt;
    std::memcpy(&pkt, in.data(), sizeof(pkt));

    if (pkt.header != SYNC_TELEMETRY)
    {
      std::cerr << "Telemetry packet bad sync byte: 0x" << std::hex << (int)pkt.header << std::dec << std::endl;
      left = 0;
      right = 0;
      return false;
    }

    uint8_t calc_crc = crc8(reinterpret_cast<const uint8_t *>(&pkt), offsetof(TelemetryPacket, crc8));
    if (calc_crc != pkt.crc8)
    {
      std::cerr << "Telemetry packet CRC mismatch." << std::endl;
      left = 0;
      right = 0;
      return false;
    }

    left = pkt.encoder_ticks_left;
    right = pkt.encoder_ticks_right;
    status_flags_ = pkt.status_flags;
    drive_fault_code_ = pkt.drive_fault_code;
    status_valid_ = true;
    return true;
  }

  // Status from the most recent successfully-parsed Telemetry Packet - see
  // calixto_mcu_protocol::STATUS_* above for the bit meanings. status_valid()
  // is false until the first successful read_encoder_values() call, and
  // stays true afterwards (holds the last-known value through a later
  // failed read, same as encoder ticks aren't reset on failure either).
  bool status_valid() const { return status_valid_; }
  uint8_t status_flags() const { return status_flags_; }
  uint8_t drive_fault_code() const { return drive_fault_code_; }

  bool front_estop_active() const { return status_flags_ & calixto_mcu_protocol::STATUS_FRONT_ESTOP; }
  bool back_estop_active() const { return status_flags_ & calixto_mcu_protocol::STATUS_BACK_ESTOP; }
  bool bumper_active() const { return status_flags_ & calixto_mcu_protocol::STATUS_BUMPER; }
  bool bumper_latched() const { return status_flags_ & calixto_mcu_protocol::STATUS_BUMPER_LATCHED; }
  bool comms_timeout() const { return status_flags_ & calixto_mcu_protocol::STATUS_COMMS_TIMEOUT; }
  bool can_fault() const { return status_flags_ & calixto_mcu_protocol::STATUS_CAN_FAULT; }

  // Fire-and-forget - firmware sends no reply to a Command Packet.
  void set_motor_values(double left, double right)
  {
    using namespace calixto_mcu_protocol;

    CommandPacket cmd{};
    cmd.header = SYNC_COMMAND;
    cmd.cmd_id = CMD_ID_DRIVE_VELOCITY;
    cmd.sequence_num = sequence_num_++;
    cmd.cmd_vel_left = static_cast<float>(left);
    cmd.cmd_vel_right = static_cast<float>(right);
    cmd.control_flags = 0x00;
    cmd.crc8 = crc8(reinterpret_cast<const uint8_t *>(&cmd), offsetof(CommandPacket, crc8));
    cmd.padding = 0x00;

    LibSerial::DataBuffer out(reinterpret_cast<const uint8_t *>(&cmd),
                               reinterpret_cast<const uint8_t *>(&cmd) + sizeof(cmd));
    serial_conn_.Write(out);
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_ = 0;
    uint16_t sequence_num_ = 0;
    uint8_t status_flags_ = 0;
    uint8_t drive_fault_code_ = 0;
    bool status_valid_ = false;
};
