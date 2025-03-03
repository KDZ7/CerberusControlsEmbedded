#ifndef __LSC_SERVO_HPP__
#define __LSC_SERVO_HPP__

#include <memory>
#include "lsc_servo/visibility_control.h"
#include "idevice/idevice.hpp"
#include "hidraw/hidraw.hpp"

namespace lsc_servo
{
  struct HidrawDeleter
  {
    void operator()(hidraw::Hidraw *phidraw) const
    {
      if (phidraw)
      {
        if (phidraw->isConnected())
          phidraw->close();

        delete phidraw;
      }
    }
  };

  class Lsc_servo : public idevice::Servo
  {
  public:
    explicit Lsc_servo();

    bool connect() override;
    void disconnect() override;
    bool isConnected() const override;

    bool sendCmd(uint8_t cmd, const std::vector<uint8_t> &params) override;
    bool recv(std::vector<uint8_t> &response) override;
    bool recvTimeout(std::vector<uint8_t> &response, int timeout = 100) override;

    bool moveServo(const std::vector<std::tuple<uint8_t, double>> &servos, uint16_t dtime) override;
    bool getBatteryVoltage(uint16_t &voltage) override;
    bool unloadServo(const std::vector<uint8_t> &servo_ids) override;
    std::map<uint8_t, double> stateServo(const std::vector<uint8_t> &servo_ids, int timeout = 100) override;

    bool runActionGroup(uint8_t group_id, uint16_t repetitions) override;
    bool stopActionGroup() override;
    bool setActionGroupSpeed(uint8_t group_id, uint16_t speed) override;

    bool isActionGroupRunning(uint8_t &group_id, uint16_t &repetitions) override;
    bool isActionGroupStopped() override;
    bool isActionGroupComplete(uint8_t &group_id, uint16_t &repetitions) override;

  private:
    std::unique_ptr<hidraw::Hidraw, HidrawDeleter> phidraw;
    static constexpr uint16_t VENDOR_ID = 0x0483;
    static constexpr uint16_t PRODUCT_ID = 0x5750;
    static constexpr uint8_t HEADER[] = {0x55, 0x55};
    std::vector<uint8_t> buildCommandPacket(uint8_t cmd, const std::vector<uint8_t> &params);
  };

} // namespace lsc_servo

#endif // __LSC_SERVO_HPP__
