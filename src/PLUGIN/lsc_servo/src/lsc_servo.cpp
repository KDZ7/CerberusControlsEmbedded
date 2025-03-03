#include <iostream>
#include <cmath>
#include "lsc_servo/lsc_servo.hpp"

/*
 *  rad <---> deg <---> servo
 *  ~[-0.785398163, +0.785398163397] <---> ~[-120°, 120°]  <---> ~[0, 1000]
 *
 */

#define RAD_TO_SERVO(rad) (uint16_t)(((-rad * (180.0 / M_PI)) + 120.0) * (1000.0 / 240.0))
#define SERVO_TO_RAD(pos) (double)((-(pos * (240.0 / 1000.0)) + 120.0) * (M_PI / 180.0))

namespace lsc_servo
{
    Lsc_servo::Lsc_servo()
    {
        phidraw = std::unique_ptr<hidraw::Hidraw, HidrawDeleter>(new hidraw::Hidraw());
    }

    bool Lsc_servo::connect()
    {
        LOG_INFO("Trying to connect to HID device ...");
        if (phidraw->openDevice(VENDOR_ID, PRODUCT_ID))
        {
            LOG_INFO("HID Connection established");
            return true;
        }
        else
        {
            LOG_ERROR("HID Connection failed");
            return false;
        }
    }

    void Lsc_servo::disconnect()
    {
        phidraw->close();
        LOG_INFO("HID Connection closed");
    }

    bool Lsc_servo::isConnected() const
    {
        return phidraw->isConnected();
    }

    bool Lsc_servo::sendCmd(uint8_t cmd, const std::vector<uint8_t> &params)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        std::vector<uint8_t> packet = buildCommandPacket(cmd, params);
        if (phidraw->write(packet.data(), packet.size()) < 0)
        {
            LOG_ERROR("Failed to send command");
            return false;
        }

        return true;
    }

    bool Lsc_servo::recv(std::vector<uint8_t> &response)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        response.clear();
        response.resize(64, 0);
        int bytesRead = phidraw->read(response.data(), response.size());
        if (bytesRead <= 0)
        {
            LOG_ERROR("Failed to receive |header|length|");
            return false;
        }

        response.resize(bytesRead);
        if (response[0] != HEADER[0] || response[1] != HEADER[1])
        {
            LOG_ERROR("Invalid response header");
            return false;
        }

        return true;
    }

    bool Lsc_servo::recvTimeout(std::vector<uint8_t> &response, int timeout)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        response.clear();
        response.resize(64, 0);
        int bytesRead = phidraw->readTimeout(response.data(), response.size(), timeout);
        if (bytesRead <= 0)
        {
            LOG_ERROR("Failed to receive |header|length| within timeout " << timeout << "ms");
            return false;
        }

        response.resize(bytesRead);
        if (response[0] != HEADER[0] || response[1] != HEADER[1])
        {
            LOG_ERROR("Invalid response header");
            return false;
        }

        return true;
    }

    bool Lsc_servo::moveServo(const std::vector<std::tuple<uint8_t, double>> &servos, uint16_t dtime)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        if (servos.empty())
        {
            LOG_ERROR("No servos specified");
            return false;
        }

        std::vector<uint8_t> params;
        params.push_back(static_cast<uint8_t>(servos.size()));
        params.push_back(static_cast<uint8_t>(dtime & 0xFF));
        params.push_back(static_cast<uint8_t>((dtime >> 8) & 0xFF));

        for (const auto &servo : servos)
        {
            uint8_t id = std::get<0>(servo);
            double angleRad = std::get<1>(servo);
            uint16_t position = RAD_TO_SERVO(angleRad);
            params.push_back(id);
            params.push_back(static_cast<uint8_t>(position & 0xFF));
            params.push_back(static_cast<uint8_t>((position >> 8) & 0xFF));
        }

        return sendCmd(CMD_SERVO_MOVE, params);
    }

    bool Lsc_servo::getBatteryVoltage(uint16_t &voltage)
    {
        if (!sendCmd(CMD_GET_BATTERY_VOLTAGE, {}))
        {
            LOG_ERROR("Unable to send the battery voltage command!");
            return false;
        }

        std::vector<uint8_t> response;
        if (!recvTimeout(response))
        {
            LOG_ERROR("Unable to receive the battery voltage response!");
            return false;
        }
        voltage = static_cast<uint16_t>((response[4] | (response[5] << 8)));

        return true;
    }

    bool Lsc_servo::unloadServo(const std::vector<uint8_t> &servo_ids)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        if (servo_ids.empty())
        {
            LOG_ERROR("No servos specified");
            return false;
        }

        std::vector<uint8_t> params;
        params.push_back(static_cast<uint8_t>(servo_ids.size()));
        params.insert(params.end(), servo_ids.begin(), servo_ids.end());
        return sendCmd(CMD_MULT_SERVO_UNLOAD, params);
    }

    std::map<uint8_t, double> Lsc_servo::stateServo(const std::vector<uint8_t> &servo_ids, int timeout)
    {
        std::map<uint8_t, double> positions;
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return positions;
        }

        if (servo_ids.empty())
        {
            LOG_ERROR("No servos specified");
            return positions;
        }

        std::vector<uint8_t> params;
        params.push_back(static_cast<uint8_t>(servo_ids.size()));
        params.insert(params.end(), servo_ids.begin(), servo_ids.end());
        if (!sendCmd(CMD_MULT_SERVO_POS_READ, params))
        {
            LOG_ERROR("Failed to send command");
            return positions;
        }

        std::vector<uint8_t> response;
        if (!recvTimeout(response, timeout))
        {
            LOG_ERROR("Failed to receive response");
            return positions;
        }

        if (response.size() < 5 || response[3] != CMD_MULT_SERVO_POS_READ)
        {
            LOG_ERROR("Invalid response");
            return positions;
        }

        uint8_t numServos = response[4];
        if (numServos != servo_ids.size())
            LOG_WARN("Number of servos in response does not match request");

        size_t index = 5;
        for (uint8_t i = 0; i < numServos; ++i)
        {
            if (index + 2 >= response.size())
            {
                LOG_WARN("Incomplete position data for servo " << (int)response[index]);
                break;
            }
            uint8_t servo_id = response[index];
            uint16_t position = static_cast<uint16_t>(response[index + 1]) | (static_cast<uint16_t>(response[index + 2]) << 8);
            double angleRad = SERVO_TO_RAD(position);
            positions[servo_id] = angleRad;
            index += 3;
        }

        return positions;
    }

    bool Lsc_servo::runActionGroup(uint8_t group_id, uint16_t repetitions)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        std::vector<uint8_t> params;
        params.push_back(group_id);
        params.push_back(static_cast<uint8_t>(repetitions & 0xFF));
        params.push_back(static_cast<uint8_t>((repetitions >> 8) & 0xFF));

        return sendCmd(CMD_ACTION_GROUP_RUN, params);
    }

    bool Lsc_servo::stopActionGroup()
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        return sendCmd(CMD_ACTION_STOP, {});
    }

    bool Lsc_servo::setActionGroupSpeed(uint8_t group_id, uint16_t speed)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        std::vector<uint8_t> params;
        params.push_back(group_id);
        params.push_back(static_cast<uint8_t>(speed & 0xFF));
        params.push_back(static_cast<uint8_t>((speed >> 8) & 0xFF));

        return sendCmd(CMD_ACTION_SPEED, params);
    }

    bool Lsc_servo::isActionGroupRunning(uint8_t &group_id, uint16_t &repetitions)
    {
        std::vector<uint8_t> response;
        if (!recvTimeout(response))
        {
            LOG_ERROR("Failed to receive response");
            return false;
        }

        if (response.size() != 7 || response[3] != CMD_ACTION_GROUP_RUN)
        {
            LOG_ERROR("Invalid response");
            return false;
        }

        group_id = response[4];
        repetitions = static_cast<uint16_t>(response[5]) | (static_cast<uint16_t>(response[6]) << 8);

        return true;
    }

    bool Lsc_servo::isActionGroupStopped()
    {
        std::vector<uint8_t> response;

        if (!recvTimeout(response))
        {
            LOG_ERROR("Failed to receive response");
            return false;
        }

        if (response.size() != 4 || response[3] != CMD_ACTION_GROUP_STOP)
        {
            LOG_ERROR("Invalid response");
            return false;
        }

        return true;
    }

    bool Lsc_servo::isActionGroupComplete(uint8_t &group_id, uint16_t &repetitions)
    {
        if (!isConnected())
        {
            LOG_ERROR("HID Connection not established");
            return false;
        }

        std::vector<uint8_t> response;

        if (!recvTimeout(response))
        {
            LOG_ERROR("Failed to receive response");
            return false;
        }

        if (response.size() != 7 || response[3] != CMD_ACTION_GROUP_COMPLETE)
        {
            LOG_ERROR("Invalid response");
            return false;
        }

        group_id = response[4];
        repetitions = static_cast<uint16_t>(response[5]) | (static_cast<uint16_t>(response[6]) << 8);
        LOG_INFO("Action Group " << (int)group_id << " completed after " << repetitions << " repetitions.");

        return true;
    }

    std::vector<uint8_t> Lsc_servo::buildCommandPacket(uint8_t cmd, const std::vector<uint8_t> &params)
    {
        std::vector<uint8_t> packet;
        packet.push_back(HEADER[0]);
        packet.push_back(HEADER[1]);
        uint8_t length = static_cast<uint8_t>(params.size() + 2);
        packet.push_back(length);
        packet.push_back(cmd);
        packet.insert(packet.end(), params.begin(), params.end());
        return packet;
    }

} // namespace lsc_servo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lsc_servo::Lsc_servo, idevice::Servo)
