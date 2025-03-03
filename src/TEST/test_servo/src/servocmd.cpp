#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include "pluginlib/class_loader.hpp"
#include "idevice/idevice.hpp"

int main(int argc, char **argv)
{
    std::vector<std::tuple<uint8_t, double>> positions;
    for (int i = 1; i < argc; i++)
    {
        if (std::string(argv[i]) == "-id" && i + 2 < argc)
        {
            uint8_t id = std::stoi(argv[i + 1]);
            double pos = std::stod(argv[i + 2]);
            positions.push_back({id, pos});
            i += 2;
        }
    }
    std::vector<uint8_t> servo_ids;
    for (const auto &pos : positions)
        servo_ids.push_back(std::get<0>(pos));

    // Initialize the plugin loader for the servo interface "iservo"
    pluginlib::ClassLoader<idevice::Servo> loader("idevice", "idevice::Servo");
    try
    {
        std::shared_ptr<idevice::Servo> servo = loader.createSharedInstance("lsc_servo::Lsc_servo");

        // Test Connection
        if (!servo->connect())
        {
            std::cout << "Failed to connect to servo" << std::endl;
            return 1;
        }
        std::cout << "Connected to servo" << std::endl;

        // Test battery voltage reading
        uint16_t voltage = 0;
        if (!servo->getBatteryVoltage(voltage))
        {
            std::cout << "Failed to get voltage" << std::endl;
            return 1;
        }
        std::cout << "Voltage: " << voltage << ".d mV" << std::endl;

        // Test moving and reading servo positions
        if (!positions.empty())
        {
            if (!servo->moveServo(positions, 1000))
            {
                std::cout << "Failed to move servos" << std::endl;
                return 1;
            }
            std::cout << "Servos moved" << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            std::map<uint8_t, double> servo_positions = servo->stateServo(servo_ids, 100);
            for (const auto &pos : servo_positions)
                std::cout << "Servo ID: " << (int)pos.first << " Position: " << pos.second << std::endl;
        }

        // Test continuous reading of servo positions
        for (int i = 0; i < 10; i++)
        {
            std::map<uint8_t, double> servo_positions = servo->stateServo(servo_ids, 100);
            if (servo_positions.empty())
            {
                std::cout << " > " << i << " - Failed to read servo positions" << std::endl;
                return 1;
            }
            for (const auto &pos : servo_positions)
                std::cout << " > " << i << " - Servo ID: " << (int)pos.first << " Position: " << pos.second << std::endl;
        }

        // Test powering off selected servos
        // std::vector<uint8_t> servos_to_power_off = {33};
        // std::cout << "Powering off selected servos..." << std::endl;
        // if (servo->unloadServo(servos_to_power_off))
        //     std::cout << "Servos powered off successfully" << std::endl;
        // else
        //     std::cout << "Failed to power off servos" << std::endl;

        // // Test sending direct command
        // uint8_t servoId = 33;
        // uint16_t position = 0xFF;
        // uint16_t dtime = 1000;

        // std::vector<uint8_t> params;
        // params.push_back(1);
        // params.push_back(time & 0xFF);
        // params.push_back((time >> 8) & 0xFF);
        // params.push_back(servoId);
        // params.push_back(position & 0xFF);
        // params.push_back((position >> 8) & 0xFF);

        // if (!servo->sendCmd(idevice::Servo::CMD_SERVO_MOVE, params))
        // {
        //     std::cout << "Failed to send move command" << std::endl;
        //     return 1;
        // }
        // std::cout << "Move command sent" << std::endl;

        servo->disconnect();
        std::cout << "Disconnected" << std::endl;
    }
    catch (pluginlib::PluginlibException &ex)
    {
        std::cerr << "Plugin exception: " << ex.what() << std::endl;
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cerr << "Standard exception: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}