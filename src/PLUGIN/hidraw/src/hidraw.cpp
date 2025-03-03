#include <iostream>
#include "hidraw/hidraw.hpp"

namespace hidraw
{
    Hidraw::~Hidraw()
    {
        close();
    }
    bool Hidraw::openDevice(uint16_t vendor_id, uint16_t product_id)
    {
        if (hid_init() < 0)
        {
            LOG_ERROR("Failed to initialize HIDAPI");
            return false;
        }

        device.reset(hid_open(vendor_id, product_id, nullptr));
        if (!device)
        {
            LOG_ERROR("Failed to open device");
            return false;
        }

        return true;
    }

    void Hidraw::close()
    {

        device.reset();
    }

    bool Hidraw::isConnected() const
    {
        return device != nullptr;
    }

    int Hidraw::write(void *buf, size_t len)
    {
        if (!device)
        {
            LOG_WARN("Device is not connected");
            return iio::STATUS::ERROR;
        }

        int res = hid_write(device.get(), (unsigned char *)buf, len);

        if (res < 0)
        {
            LOG_ERROR("Failed to write data");
            return iio::STATUS::ERROR;
        }
        return res;
    }

    int Hidraw::read(void *buf, size_t len)
    {
        if (!device)
        {
            LOG_ERROR("Device is not connected");
            return iio::STATUS::ERROR;
        }

        int res = hid_read(device.get(), (unsigned char *)buf, len);
        if (res < 0)
        {
            LOG_ERROR("Failed to read data");
            return iio::STATUS::ERROR;
        }
        return res;
    }

    int Hidraw::readTimeout(void *buf, size_t len, int timeout)
    {
        if (!device)
        {
            LOG_ERROR("Device is not connected");
            return iio::STATUS::ERROR;
        }

        int res = hid_read_timeout(device.get(), (unsigned char *)buf, len, timeout);
        if (res < 0)
        {
            LOG_ERROR("Failed to read data");
            return iio::STATUS::ERROR;
        }
        return res;
    }

} // namespace HIDRAW

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(hidraw::Hidraw, iio::Bus)
