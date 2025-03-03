#ifndef __HIDRAW_HPP__
#define __HIDRAW_HPP__

#include <vector>
#include <cstdint>
#include <memory>
#include "hidraw/visibility_control.h"
#include "hidapi/hidapi.h"
#include "iio/iio.hpp"

namespace hidraw
{

  class Hidraw : public iio::Bus
  {
  public:
    virtual ~Hidraw();
    int write(void *buf, size_t len) override;
    int read(void *buf, size_t len) override;
    int readTimeout(void *buf, size_t len, int timeout) override;

    bool openDevice(uint16_t vendor_id, uint16_t product_id);
    void close();
    bool isConnected() const;

  private:
    std::unique_ptr<hid_device, decltype(&hid_close)> device{nullptr, &hid_close};
  };

} // namespace hidraw

#endif // __HIDRAW_HPP__
