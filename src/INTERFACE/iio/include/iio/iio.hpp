#ifndef __IIO_HPP__
#define __IIO_HPP__
#include <stddef.h>
#include <iostream>
#include "log/log.hpp"

/*
 * This is the interface class for Input/Output operations.
 */
namespace iio
{
    enum STATUS
    {
        ERROR = -1
    };
    class Bus
    {
    public:
        virtual ~Bus() = default;
        virtual int write(void *buf, size_t len) = 0;
        virtual int read(void *buf, size_t len) = 0;
        virtual int readTimeout(void *buf, size_t len, int timeout) = 0;
    };
} // namespace IIO

#endif // __IIO_HPP__