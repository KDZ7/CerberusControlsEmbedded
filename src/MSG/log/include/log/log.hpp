#ifndef __LOG_HPP__
#define __LOG_HPP__

#if defined(__DEBUG__)
#define LOG_INFO(x) std::cout << "[TRACE INFO]:" << x << std::endl
#define LOG_ERROR(x) std::cerr << "[TRACE ERROR]:" << x << std::endl
#define LOG_WARN(x) std::cerr << "[TRACE WARN]:" << x << std::endl

#else
#define LOG_INFO(x)
#define LOG_ERROR(x)
#define LOG_WARN(x)
#endif

#endif // __LOG_HPP__