#ifndef FATAL_ERR
#define FATAL_ERR

#include <string>
#include <exception>
#include <stdexcept>
#include <iostream>

class sigsegv_exception: public std::exception
{
public:
    virtual const char *what() const noexcept override
    {
        return "SIGSEGV, possible access to unlocked vm.";
    }
};

class fatal_exception : public std::runtime_error
{
public:
    explicit fatal_exception(const std::string& msg)
        :std::runtime_error(msg)
    {
        std::cerr << "FATAL_RISE: "<<msg<<std::endl;
    }
};

#define FATAL_RISE(TEXT) {auto EXCPT = fatal_exception(std::string(TEXT)+"\n\tat "+std::string(__FILE__)+": "+std::to_string(__LINE__)); throw EXCPT;}



#endif // FATAL_ERR

