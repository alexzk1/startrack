#pragma once
#include <stdint.h>
#include "type_traits"
#include <Arduino.h>

namespace my_helpers
{
    using circular_counter_t = uint8_t;

    template <typename T>
    struct epsilon_t
    {
        inline static T mul() { return 0;};
        inline static T epsilon()
        {
            return 1./ static_cast<T>(mul);
        }
    };

    template <>
    struct epsilon_t<uint8_t>
    {
        inline static uint8_t epsilon(){ return 0;}
    };

    template <>
    struct epsilon_t<int>
    {
        inline static int epsilon(){ return 0;}
    };


    template <>
    struct  epsilon_t<float>
    {
        inline static float mul() { return  10000000.f;}
    };

    template <>
    struct  epsilon_t<double >
    {
        inline static double mul() { return  10000000000.;}
    };

    template <class T>
    bool isZero(T v)
    {
        return (abs(v) <= epsilon_t<T>::epsilon());
    }

    //not really circular buffer yet, but calcs avr. value and can do simple low-pass
    template <typename T, circular_counter_t buff_size = 3, bool use_volatiles = false>
    class Circular
    {
    private:
        using counter_t = decltype(buff_size);
        typename std::conditional<use_volatiles, volatile counter_t, counter_t>::type insPos;
        typename std::conditional<use_volatiles, volatile T, T>::type buffer[buff_size];

        static counter_t nextIndex(counter_t index, counter_t delta = 1)
        {
            return (index + delta) % buff_size;
        }
        static counter_t prevIndex(counter_t index, counter_t delta = 1)
        {
            //operation "minus" using overflow by mod "buff_size"
            delta %= buff_size; //that we need, because holder type can be unsigned
            return nextIndex(index, buff_size - delta);
        }
    public:
        using values_type = T;
        Circular(T def)
        {
            clear(def);
        }

        Circular()                           = delete;
        Circular(const Circular&)            = default;
        Circular(Circular&&)                 = default;
        Circular&operator=(const Circular&)  = default;
        Circular&operator=(Circular&&)       = default;
        ~Circular()                          = default; //yep, yep, no virtuals on MK!

        void clear(T def)
        {
            for(counter_t i = 0; i < buff_size; ++i)
                *(buffer + i) = def;
            insPos = 0;
        }

        void push_back(T value)
        {
            buffer[insPos] = value;
            insPos = nextIndex(insPos);
        }

        void push_back_lpf(T value, double alpha =0.8)
        {
            push_back(static_cast<T>(value + alpha *(back() - value)));
        }

        const T& back() const
        {
            return *(buffer + prevIndex(insPos));
        }

        T avr() const
        {
            T summ = 0;
            for(counter_t  i = 0; i < buff_size; ++i)
                summ += *(buffer + i);
            return summ / static_cast<T>(buff_size);
        }

        T lastDelta() const
        {
            auto excl = prevIndex(insPos);
            T summ = 0;
            for(counter_t  i = 0; i < buff_size; ++i)
                if (i != excl)
                summ += *(buffer + i);

            return (summ / (buff_size - 1)) - back();
        }

        operator T() const
        {
            return avr();
        }
    };


    template <class T>
    T removeRotRad(T value)
    {
        const static int64_t mpi = 2 * M_PI * epsilon_t<T>::mul();
        return (static_cast<decltype(mpi)>(value  * epsilon_t<T>::mul()) % mpi) / epsilon_t<T>::mul();
    }

    template <typename T, typename P = typename std::conditional<std::is_arithmetic<T>::value, T, const T&>::type>
    inline T sqr(P x)
    {
        return x * x;
    }

    struct no_interrupts
    {
        no_interrupts()
        {
            noInterrupts();
        }
        ~no_interrupts()
        {
            interrupts();
        }
    };
}

#define sqrf(T) my_helpers::sqr<float>(T)
#define sqrd(T) my_helpers::sqr<double>(T)