#pragma once
#include <stdint.h>
#include "type_traits"
#include <Arduino.h>

namespace my_helpers
{
    using def_counter_t = uint8_t;

#define EPSILON(T) (1./epsilon_t<T>::mul())
    template <typename T>
    struct epsilon_t
    {
        inline static T mul() { return 0;};
    };

    template <>
    struct epsilon_t<uint8_t>
    {
        inline static uint8_t mul(){ return 1;}
    };

    template <>
    struct epsilon_t<int>
    {
        inline static int mul(){ return 1;}
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
        epsilon_t<T> tmp;
        return (abs(v) <= EPSILON(T));
    }

    template <typename T>
    typename std::enable_if<std::is_unsigned<T>::value, int>::type
    inline constexpr signum(T x)
    {
        return T(0) < x;
    }

    template <typename T>
    typename std::enable_if<std::is_signed<T>::value, int>::type
    inline constexpr signum(T x)
    {
        return (T(0) < x) - (x < T(0));
    }


    template <typename T, def_counter_t buff_size, bool use_volatiles = false>
    class CBuffer
    {
    protected:
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
        void clear(T def)
        {
            for(counter_t i = 0; i < buff_size; ++i)
                *(buffer + i) = def;
            insPos = 0;
        }

        const T& back() const
        {
            return *(buffer + end());
        }

        const T& first() const
        {
            return *(buffer + begin());
        }

        counter_t begin() const
        {
            return nextIndex(insPos, 0);
        }

        counter_t end() const
        {
            return prevIndex(insPos);
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

        counter_t size() const
        {
            return buff_size;
        }

        void push_back_plain(T value)
        {
            buffer[insPos] = value;
            insPos = nextIndex(insPos);
        }

        T at(counter_t index) const
        {
            return buffer[nextIndex(index, 0)];
        }
        virtual ~CBuffer() = default;
    };

    template <typename T, bool use_volatiles = false, typename parent_t = CBuffer<T, 2, use_volatiles>>
    class LowPassFilter : public parent_t
    {
    public:

        LowPassFilter(T def)
        {
            parent_t::clear(def);
        }

        LowPassFilter()                                = delete;
        LowPassFilter(const LowPassFilter&)            = default;
        LowPassFilter(LowPassFilter&&)                 = default;
        LowPassFilter&operator=(const LowPassFilter&)  = default;
        LowPassFilter&operator=(LowPassFilter&&)       = default;
        ~LowPassFilter()                               = default;


        void push_back(T value, double alpha = 0.8)
        {
            parent_t::push_back_plain(static_cast<T>(parent_t::back() * alpha +  value * (1 - alpha)));
        }

        operator T() const
        {
            return parent_t::back();
        }
    };

    template <typename T, bool use_volatiles = false, typename parent_t = CBuffer<T, 3, use_volatiles>>
    class HighPassFilter : public parent_t
    {
    private:
        double alpha;
    public:

        HighPassFilter(T def, double alpha):
        alpha(alpha)
        {

            parent_t::clear(def);
        }

        HighPassFilter()                                = delete;
        HighPassFilter(const HighPassFilter&)           = default;
        HighPassFilter(HighPassFilter&&)                = default;
        HighPassFilter&operator=(const HighPassFilter&) = default;
        HighPassFilter&operator=(HighPassFilter&&)      = default;
        ~HighPassFilter()                               = default;

        void push_back(T value)
        {
            parent_t::push_back_plain(value);
        }

        operator T() const
        {
            T y = parent_t::first();
            for (typename parent_t::counter_t i = parent_t::begin() + 1, sz = parent_t::begin() + 1 + parent_t::size(); i < sz; ++i)
            {
                y = alpha * (y + parent_t::at(i) - parent_t::at(i - 1));
            }
            return y;
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