//
// Created by alex on 5/7/17.
//

#ifndef ARDUINO_ST_SIMPLE_COMM_H
#define ARDUINO_ST_SIMPLE_COMM_H

#include <stdint.h>

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error Only little-endian is implemented, consider to update code.
#endif

#define COMM_MULL 100000
#define MESSAGE_HDR "MSG"




namespace ard_st
{
    union Packer
    {
        struct vals_t
        {
            int32_t az;
            int32_t el;
        } vals;
        char   buffer[8];

    }__attribute__((packed));

    union Message
    {

        struct Msg
        {
            char   Command;
            Packer value;
        } message;
        char buffer[9];
    }__attribute__((packed));



    inline void packAzEl(Packer& tmp, float azv, float elv)
    {
        static_assert(sizeof(Packer) == 8, "Something wrong with compiler");
        tmp.vals.az = static_cast<decltype(tmp.vals.az)>(static_cast<float>(COMM_MULL) * azv);
        tmp.vals.el = static_cast<decltype(tmp.vals.el)>(static_cast<float>(COMM_MULL) * elv);
    };

    inline void readAzEl(const Packer& value, float *azv, float *elv)
    {
        static_assert(sizeof(Packer) == 8, "Something wrong with compiler");
        *azv = static_cast<float>(value.vals.az) / static_cast<float>(COMM_MULL);
        *elv = static_cast<float>(value.vals.el) / static_cast<float>(COMM_MULL);
    }
}
#endif //ARDUINO_ST_SIMPLE_COMM_H
