#ifndef STELL_MSG_H
#define STELL_MSG_H

#include <stdint.h>
union StellBasicMessage
{
    struct Msg
    {
        uint16_t type;
        uint64_t clientMicros;
        uint32_t ra_int;
        uint32_t dec_int;
    } __attribute__((packed)) msg;
    char buffer[sizeof (Msg)];
}__attribute__((packed));

#endif // STELL_MSG_H
