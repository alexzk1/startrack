#ifndef STELL_MSG_H
#define STELL_MSG_H

#include <stdint.h>
union StellBasicMessage
{
    struct Msg
    {
        uint16_t type;
        int64_t clientMicros;
        uint32_t ra_int;
        int32_t dec_int;
    } __attribute__((packed)) msg;
    char buffer[sizeof (Msg)];
}__attribute__((packed));


union ToStellMessage
{
    struct Msg
    {
        uint16_t size; //24
        uint16_t type;
        int64_t clientMicros;
        uint32_t ra_int;
        int32_t dec_int;
        int32_t status; //0
    }__attribute__((packed)) msg;
    char buffer[sizeof (Msg)];
}__attribute__((packed));

#endif // STELL_MSG_H
