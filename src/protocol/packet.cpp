#include "protocol/packet.h"

const char* PacketTypeToString(PacketType type)
{
    switch (type)
    {
        case PKT_READY:      return "READY";
        case PKT_CHALLENGE:  return "CHALLENGE";
        case PKT_RESPONSE:   return "RESPONSE";
        case PKT_AUTH_OK:    return "AUTH_OK";
        case PKT_AUTH_FAIL:  return "AUTH_FAIL";
        case PKT_UNLOCK:     return "UNLOCK";
        case PKT_LOCK:       return "LOCK";
        case PKT_ERROR:      return "ERROR";
        default:             return "UNKNOWN";
    }
}