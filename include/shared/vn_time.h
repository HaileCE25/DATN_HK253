#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

// -----------------------------------------------------------------------------
// Giờ Việt Nam (UTC+7) - chỉ để HIỂN THỊ log. Trên dây và trong logic luôn
// dùng Unix time UTC.
// -----------------------------------------------------------------------------

constexpr int32_t VN_UTC_OFFSET_SEC = 7 * 3600;

// Ghi "YYYY-MM-DD HH:MM:SS" (giờ Việt Nam) vào buf; buf nên >= 20 byte.
inline void VnTime_Format(uint32_t unixUtc, char* buf, size_t bufSize)
{
    int64_t t = (int64_t)unixUtc + VN_UTC_OFFSET_SEC;
    int64_t days = t / 86400;
    int32_t secOfDay = (int32_t)(t % 86400);

    // civil_from_days (Howard Hinnant)
    int64_t z = days + 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    int64_t doe = z - era * 146097;
    int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int64_t mp = (5 * doy + 2) / 153;
    int day = (int)(doy - (153 * mp + 2) / 5 + 1);
    int month = (int)(mp < 10 ? mp + 3 : mp - 9);
    int year = (int)(yoe + era * 400 + (month <= 2 ? 1 : 0));

    snprintf(buf, bufSize, "%04d-%02d-%02d %02d:%02d:%02d",
             year, month, day, secOfDay / 3600, (secOfDay % 3600) / 60, secOfDay % 60);
}
