//
// Created by jugo on 11.03.25.
//

#ifndef VELODYNE_POINTCLOUD_RETURN_TYPES_FLAGS_H
#define VELODYNE_POINTCLOUD_RETURN_TYPES_FLAGS_H

#include <cstdint>

namespace velodyne_rawdata
{
// per point return type 3 Bits first return flag(strongest), is last flag(last), is unique return flag
    static const uint8_t VLS128_RETURN_TYPE_STRONGEST = 4;
    static const uint8_t VLS128_RETURN_TYPE_LAST = 2;
    static const uint8_t VLS128_RETURN_TYPE_STRONGEST_LAST = 6;
    static const uint8_t VLS128_RETURN_TYPE_SECOND_STRONGEST = 0;
    static const uint8_t VLS128_RETURN_TYPE_STRONGEST_LAST_UNIQUE = 7;

    static const uint8_t VLS128_RT_S = VLS128_RETURN_TYPE_STRONGEST;
    static const uint8_t VLS128_RT_L = VLS128_RETURN_TYPE_LAST;
    static const uint8_t VLS128_RT_SL = VLS128_RETURN_TYPE_STRONGEST_LAST;
    static const uint8_t VLS128_RT_SS = VLS128_RETURN_TYPE_SECOND_STRONGEST;
    static const uint8_t VLS128_RT_SLU = VLS128_RETURN_TYPE_STRONGEST_LAST_UNIQUE;

    constexpr uint8_t FIRST_RETURN_FLAG = (1 << 2);  // Bit 2
    constexpr uint8_t LAST_RETURN_FLAG = (1 << 1);  // Bit 1
    constexpr uint8_t UNIQUE_RETURN_FLAG = (1 << 0);  // Bit 0

    inline bool is_flag_set(const uint8_t value, const uint8_t flag)
    {
        return (value & flag) != 0;
    }
}
#endif //VELODYNE_POINTCLOUD_RETURN_TYPES_FLAGS_H
