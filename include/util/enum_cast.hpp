// Copyright (c) 2023. Created on 7/7/23 1:29 PM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_ENUM_CAST_HPP
#define RIVER_ENUM_CAST_HPP

#include "exception"
#include "magic_enum.hpp"
#include "string"

namespace ns_river {

    struct EnumCast {
        template<class EnumType>
        static constexpr auto stringToEnum(const std::string &enumStr) {
            if (auto color = magic_enum::enum_cast<EnumType>(enumStr);color.has_value()) {
                return color.value();
            } else {
                throw std::runtime_error("'EnumCast::stringToEnum' cast failed");
            }
        }

        template<class EnumType>
        static constexpr auto integerToEnum(int enumValue) {
            if (auto color = magic_enum::enum_cast<EnumType>(enumValue);color.has_value()) {
                return color.value();
            } else {
                throw std::runtime_error("'EnumCast::integerToEnum' cast failed");
            }
        }

        template<class EnumType>
        static constexpr auto enumToInteger(EnumType enumType) {
            return magic_enum::enum_integer(enumType);
        }

        template<class EnumType>
        static constexpr auto enumToString(EnumType enumType) {
            return magic_enum::enum_name(enumType);
        }

        template<class EnumType>
        static constexpr auto stringToInteger(const std::string &enumStr) {
            return enumToInteger(stringToEnum<EnumType>(enumStr));
        }

        template<class EnumType>
        static constexpr auto integerToString(int enumValue) {
            return enumToString(integerToEnum<EnumType>(enumValue));
        }
    };

}

#endif //RIVER_ENUM_CAST_HPP
