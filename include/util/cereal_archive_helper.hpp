// Copyright (c) 2023. Created on 9/16/23 10:31 AM by shlchen@whu.edu.cn (Shuolong Chen), who received the B.S. degree in
// geodesy and geomatics engineering from Wuhan University, Wuhan China, in 2023. He is currently a master candidate at
// the school of Geodesy and Geomatics, Wuhan University. His area of research currently focuses on integrated navigation
// systems and multi-sensor fusion.

#ifndef RIVER_CEREAL_ARCHIVE_HELPER_HPP
#define RIVER_CEREAL_ARCHIVE_HELPER_HPP

#include "util/cereal_yaml.hpp"
#include "variant"
#include "cereal/archives/json.hpp"
#include "cereal/archives/xml.hpp"
#include "cereal/cereal.hpp"
#include "cereal/archives/binary.hpp"
#include "memory"

namespace ns_river {

    struct CerealArchiveType {
        enum class Enum {
            JSON = 0, XML = 1, YAML = 2, BINARY = 3
        };
        using InputArchiveVariant = std::variant<
                std::shared_ptr<cereal::JSONInputArchive>,
                std::shared_ptr<cereal::XMLInputArchive>,
                std::shared_ptr<cereal::YAMLInputArchive>,
                std::shared_ptr<cereal::BinaryInputArchive>>;

        using OutputArchiveVariant = std::variant<
                std::shared_ptr<cereal::JSONOutputArchive>,
                std::shared_ptr<cereal::XMLOutputArchive>,
                std::shared_ptr<cereal::YAMLOutputArchive>,
                std::shared_ptr<cereal::BinaryOutputArchive>>;

        struct JSON {
        };
        struct XML {
        };
        struct YAML {
        };
        struct BINARY {
        };
    };


    template<class Type>
    struct CerealArchiveTypeExtractor {
        using InputArchive = CerealArchiveType;
        using OutputArchive = CerealArchiveType;
    };

    template<>
    struct CerealArchiveTypeExtractor<CerealArchiveType::JSON> {
        using InputArchive = cereal::JSONInputArchive;
        using OutputArchive = cereal::JSONOutputArchive;
    };

    template<>
    struct CerealArchiveTypeExtractor<CerealArchiveType::XML> {
        using InputArchive = cereal::XMLInputArchive;
        using OutputArchive = cereal::XMLOutputArchive;
    };

    template<>
    struct CerealArchiveTypeExtractor<CerealArchiveType::YAML> {
        using InputArchive = cereal::YAMLInputArchive;
        using OutputArchive = cereal::YAMLOutputArchive;
    };

    template<>
    struct CerealArchiveTypeExtractor<CerealArchiveType::BINARY> {
        using InputArchive = cereal::BinaryInputArchive;
        using OutputArchive = cereal::BinaryOutputArchive;
    };


    template<class ArchiveType>
    static inline std::shared_ptr<typename CerealArchiveTypeExtractor<ArchiveType>::InputArchive>
    GetInputArchive(std::ifstream &file) {
        return std::make_shared<typename CerealArchiveTypeExtractor<ArchiveType>::InputArchive>(file);
    }

    template<class ArchiveType>
    static inline std::shared_ptr<typename CerealArchiveTypeExtractor<ArchiveType>::OutputArchive>
    GetOutputArchive(std::ofstream &file) {
        return std::make_shared<typename CerealArchiveTypeExtractor<ArchiveType>::OutputArchive>(file);
    }

    static inline CerealArchiveType::InputArchiveVariant
    GetInputArchiveVariant(std::ifstream &file, CerealArchiveType::Enum archiveType) {
        switch (archiveType) {
            case CerealArchiveType::Enum::JSON:
                return std::make_shared<cereal::JSONInputArchive>(file);
            case CerealArchiveType::Enum::XML:
                return std::make_shared<cereal::XMLInputArchive>(file);
            case CerealArchiveType::Enum::YAML:
                return std::make_shared<cereal::YAMLInputArchive>(file);
            case CerealArchiveType::Enum::BINARY:
                return std::make_shared<cereal::BinaryInputArchive>(file);
            default:
                throw std::runtime_error("unknown cereal archive type!");
        }
    }

    static inline CerealArchiveType::OutputArchiveVariant
    GetOutputArchiveVariant(std::ofstream &file, CerealArchiveType::Enum archiveType) {
        switch (archiveType) {
            case CerealArchiveType::Enum::JSON:
                return std::make_shared<cereal::JSONOutputArchive>(file);
            case CerealArchiveType::Enum::XML:
                return std::make_shared<cereal::XMLOutputArchive>(file);
            case CerealArchiveType::Enum::YAML:
                return std::make_shared<cereal::YAMLOutputArchive>(file);
            case CerealArchiveType::Enum::BINARY:
                return std::make_shared<cereal::BinaryOutputArchive>(file);
            default:
                throw std::runtime_error("unknown cereal archive type!");
        }
    }

    template<class ... Types>
    static inline void SerializeByInputArchiveVariant(const CerealArchiveType::InputArchiveVariant &ar,
                                                      CerealArchiveType::Enum archiveType, Types &&... args) {
        switch (archiveType) {
            case CerealArchiveType::Enum::JSON:
                (*std::get<std::shared_ptr<cereal::JSONInputArchive >>(ar))(args...);
                break;
            case CerealArchiveType::Enum::XML:
                (*std::get<std::shared_ptr<cereal::XMLInputArchive >>(ar))(args...);
                break;
            case CerealArchiveType::Enum::YAML:
                (*std::get<std::shared_ptr<cereal::YAMLInputArchive >>(ar))(args...);
                break;
            case CerealArchiveType::Enum::BINARY:
                (*std::get<std::shared_ptr<cereal::BinaryInputArchive >>(ar))(args...);
                break;
            default:
                throw std::runtime_error("unknown cereal archive type!");
        }
    }

    template<class ... Types>
    static inline void SerializeByOutputArchiveVariant(const CerealArchiveType::OutputArchiveVariant &ar,
                                                       CerealArchiveType::Enum archiveType, Types &&... args) {
        switch (archiveType) {
            case CerealArchiveType::Enum::JSON:
                (*std::get<std::shared_ptr<cereal::JSONOutputArchive >>(ar))(args...);
                break;
            case CerealArchiveType::Enum::XML:
                (*std::get<std::shared_ptr<cereal::XMLOutputArchive >>(ar))(args...);
                break;
            case CerealArchiveType::Enum::YAML:
                (*std::get<std::shared_ptr<cereal::YAMLOutputArchive >>(ar))(args...);
                break;
            case CerealArchiveType::Enum::BINARY:
                (*std::get<std::shared_ptr<cereal::BinaryOutputArchive >>(ar))(args...);
                break;
            default:
                throw std::runtime_error("unknown cereal archive type!");
        }
    }
}

#endif //RIVER_CEREAL_ARCHIVE_HELPER_HPP
