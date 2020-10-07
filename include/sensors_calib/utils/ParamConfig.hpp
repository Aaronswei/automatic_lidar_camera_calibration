/**
 * @file    ParamConfig.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <fstream>
#include <optional>
#include <sstream>
#include <string>

#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>

namespace perception
{
template <typename ParamType> ParamType fromJson(const rapidjson::Document& rapidjsonDoc)
{
    throw std::runtime_error("Not supported param type");
}

template <typename VALUE_TYPE> VALUE_TYPE getValueAs(const rapidjson::Document& rapidjsonDoc, const std::string& key)
{
    const rapidjson::Value& node = rapidjsonDoc[key.c_str()];
    if (node.IsNull()) {
        throw std::runtime_error(key + " does not exist");
    }

    if (!node.Is<VALUE_TYPE>()) {
        throw std::runtime_error("mismatch data type: " + key);
    }

    return node.Get<VALUE_TYPE>();
}

inline rapidjson::Document readFromJsonFile(const std::string& jsonPath)
{
    std::ifstream inFile;
    inFile.open(jsonPath);
    if (!inFile.is_open()) {
        throw std::runtime_error("Failed to open json file");
    }

    rapidjson::IStreamWrapper isw(inFile);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError()) {
        std::stringstream ss;
        ss << "error offset:" << doc.GetErrorOffset() << std::endl;
        ss << "error pase:" << rapidjson::GetParseError_En(doc.GetParseError()) << std::endl;
        throw std::runtime_error(ss.str());
    }

    return doc;
}
}  // namespace perception