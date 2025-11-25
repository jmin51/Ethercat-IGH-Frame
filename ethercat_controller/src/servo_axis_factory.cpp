#include "servo_axis_factory.hpp"
#include <stdexcept>

std::unique_ptr<ServoAxisBase> ServoAxisFactory::create_servo_axis(
    DriveBrand brand, const std::string& name, uint16_t position, AxisType axis_type) {
    
    switch (brand) {
        case DriveBrand::LEISAI:
            return std::make_unique<LeisaiServoAxis>(name, position, axis_type);
        case DriveBrand::HUICHUAN:
            return std::make_unique<HuichuanServoAxis>(name, position, axis_type);
        default:
            throw std::invalid_argument("未知的驱动器品牌");
    }
}

DriveBrand ServoAxisFactory::string_to_brand(const std::string& brand_str) {
    if (brand_str == "leisai" || brand_str == "LEISAI" || brand_str == "雷赛") {
        return DriveBrand::LEISAI;
    } else if (brand_str == "huichuan" || brand_str == "HUICHUAN" || brand_str == "汇川") {
        return DriveBrand::HUICHUAN;
    } else {
        throw std::invalid_argument("不支持的驱动器品牌: " + brand_str);
    }
}

std::string ServoAxisFactory::brand_to_string(DriveBrand brand) {
    switch (brand) {
        case DriveBrand::LEISAI: return "LEISAI";
        case DriveBrand::HUICHUAN: return "HUICHUAN";
        default: return "UNKNOWN";
    }
}