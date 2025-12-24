#include "servo_axis_factory.hpp"
#include <stdexcept>

std::shared_ptr<ServoAxisBase> ServoAxisFactory::create_servo_axis(
    DriveBrand brand, const std::string& name, uint16_t position, 
    AxisType axis_type, uint32_t product_code, double gear_ratio) {  // 更新函数签名    
    // 如果没有指定产品号，使用默认值
    if (product_code == 0) {
        product_code = get_default_product_code(brand);
    }
    
    switch (brand) {
        case DriveBrand::LEISAI:
            return std::make_shared<LeisaiServoAxis>(name, position, axis_type, product_code, gear_ratio);
        case DriveBrand::HUICHUAN:
            // 汇川使用固定产品号，忽略传入的product_code
            return std::make_shared<HuichuanServoAxis>(name, position, axis_type, gear_ratio);
        default:
            throw std::invalid_argument("未知的驱动器品牌");
    }
}

uint32_t ServoAxisFactory::get_default_product_code(DriveBrand brand) {
    switch (brand) {
        case DriveBrand::LEISAI:
            return LEISAI_PRODUCT_CODE_DEFAULT;
        case DriveBrand::HUICHUAN:
            return HUICHUAN_PRODUCT_CODE;
        default:
            return 0;
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