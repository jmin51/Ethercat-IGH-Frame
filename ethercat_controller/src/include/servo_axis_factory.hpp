#ifndef SERVO_AXIS_FACTORY_HPP
#define SERVO_AXIS_FACTORY_HPP

#include "servo_axis_base.hpp"
#include "leisai_servo_axis.hpp"
#include "huichuan_servo_axis.hpp"
#include <memory>
#include <string>

class ServoAxisFactory {
public:
    static std::unique_ptr<ServoAxisBase> create_servo_axis(
        DriveBrand brand,
        const std::string& name,
        uint16_t position,
        AxisType axis_type = AxisType::AXIS1
    );
    
    static DriveBrand string_to_brand(const std::string& brand_str);
    static std::string brand_to_string(DriveBrand brand);
};

#endif // SERVO_AXIS_FACTORY_HPP