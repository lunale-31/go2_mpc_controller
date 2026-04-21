#pragma once

#include <cstdint>

namespace go2_utils::interface::lowlevel {
    /**
     * Battery management system state reported by the robot
     */
    struct BmsState {
        // Battery version
        uint8_t version_high, version_low;

        /**
         * Battery status values
         * @see https://support.unitree.com/home/en/developer/Basic_services
         */
        enum Status {
            OFF = 0,
            WAKE_UP = 1,
            PRE_CHARGE = 6,
            CHARGING = 7,
            DISCHARGING = 8,
            SELF_DISCHARGING = 9,
            ALARM = 11,
            RESET_ALARM = 12,
            AUTO_RECOVERY = 13
        };

        Status status;

        /**
         * Battery charge level
         * Given in percent, i.e. in interval [0, 100]
         */
        uint8_t charge_level;

        // Battery current: positive while charging, negative when discharging
        long current;

        // Number of charge cycle
        unsigned charge_cycle;

        using SharedPtr = std::shared_ptr<BmsState>;
    };
} // namespace interface::lowlevel