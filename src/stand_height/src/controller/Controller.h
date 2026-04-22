#pragma once

#include <rclcpp/rclcpp.hpp>
#include <stand_height/srv/stand_height.hpp>

class Controller : public rclcpp::Node {
    private:
        rclcpp::Service<stand_height::srv::StandHeight>::SharedPtr service_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        Controller();

        void timer_tick();

        void service_request(const stand_height::srv::StandHeight::Request::SharedPtr request,
                          stand_height::srv::StandHeight::Response::SharedPtr response);
};