#ifndef _UTILITIES_HPP_
#define _UTILITIES_HPP_

#include "types.hpp"

void resetTimer(rclcpp::TimerBase::SharedPtr&);
void cancelTimer(rclcpp::TimerBase::SharedPtr&);
template<typename T> void resetSharedPointer(std::shared_ptr<T>&);

#include "templates.tpp"

#endif
