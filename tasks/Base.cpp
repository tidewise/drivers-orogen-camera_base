/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Base.hpp"

#include <opencv2/core/utility.hpp>

using namespace camera_base;

Base::Base(std::string const& name, TaskCore::TaskState initial_state)
    : BaseBase(name, initial_state)
{
}

Base::~Base()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Base.hpp for more detailed
// documentation about them.

bool Base::configureHook()
{
    if (!BaseBase::configureHook())
        return false;

    cv::setNumThreads(_opencv_num_threads.get());
    return true;
}
bool Base::startHook()
{
    if (!BaseBase::startHook())
        return false;
    return true;
}
void Base::updateHook()
{
    BaseBase::updateHook();
}
void Base::errorHook()
{
    BaseBase::errorHook();
}
void Base::stopHook()
{
    BaseBase::stopHook();
}
void Base::cleanupHook()
{
    BaseBase::cleanupHook();
}
