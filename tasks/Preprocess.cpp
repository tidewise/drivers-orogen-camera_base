/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Preprocess.hpp"

using namespace camera_base;
using namespace base::samples::frame;

Preprocess::Preprocess(std::string const& name)
    : PreprocessBase(name)
    , oframe(new Frame())
{
}

Preprocess::Preprocess(std::string const& name, RTT::ExecutionEngine* engine)
    : PreprocessBase(name, engine)
    , oframe(new Frame())
{
}

Preprocess::~Preprocess()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Preprocess.hpp for more detailed
// documentation about them.
bool Preprocess::configureHook()
{
    if (!PreprocessBase::configureHook())
        return false;
    if (_undistort.value())
        frame_helper.setCalibrationParameter(_calibration_parameters.value());
    return true;
}

bool Preprocess::startHook()
{
    if (!PreprocessBase::startHook())
        return false;
    return true;
}

void Preprocess::updateHook()
{
    PreprocessBase::updateHook();
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> iframe;
    if (_iframe.read(iframe) != RTT::NewData)
        return;

    Frame* frame_ptr = oframe.write_access();
    frame_ptr->init(iframe->size.width * _scale_x - _offset_x,
        iframe->size.height * _scale_y - _offset_y,
        iframe->getDataDepth(),
        _format);
    try {
        frame_helper.convert(*iframe,
            *frame_ptr,
            _offset_x.value(),
            _offset_y.value(),
            _resize_algorithm.value(),
            _undistort.value(),
            _undistort_algorithm.value());
    }
    catch (std::runtime_error e) {
        RTT::log(RTT::Error) << "processing error: " << e.what() << RTT::endlog();
        if (state() != PROCESSING_ERROR) {
            state(PROCESSING_ERROR);
        }
    }

    oframe.reset(frame_ptr);
    _oframe.write(oframe);
}

void Preprocess::errorHook()
{
    PreprocessBase::errorHook();
}

void Preprocess::stopHook()
{
    PreprocessBase::stopHook();
}

void Preprocess::cleanupHook()
{
    PreprocessBase::cleanupHook();
}
