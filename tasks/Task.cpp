/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace camera_base;
using namespace base::samples::frame;

Task::Task(std::string const& name)
    : TaskBase(name),cam_interface(NULL),process_image(false),
      stat_frame_rate(0),stat_invalid_frame_rate(0),stat_valid_frame_rate(0)

{

}

Task::~Task()
{
    if(cam_interface)
    {
        cam_interface->close();
        delete cam_interface;
        cam_interface = NULL;
    }
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    invalid_frames_count = 0;
    valid_frames_count = 0;
    time_save = base::Time::now();

    //test if we have to process the frame before writing it to the output port
    frame_mode_t output_frame_mode = _output_format.value();
    if(output_frame_mode == MODE_UNDEFINED)
        output_frame_mode = _camera_format.value();

    if(_undistort.value() || _scale_x.value() != 1 || _scale_y.value() != 1 || _offset_x.value() != 0 || _offset_y.value() != 0 ||
       _camera_format.value() != output_frame_mode)
    {
        if(_undistort.value())
            frame_helper.setCalibrationParameter(_calibration_parameters);
        process_image = true;
    }
    else
        process_image = false;

    //initialize camera frame
    Frame* frame = new Frame(_width,_height,8,_camera_format.value()); 
    camera_frame.reset(frame);

    //initialize output frame
    frame = new Frame(_width*_scale_x,_height*_scale_y,8,output_frame_mode); 
    output_frame.reset(frame);	
    frame = NULL;

    //configure and start camera
    try 
    {
        RTT::log(RTT::Info) << "configure camera" << RTT::endlog(); 

        //this function must be defined in the derived class
        //to configure the camera
        setCameraSettings();

        //check if the camera is initialized
        if(cam_interface == NULL)
        {
            RTT::log(RTT::Error) << "Camera Driver Error: No camera interface was initialized" << RTT::endlog();
            fatal(NO_CAMERA_INTERFACE);
            return false;
        }

        RTT::log(RTT::Info) << cam_interface->doDiagnose() << RTT::endlog();
        cam_interface->grab(camera::Continuously,_frame_buffer_size); 
    }
    catch(std::runtime_error e)
    {
        RTT::log(RTT::Error) << "failed to start camera: " << e.what() << RTT::endlog();
        error(CANNOT_START_GRABBING);
        return false;
    }
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(cam_interface->isFrameAvailable())
    {
        Frame *frame_ptr = camera_frame.write_access();
        try
        {
            cam_interface->retrieveFrame(*frame_ptr);
        }
        catch(std::runtime_error e)
        { 
            RTT::log(RTT::Warning) << "failed to retrieve frame: " << e.what() << RTT::endlog();
            if(_clear_buffer_if_frame_drop)
                cam_interface->skipFrames();
        }

        if (frame_ptr->getStatus() == STATUS_VALID)
        {
            valid_frames_count++;
            //set extra attributes 
            setExtraAttributes(frame_ptr);
            camera_frame.reset(frame_ptr);

            //check if we have to process the frame before writing it to the port
            if(process_image)
            {
                frame_ptr = output_frame.write_access();
                try 
                {
                    frame_helper.convert(*camera_frame,*frame_ptr,_offset_x.value(),
                            _offset_y.value(),_resize_algorithm.value(),_undistort.value());
                }
                catch(std::runtime_error e)
                {
                    RTT::log(RTT::Error) << "processing error: " << e.what() << RTT::endlog();
                    RTT::log(RTT::Error) << "Have you specified camera_format and output_format right?" << RTT::endlog();
                    error(PROCESSING_ERROR);
                    return;
                }
                output_frame.reset(frame_ptr);
                _frame.write(output_frame);
            }
            else
                _frame.write(camera_frame);

            if(!_disable_frame_raw)
                _frame_raw.write(camera_frame);
        }
        else
        {
            camera_frame.reset(frame_ptr);
            invalid_frames_count++;
        }
    }
    if (cam_interface->isFrameAvailable())
        this->getActivity()->trigger();
}

//void Task::errorHook()
//{
//    TaskBase::errorHook();
//}

void Task::stopHook()
{
    TaskBase::stopHook();
    RTT::log(RTT::Info) << "stop grabbing" << RTT::endlog();
    cam_interface->grab(camera::Stop);
    sleep(1);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    if(cam_interface)
    {
        cam_interface->close();
        delete cam_interface;
        cam_interface = NULL;
    }
}

void Task::setExtraAttributes(Frame *frame_ptr)
{
    if(_log_interval_in_sec)
    {
        base::Time time = base::Time::now();
        float time_diff = (time - time_save).toSeconds();

        if(time_diff > _log_interval_in_sec)
        {
            stat_valid_frame_rate  = ((float)valid_frames_count)/time_diff;
            stat_invalid_frame_rate  = ((float)invalid_frames_count)/time_diff;
            //  time_save_ = time;
            valid_frames_count = 0;
            invalid_frames_count = 0;
            //get camera frame rate
            if(cam_interface->isAttribAvail(camera::double_attrib::StatFrameRate))
                stat_frame_rate = cam_interface->getAttrib(camera::double_attrib::StatFrameRate);
        }

        frame_ptr->setAttribute<float>("StatFps",stat_frame_rate);
        frame_ptr->setAttribute<float>("StatValidFps",stat_valid_frame_rate);
        frame_ptr->setAttribute<float>("StatInValidFps",stat_invalid_frame_rate);
    }
    else  //prevents overflow after > 800 days
    {
        valid_frames_count = 0;
        invalid_frames_count = 0;
    }
}


//methods interface for the orocos module
bool Task::setDoubleAttrib(camera::double_attrib::CamAttrib const & type, double value)
{
    try
    {
        cam_interface->setAttrib(type,value);
    }
    catch(std::runtime_error e)
    {
        return false;
    }
    return true;
}

bool Task::setEnumAttrib(camera::enum_attrib::CamAttrib const & type)
{
    try
    {
        cam_interface->setAttrib(type);
    }
    catch(std::runtime_error e)
    {
        return false;
    }
    return true;
}

bool Task::setIntAttrib(camera::int_attrib::CamAttrib const & type, int value)
{
    try
    {
        cam_interface->setAttrib(type,value);
    }
    catch(std::runtime_error e)
    {
        return false;
    }
    return true;
}

bool Task::setStringAttrib(camera::str_attrib::CamAttrib const & type, std::string const & value)
{
    try
    {
        cam_interface->setAttrib(type,value);
    }
    catch(std::runtime_error e)
    {
        return false;
    }
    return true;
}

double Task::getDoubleAttrib(camera::double_attrib::CamAttrib const & type)
{
    try
    {
        return cam_interface->getAttrib(type);
    }
    catch(std::runtime_error e)
    {
        return -1;
    }
    return -1;
}

bool Task::isEnumAttribSet(camera::enum_attrib::CamAttrib const & type)
{
    try
    {
        return cam_interface->isAttribSet(type);
    }
    catch(std::runtime_error e)
    {
        return false;
    }
    return false;
}

int Task::getIntAttrib(camera::int_attrib::CamAttrib const & type)
{
    try
    {
        return cam_interface->getAttrib(type);
    }
    catch(std::runtime_error e)
    {
        return -1;
    }
    return -1;
}

std::string Task::getStringAttrib(camera::str_attrib::CamAttrib const & type)
{
    try
    {
        return cam_interface->getAttrib(type);
    }
    catch(std::runtime_error e)
    {
        return "";
    }
    return "";
}

boost::int32_t Task::getIntRangeMin(camera::int_attrib::CamAttrib const & type)
{
    try
    {
        int imax,imin;
        cam_interface->getRange(type,imin,imax);
        return imin;
    }
    catch(std::runtime_error e)
    {
        return -1;
    }
    return -1;
}

boost::int32_t Task::getIntRangeMax(camera::int_attrib::CamAttrib const & type)
{
    try
    {
        int imax,imin;
        cam_interface->getRange(type,imin,imax);
        return imax;
    }
    catch(std::runtime_error e)
    {
        return -1;
    }
    return -1;
}

double Task::getDoubleRangeMin(camera::double_attrib::CamAttrib const & type)
{
    try
    {
        double dmax,dmin;
        cam_interface->getRange(type,dmin,dmax);
        return dmin;
    }
    catch(std::runtime_error e)
    {
        return -1;
    }
    return -1;
}

double Task::getDoubleRangeMax(camera::double_attrib::CamAttrib const & type)
{
    try
    {
        double dmax,dmin;
        cam_interface->getRange(type,dmin,dmax);
        return dmax;
    }
    catch(std::runtime_error e)
    {
        return -1;
    }
    return -1;
}
