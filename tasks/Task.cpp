/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp>

using namespace camera_base;
using namespace camera;
using namespace base::samples::frame;

Task::Task(std::string const& name)
    : TaskBase(name),cam_interface(NULL),process_image(false),
      stat_frame_rate(0),stat_invalid_frame_rate(0),stat_valid_frame_rate(0)

{

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine) : 
	TaskBase(name, engine)
	,cam_interface(NULL)
	,process_image(false)
	,stat_frame_rate(0)
	,stat_invalid_frame_rate(0)
	,stat_valid_frame_rate(0)
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
    Frame* frame = new Frame(_width,_height,_channel_data_depth,_camera_format.value()); 
    camera_frame.reset(frame);

    //initialize output frame
    frame = new Frame(_width*_scale_x,_height*_scale_y,_channel_data_depth,output_frame_mode); 
    output_frame.reset(frame);	
    frame = NULL;

    //configure and start camera
    try 
    {
        RTT::log(RTT::Info) << "configure camera:" << RTT::endlog(); 
        if (!configureCamera())
            return false;

        //check if the camera is initialized
        if(cam_interface == NULL)
        {
            RTT::log(RTT::Error) << "Camera Driver Error: No camera interface was initialized" << RTT::endlog();
            report(NO_CAMERA_INTERFACE);
            return false;
        }

        RTT::log(RTT::Info) << cam_interface->doDiagnose() << RTT::endlog();

        if(!cam_interface->grab(_grab_mode,_frame_buffer_size))
	{
            RTT::log(RTT::Error) << "Camera Driver Error: Could not start grabbing" << RTT::endlog();	    
	    return false;
	}
    }
    catch(std::runtime_error e)
    {
        RTT::log(RTT::Error) << "failed to start camera: " << e.what() << RTT::endlog();
        report(CANNOT_START_GRABBING);
        return false;
    }
    
    // add file descriptor if task is fd driven
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        try 
        {
            RTT::log(RTT::Info) << "using FD activity !" << RTT::endlog();
            RTT::log(RTT::Info) << "  FD=" << cam_interface->getFileDescriptor() << RTT::endlog();
            fd_activity->watch(cam_interface->getFileDescriptor());
        }
        catch(std::runtime_error e)
        {
            RTT::log(RTT::Error) << "failed to get file descriptor: " << e.what() << RTT::endlog();
            report(CONFIGURE_ERROR);
            return false;
        }
    }
    return true;
}

void Task::processImage() {
    Frame *frame_ptr = output_frame.write_access();
    try 
    {
        frame_helper.convert(*camera_frame,*frame_ptr,_offset_x.value(),
                _offset_y.value(),_resize_algorithm.value(),_undistort.value());
    }
    catch(std::runtime_error e)
    {
        RTT::log(RTT::Error) << "processing error: " << e.what() << RTT::endlog();
        RTT::log(RTT::Error) << "Have you specified camera_format and output_format right?" << RTT::endlog();
        report(PROCESSING_ERROR);
    }
    output_frame.reset(frame_ptr);
}

bool Task::getFrame() {
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
        camera_frame.reset(frame_ptr);
    }

    if (frame_ptr->getStatus() == STATUS_VALID)
    {
        valid_frames_count++;
        //set extra attributes 
        setExtraAttributes(frame_ptr);
        camera_frame.reset(frame_ptr);
        //callback on frame retrieve
        onRetrieveNewFrame(*frame_ptr);
        return true;

    }
    else
    {
        camera_frame.reset(frame_ptr);
        invalid_frames_count++;
        return false;
    }

}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(cam_interface->isFrameAvailable())
        if (getFrame()) {
            //check if we have to process the frame before writing it to the port
            if(process_image) {
                processImage();
                _frame.write(output_frame);
            }
            else
                _frame.write(camera_frame);

            if(!_disable_frame_raw)
                _frame_raw.write(camera_frame);
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
    // remove file descriptor if task is fd driven
    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (fd_activity)
    {
        RTT::log(RTT::Info) << "clear FD watches" << RTT::endlog();
        fd_activity->clearAllWatches();
    }
    
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

bool Task::configureCamera()
{
    //sets binning to 1 otherwise high resolution can not be set
    if(cam_interface->isAttribAvail(int_attrib::BinningX))
    {
        cam_interface->setAttrib(int_attrib::BinningX,1);
        cam_interface->setAttrib(int_attrib::BinningY,1);
    }
    else
        RTT::log(RTT::Info) << "Binning is not supported by the camera" << RTT::endlog();

    //setting resolution and color mode
    try
    {
        cam_interface->setFrameSettings(*camera_frame);
    }
    catch(std::runtime_error e)
    {
        RTT::log(RTT::Error) << "failed to configure camera: " << e.what() << RTT::endlog();
        report(CONFIGURE_ERROR);
        return false;
    }

    //setting FrameRate
    if(_trigger_mode.value() == "fixed")
    {
        if (cam_interface->isAttribAvail(double_attrib::FrameRate))
            cam_interface->setAttrib(camera::double_attrib::FrameRate,_fps);
        else
            RTT::log(RTT::Info) << "FrameRate is not supported by the camera" << RTT::endlog();
    }

    //setting Region
    if(cam_interface->isAttribAvail(int_attrib::RegionX))
    {
        cam_interface->setAttrib(camera::int_attrib::RegionX,_region_x);
        cam_interface->setAttrib(camera::int_attrib::RegionY,_region_y);
    }
    else
        RTT::log(RTT::Info) << "Region is not supported by the camera" << RTT::endlog();

    //setting Binning
    if(cam_interface->isAttribAvail(int_attrib::BinningX))
    {
        cam_interface->setAttrib(camera::int_attrib::BinningX,_binning_x);
        cam_interface->setAttrib(camera::int_attrib::BinningY,_binning_y);
    }

    //setting ExposureValue
    if(cam_interface->isAttribAvail(int_attrib::ExposureValue))
        cam_interface->setAttrib(camera::int_attrib::ExposureValue,_exposure);
    else
        RTT::log(RTT::Info) << "ExposureValue is not supported by the camera" << RTT::endlog();

   
    //setting GainValue
    if(cam_interface->isAttribAvail(int_attrib::GainValue))
        cam_interface->setAttrib(camera::int_attrib::GainValue,_gain);
    else
        RTT::log(RTT::Info) << "GainValue is not supported by the camera" << RTT::endlog();

    //setting WhitebalValueBlue
    if(cam_interface->isAttribAvail(int_attrib::WhitebalValueBlue))
        cam_interface->setAttrib(camera::int_attrib::WhitebalValueBlue,_whitebalance_blue);
    else
        RTT::log(RTT::Info) << "WhitebalValueBlue is not supported by the camera" << RTT::endlog();

    //setting WhitebalValueRed
    if(cam_interface->isAttribAvail(int_attrib::WhitebalValueRed))
        cam_interface->setAttrib(camera::int_attrib::WhitebalValueRed,_whitebalance_red);
    else
        RTT::log(RTT::Info) << "WhitebalValueRed is not supported by the camera" << RTT::endlog();

    //setting WhitebalAutoRate
    if(cam_interface->isAttribAvail(int_attrib::WhitebalAutoRate))
        cam_interface->setAttrib(camera::int_attrib::WhitebalAutoRate,_whitebalance_auto_rate);
    else
        RTT::log(RTT::Info) << "WhitebalAutoRate is not supported by the camera" << RTT::endlog();

    //setting WhitebalAutoAdjustTol
    if(cam_interface->isAttribAvail(int_attrib::WhitebalAutoAdjustTol))
        cam_interface->setAttrib(camera::int_attrib::WhitebalAutoAdjustTol,_whitebalance_auto_threshold);
    else
        RTT::log(RTT::Info) << "WhitebalAutoAdjustTol is not supported by the camera" << RTT::endlog();
    
    //setting AcquisitionFrameCount
    if(cam_interface->isAttribAvail(int_attrib::AcquisitionFrameCount))
      cam_interface->setAttrib(int_attrib::AcquisitionFrameCount, _acquisition_frame_count);
    else
      RTT::log(RTT::Info) << "AcquisitionFrameCount is not supported by the camera" << RTT::endlog();

    //setting gamma mode
    if(_gamma.get())
    {
        if(cam_interface->isAttribAvail(enum_attrib::GammaToOn))
            cam_interface->setAttrib(enum_attrib::GammaToOn);
        else
            RTT::log(RTT::Info) << "GammaToOn is not supported by the camera" << RTT::endlog();
    }
    else
    {
        if(cam_interface->isAttribAvail(enum_attrib::GammaToOff))
            cam_interface->setAttrib(enum_attrib::GammaToOff);
        else
            RTT::log(RTT::Info) << "GammaToOff is not supported by the camera" << RTT::endlog();
      
    }
    
    //setting _whitebalance_mode
    if(_whitebalance_mode.value() == "manual")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::WhitebalModeToManual))
            cam_interface->setAttrib(camera::enum_attrib::WhitebalModeToManual);
        else
            RTT::log(RTT::Info) << "WhitebalModeToManual is not supported by the camera" << RTT::endlog();
    }
    else if (_whitebalance_mode.value() == "auto")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::WhitebalModeToAuto))
            cam_interface->setAttrib(camera::enum_attrib::WhitebalModeToAuto);
        else
            RTT::log(RTT::Info) << "WhitebalModeToAuto is not supported by the camera" << RTT::endlog();
    }
    else if (_whitebalance_mode.value() == "auto_once")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::WhitebalModeToAutoOnce))
            cam_interface->setAttrib(camera::enum_attrib::WhitebalModeToAutoOnce);
        else
            RTT::log(RTT::Info) << "WhitebalModeToAutoOnce is not supported by the camera" << RTT::endlog();
    }
    else if(_whitebalance_mode.value() == "none")
    {
        //do nothing
    }
    else
    {
        RTT::log(RTT::Error) << "Whitebalance mode "+ _whitebalance_mode.value() + " is not supported!" << RTT::endlog();
        report(UNKOWN_PARAMETER);
        return false;
    }

    //setting _exposure_mode
    if(_exposure_mode.value() == "auto")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::ExposureModeToAuto))
            cam_interface->setAttrib(camera::enum_attrib::ExposureModeToAuto);
        else
            RTT::log(RTT::Info) << "ExposureModeToAuto is not supported by the camera" << RTT::endlog();
    }
    else if(_exposure_mode.value() =="manual")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::ExposureModeToManual))
            cam_interface->setAttrib(camera::enum_attrib::ExposureModeToManual);
        else
            RTT::log(RTT::Info) << "ExposureModeToManual is not supported by the camera" << RTT::endlog();
    }
    else if (_exposure_mode.value() =="external")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::ExposureModeToExternal))
            cam_interface->setAttrib(camera::enum_attrib::ExposureModeToExternal);
        else
            RTT::log(RTT::Info) << "ExposureModeToExternal is not supported by the camera" << RTT::endlog();
    }
    else if(_exposure_mode.value() == "none")
    {
        //do nothing
    }
    else
    {
        RTT::log(RTT::Error) << "Exposure mode "+ _exposure_mode.value() + " is not supported!" << RTT::endlog();
        report(UNKOWN_PARAMETER);
        return false;
    }

    //setting _trigger_mode
    if(_trigger_mode.value() == "freerun")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerModeToFreerun))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerModeToFreerun);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerModeToFreerun is not supported by the camera" << RTT::endlog();
    }
    else if (_trigger_mode.value() == "fixed")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerModeToFixedRate))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerModeToFixedRate);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerModeToFixedRate is not supported by the camera" << RTT::endlog();
    }
    else if (_trigger_mode.value() == "sync_in1")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerModeToSyncIn1))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerModeToSyncIn1);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerModeToSyncIn1 is not supported by the camera" << RTT::endlog();
    }
    else if(_trigger_mode.value() == "none")
    {
        //do nothing
    }
    else
    {
        RTT::log(RTT::Error) << "Trigger mode "+ _trigger_mode.value() + " is not supported!" + " is not supported!" << RTT::endlog();
        report(UNKOWN_PARAMETER);
        return false;
    }

    //setting _frame_start_trigger_event
    if(_frame_start_trigger_event.value() == "EdgeRising")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerEventToEdgeRising))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerEventToEdgeRising);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerEventToEdgeRising is not supported by the camera" << RTT::endlog();
    }
    else if (_frame_start_trigger_event.value() == "EdgeFalling")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerEventToEdgeFalling))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerEventToEdgeFalling);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerEventToEdgeFalling is not supported by the camera" << RTT::endlog();
    }
    else if (_frame_start_trigger_event.value() == "EdgeAny")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerEventToEdgeAny))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerEventToEdgeAny);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerEventToEdgeAny is not supported by the camera" << RTT::endlog();
    }
    else if (_frame_start_trigger_event.value() == "LevelHigh")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerEventToLevelHigh))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerEventToLevelHigh);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerEventToLevelHigh is not supported by the camera" << RTT::endlog();
    }
    else if (_frame_start_trigger_event.value() == "LevelLow")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::FrameStartTriggerEventToLevelLow))
            cam_interface->setAttrib(camera::enum_attrib::FrameStartTriggerEventToLevelLow);
        else
            RTT::log(RTT::Info) << "FrameStartTriggerEventToLevelLow is not supported by the camera" << RTT::endlog();
    }
    else if(_frame_start_trigger_event.value() == "none")
    {
        //do nothing
    }
    else
    {
        RTT::log(RTT::Error) << "Frame start trigger event "+ _frame_start_trigger_event.value() + " is not supported!" << RTT::endlog();
        report(UNKOWN_PARAMETER);
        return false;
    }

    //setting _sync_out1_mode
    if(_sync_out1_mode.value() == "GPO")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToGPO))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToGPO);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToGPO is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "AcquisitionTriggerReady")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToAcquisitionTriggerReady))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToAcquisitionTriggerReady);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToAcquisitionTriggerReady is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "FrameTriggerReady")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToFrameTriggerReady))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToFrameTriggerReady);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToFrameTriggerReady is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "FrameTrigger")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToFrameTrigger))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToFrameTrigger);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToFrameTrigger is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "Exposing")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToExposing))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToExposing);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToExposing is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "FrameReadout")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToFrameReadout))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToFrameReadout);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToFrameReadout is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "Acquiring")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToAcquiring))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToAcquiring);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToAcquiring is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "SyncIn1")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToSyncIn1))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToSyncIn1);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToSyncIn1 is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "SyncIn2")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToSyncIn2))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToSyncIn2);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToSyncIn2 is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out1_mode.value() == "Strobe1")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut1ModeToStrobe1))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut1ModeToStrobe1);
        else
            RTT::log(RTT::Info) << "SyncOut1ModeToStrobe1 is not supported by the camera" << RTT::endlog();
    }
    else
    {
        RTT::log(RTT::Error) << "SyncOut1Mode "+ _frame_start_trigger_event.value() + " is not supported!" << RTT::endlog();
        report(UNKOWN_PARAMETER);
        return false;
    }

    //setting _sync_out2_mode
    if(_sync_out2_mode.value() == "GPO")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToGPO))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToGPO);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToGPO is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "AcquisitionTriggerReady")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToAcquisitionTriggerReady))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToAcquisitionTriggerReady);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToAcquisitionTriggerReady is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "FrameTriggerReady")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToFrameTriggerReady))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToFrameTriggerReady);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToFrameTriggerReady is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "FrameTrigger")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToFrameTrigger))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToFrameTrigger);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToFrameTrigger is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "Exposing")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToExposing))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToExposing);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToExposing is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "FrameReadout")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToFrameReadout))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToFrameReadout);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToFrameReadout is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "Acquiring")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToAcquiring))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToAcquiring);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToAcquiring is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "SyncIn1")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToSyncIn1))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToSyncIn1);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToSyncIn1 is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "SyncIn2")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToSyncIn2))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToSyncIn2);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToSyncIn2 is not supported by the camera" << RTT::endlog();
    }
    else if (_sync_out2_mode.value() == "Strobe1")
    {
        if(cam_interface->isAttribAvail(camera::enum_attrib::SyncOut2ModeToStrobe1))
            cam_interface->setAttrib(camera::enum_attrib::SyncOut2ModeToStrobe1);
        else
            RTT::log(RTT::Info) << "SyncOut2ModeToStrobe1 is not supported by the camera" << RTT::endlog();
    }
    else
    {
        RTT::log(RTT::Error) << "SyncOut2Mode "+ _frame_start_trigger_event.value() + " is not supported!" << RTT::endlog();
        report(UNKOWN_PARAMETER);
        return false;
    }


    RTT::log(RTT::Info) << "camera configuration: width="<<_width <<
        "; height=" << _height << 
        "; region_x=" << _region_x << 
        "; region_y=" << _region_y << 
        "; Trigger mode=" << _trigger_mode << 
        "; fps=" << _fps << 
        "; exposure=" << _exposure << 
        "; Whitebalance mode=" << _whitebalance_mode << 
        RTT::endlog();

    return true;
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

void Task::onRetrieveNewFrame(base::samples::frame::Frame& frame)
{

}
