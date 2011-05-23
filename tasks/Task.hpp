/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */


#ifndef CAMERA_BASE_TASK_TASK_HPP
#define CAMERA_BASE_TASK_TASK_HPP

#include "camera_base/TaskBase.hpp"
#include "frame_helper/FrameHelper.h"
#include "camera_interface/CamInterface.h"

namespace camera_base {
    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:
        /* Handler for the getDoubleAttrib operation
         */
        virtual double getDoubleAttrib(::camera::double_attrib::CamAttrib const & type);
        /* Handler for the getDoubleRangeMax operation
         */
        virtual double getDoubleRangeMax(::camera::double_attrib::CamAttrib const & type);
        /* Handler for the getDoubleRangeMin operation
         */
        virtual double getDoubleRangeMin(::camera::double_attrib::CamAttrib const & type);
        /* Handler for the getIntAttrib operation
         */
        virtual boost::int32_t getIntAttrib(::camera::int_attrib::CamAttrib const & type);
        /* Handler for the getIntRangeMax operation
         */
        virtual boost::int32_t getIntRangeMax(::camera::int_attrib::CamAttrib const & type);
        /* Handler for the getIntRangeMin operation
         */
        virtual boost::int32_t getIntRangeMin(::camera::int_attrib::CamAttrib const & type);
        /* Handler for the getStringAttrib operation
         */
        virtual ::std::string getStringAttrib(::camera::str_attrib::CamAttrib const & type);
        /* Handler for the isEnumAttribSet operation
         */
        virtual bool isEnumAttribSet(::camera::enum_attrib::CamAttrib const & type);
        /* Handler for the setDoubleAttrib operation
         */
        virtual bool setDoubleAttrib(::camera::double_attrib::CamAttrib const & type, double value);
        /* Handler for the setEnumAttrib operation
         */
        virtual bool setEnumAttrib(::camera::enum_attrib::CamAttrib const & type);
        /* Handler for the setIntAttrib operation
         */
        virtual bool setIntAttrib(::camera::int_attrib::CamAttrib const & type, boost::int32_t value);
        /* Handler for the setStringAttrib operation
         */
        virtual bool setStringAttrib(::camera::str_attrib::CamAttrib const & type, ::std::string const & value);

    protected:

      camera::CamInterface* cam_interface;			                //handle to the camera
      RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> camera_frame;	
      RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> output_frame;	

      bool process_image;                       //indicates if the frames have to be processed 
                                                //before they are written to the output port
      frame_helper::FrameHelper frame_helper;   //helper for image processing

      //variables for a statistical analyse 
      unsigned int invalid_frames_count;		
      unsigned int valid_frames_count;
      
      float stat_frame_rate;
      float stat_invalid_frame_rate;
      float stat_valid_frame_rate;
      base::Time time_save;

    public:
        Task(std::string const& name = "camera::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

	~Task();

        //you have to implement this function for each camera
        //the function is called before the camera is started
        virtual void configureCamera();

        //if you wish to add extended attributes to the frame
        //implement this function in your derived class
        //this function is called after a frame was received from the camera
        virtual void setExtraAttributes(base::samples::frame::Frame *frame_ptr);

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         *
         *   task_context "TaskName" do
         *     needs_configuration
         *     ...
         *   end
         */
         bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
         bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
         void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recovered() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
         void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
         void cleanupHook();
    };
}

#endif

