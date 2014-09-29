/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace rosa_localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::depthTransformerCallback(const base::Time &ts, const ::odometry::DepthState &depth_sample)
{
    throw std::runtime_error("Transformer callback for depth not implemented");
}

void Task::rollTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &roll_sample)
{
    throw std::runtime_error("Transformer callback for roll not implemented");
}

void Task::sonarBeamsTransformerCallback(const base::Time &ts, const ::base::samples::SonarBeam &sonarBeams_sample)
{
  //Transformation between the sonar and the body, takes in account the effect of the PTU.
  Eigen::Affine3d tf;
  if (!_sonar2body.get(ts, tf, false))
    return;  
  
  throw std::runtime_error("Transformer callback for sonarBeams not implemented");
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if( !_octomap_path.value().empty() )
    {
	LOG_INFO_S << "loading map: " << _octomap_path.value() << std::endl;
	
	refMap = new octomap::SonarOcTree(_octomap_path.value());
	
	odometry = new odometry::RosaOdometry(_odometry_config.get());
	
	if(!refMap){
	  
	   filter = new RosaPoseEstimator( *odometry,
				      *refMap, _rosa_localization_config.get() );
	   
	}
	else this->error();
	

    }
    else this->error();

    return TaskBase::configureHook();
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
