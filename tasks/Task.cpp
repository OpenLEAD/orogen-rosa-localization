/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <octomap_wrapper/Conversion.hpp>

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

void Task::depthTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_sample)
{
    Eigen::Affine3d depth2body_tf;
    if (!_depth2body.get(ts, depth2body_tf, false))
        return;
    
    Eigen::Affine3d body_tf = depth2body_tf*depth_sample.getTransform();
    
    //copy to preserve the covs
    base::samples::RigidBodyState body_depth = depth_sample;
    body_depth.setTransform(body_tf);
    odometry->update(body_depth,base::Orientation::Identity());

}

void Task::rollTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &roll_sample)
{
    filter->project(roll_sample.orientation);
}

void Task::sonarBeamsTransformerCallback(const base::Time &ts, const ::base::samples::SonarBeam &sonarBeams_sample)
{
    //Transformation between the sonar and the body, takes in account the effect of the PTU.
    Eigen::Affine3d tf;
    if (!_sonar2body.get(ts, tf, false))
        return;  

    filter->update(sonarBeams_sample, tf);
  
    // write the centroid to the output port as the current best guess
    base::Affine3d centroid = filter->getCentroid().toTransform();;
    base::samples::RigidBodyState position;
    position.time = ts;
    position.setTransform( centroid );
            
    // write result to output port
    _pose_samples.write( position );
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
   // init the filter
    base::Pose pose( _start_pose.value().position, _start_pose.value().orientation );
    LOG_INFO_S << "starting at position " << pose.position.transpose() << std::endl;  
    
    const double angle = pose.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];
    
    
    
    Configuration const& conf = _rosa_localization_config.get();
    filter->init(
                conf.particleCount, 
		Eigen::Vector3d(pose.position.x(),pose.position.y(),pose.position.z()), 
		Eigen::Vector3d(conf.initialTranslationError.x(),
                                conf.initialTranslationError.y(),
			        conf.initialTranslationError.z() ),
		_start_roll,
		conf.initialRotationError.x()
		);
    LOG_INFO_S << "initialized" << std::endl;
    
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
    delete refMap;
    delete odometry;
    delete filter;
    TaskBase::cleanupHook();
}
