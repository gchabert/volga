/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Omid Heidari
   Desc:   Volga planning plugin
 */

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <class_loader/class_loader.hpp>
#include <volga_moveit/motion_validator.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/profiler/profiler.h>
#include <class_loader/class_loader.hpp>

#include <dynamic_reconfigure/server.h>
#include <moveit_planners_ompl/OMPLDynamicReconfigureConfig.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/Console.h>

#include <tf_conversions/tf_eigen.h>
#include <thread>
#include <memory>
#include <volga_core/SerialRobot.h>
#include <volga_core/tf2ibex.h>

using namespace moveit_planners_ompl;
using namespace volga_core;

namespace volga_moveit
{
#define OMPL_ROS_LOG(ros_log_level)                                                                                    \
		{                                                                                                                    \
	ROSCONSOLE_DEFINE_LOCATION(true, ros_log_level, ROSCONSOLE_NAME_PREFIX ".ompl");                                   \
	if (ROS_UNLIKELY(__rosconsole_define_location__enabled))                                                           \
	::ros::console::print(0, __rosconsole_define_location__loc.logger_, __rosconsole_define_location__loc.level_,    \
			filename, line, __ROSCONSOLE_FUNCTION__, "%s", text.c_str());                              \
		}

class VolgaPlannerManager : public planning_interface::PlannerManager
{
public:
	VolgaPlannerManager() : planning_interface::PlannerManager(), nh_("~"), volga_epsilon(0.1 /* TODO: to be set by the user */), volga_model(NULL), motion_validator(NULL)
	{
		class OutputHandler : public ompl::msg::OutputHandler
		{
		public:
			void log(const std::string& text, ompl::msg::LogLevel level, const char* filename, int line) override
			{
				switch (level)
				{
				case ompl::msg::LOG_DEV2:
				case ompl::msg::LOG_DEV1:
				case ompl::msg::LOG_DEBUG:
					OMPL_ROS_LOG(::ros::console::levels::Debug);
					break;
				case ompl::msg::LOG_INFO:
					OMPL_ROS_LOG(::ros::console::levels::Info);
					break;
				case ompl::msg::LOG_WARN:
					OMPL_ROS_LOG(::ros::console::levels::Warn);
					break;
				case ompl::msg::LOG_ERROR:
					OMPL_ROS_LOG(::ros::console::levels::Error);
					break;
				case ompl::msg::LOG_NONE:
				default:
					/* ignore */
					break;
				}
			}
		};

		log = 2;
		output_handler_.reset(new OutputHandler());
		ompl::msg::useOutputHandler(output_handler_.get());
	}

	bool initialize(const moveit::core::RobotModelConstPtr& model, const std::string& ns) override
	{
		volga_model = new volga_core::SerialRobot(*model->getURDF(), volga_epsilon, false, false);
		if (!ns.empty())
			nh_ = ros::NodeHandle(ns);
		ompl_interface_.reset(new ompl_interface::OMPLInterface(model, nh_));
		std::string ompl_ns = ns.empty() ? "ompl" : ns + "/ompl";
		dynamic_reconfigure_server_.reset(
				new dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig>(ros::NodeHandle(nh_, ompl_ns)));
		dynamic_reconfigure_server_->setCallback(
				std::bind(&VolgaPlannerManager::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2));
		config_settings_ = ompl_interface_->getPlannerConfigurations();
		return true;
	}

	~VolgaPlannerManager() {
		for (std::vector<volga_core::Mesh*>:: iterator it=obstacles.begin(); it!=obstacles.end(); ++it) {
			delete *it;
		}
		obstacles.clear();
	}

	bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const override
	{
		return req.trajectory_constraints.constraints.empty();
	}

	std::string getDescription() const override
	{
		return "OMPL/Volga";
	}

	void getPlanningAlgorithms(std::vector<std::string>& algs) const override
	{
		const planning_interface::PlannerConfigurationMap& pconfig = ompl_interface_->getPlannerConfigurations();
		algs.clear();
		algs.reserve(pconfig.size());
		for (const std::pair<const std::string, planning_interface::PlannerConfigurationSettings>& config : pconfig)
			algs.push_back(config.first);
	}

	void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig) override
	{
		// this call can add a few more configs than we pass in (adds defaults)
		ompl_interface_->setPlannerConfigurations(pconfig);
		// so we read the configs instead of just setting pconfig
		PlannerManager::setPlannerConfigurations(ompl_interface_->getPlannerConfigurations());
	}

	planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
			const planning_interface::MotionPlanRequest& req,
			moveit_msgs::MoveItErrorCodes& error_code) const override
	{

		// Modification by Gilles Chabert:
		// ===============================================================================
		const collision_detection::WorldConstPtr& world = planning_scene->getWorld();
		collision_detection::World::const_iterator it;

		/*
		 *  TODO : handle scene "diffs"
		 *
		 *  so far, we cleanup everything...
		 */
		if (log>=2) std::cout << " [volga-moveit] cleaning obstacles\n";
		for (std::vector<volga_core::Mesh*>:: iterator it=obstacles.begin(); it!=obstacles.end(); ++it) {
			delete *it;
		}
		obstacles.clear();

		if (log>=2) std::cout << " [volga-moveit] loading obstacles\n";
		for (it = world->begin(); it!=world->end(); ++it) {

			if (log>=2) std::cout << " [volga-moveit] loading object " << it->second->id_ << "\n";

			for (std::vector<shapes::ShapeConstPtr>::const_iterator it2 = (it->second)->shapes_.begin(); it2!=(it->second)->shapes_.end(); ++it2) {

				if ((*it2)->type!=shapes::MESH && (*it2)->type!=shapes::SPHERE) {
					ROS_WARN("   Only spheres or objects with meshs are currently handled by Volga! --> object ignored");
					continue;
				}
				volga_core::Mesh* volga_mesh = new volga_core::Mesh();

				if ((*it2)->type==shapes::MESH) {
					bool meters = true;
					const shapes::Mesh& mesh =  (const shapes::Mesh&) (**it2);
					volga_mesh->init(mesh.vertex_count, mesh.vertices, mesh.triangle_count, mesh.triangles, meters, volga_model->epsilon, log>=1);
				} else {
					const shapes::Sphere& sphere = (const shapes::Sphere&) (**it2);
					volga_mesh->init(volga_core::Ball(Point3D(0,0,0),sphere.radius));
				}

				// note: one transform per object (not per shape)
				tf::Transform robot_frame;
				tf::Transform object_frame;

				//std::cout << "   robot model frame=" << planning_scene->getRobotModel()->getModelFrame() << std::endl;
				//std::cout << "   planning frame=" << planning_scene->getPlanningFrame() << std::endl;
				//std::cout << "   object frame=" << it->second->id_ << std::endl;

				//transformEigenToTF(planning_scene->getFrameTransform(planning_scene->getRobotModel()->getModelFrame()), robot_frame);
				transformEigenToTF(planning_scene->getFrameTransform(planning_scene->getPlanningFrame()), robot_frame);
				transformEigenToTF(planning_scene->getFrameTransform(it->second->id_), object_frame);

				tf::Transform t=robot_frame.inverse()*object_frame;
				if (log>=2)
					std::cout << "   [volga-moveit] object-robot frame:" << t << std::endl;
				volga_mesh->move(t);

				obstacles.push_back(volga_mesh);
			}
		}

		ompl_interface::ModelBasedPlanningContextPtr context = ompl_interface_->getPlanningContext(planning_scene, req, error_code);
		ompl::geometric::SimpleSetupPtr& ompl_setup = context->getOMPLSimpleSetup();
		ompl::base::SpaceInformationPtr& ompl_space_info = (ompl::base::SpaceInformationPtr&) ompl_setup->getSpaceInformation();
		
		volga_moveit::MotionValidatorPtr motion_validator(new volga_moveit::MotionValidator(context, *volga_model, obstacles));

		ompl_space_info->setMotionValidator(motion_validator);

		return context;
	}

private:
	/*
  bool r = ompl_interface_->solve(planning_scene, req, res);
  if (!planner_data_link_name_.empty())
    displayPlannerData(planning_scene, planner_data_link_name_);
  return r;
	 */

	/*
  bool r = ompl_interface_->solve(planning_scene, req, res);
  if (!planner_data_link_name_.empty())
    displayPlannerData(planning_scene, planner_data_link_name_);
  return r;
	 */

	/*
  void displayRandomValidStates()
  {
    ompl_interface::ModelBasedPlanningContextPtr pc = ompl_interface_->getLastPlanningContext();
    if (!pc || !pc->getPlanningScene())
    {
      ROS_ERROR("No planning context to sample states for");
      return;
    }
    ROS_INFO_STREAM("Displaying states for context " << pc->getName());
    const og::SimpleSetup &ss = pc->getOMPLSimpleSetup();
    ob::ValidStateSamplerPtr vss = ss.getSpaceInformation()->allocValidStateSampler();
    moveit::core::RobotState robot_state = pc->getPlanningScene()->getCurrentState();
    ob::ScopedState<> rstate1(ss.getStateSpace());
    ob::ScopedState<> rstate2(ss.getStateSpace());
    ros::WallDuration wait(2);
    unsigned int n = 0;
    std::vector<ob::State*> sts;
    if (vss->sample(rstate2.get()))
      while (display_random_valid_states_)
      {
        if (!vss->sampleNear(rstate1.get(), rstate2.get(), 10000000))
          continue;
        pc->getOMPLStateSpace()->copyToRobotState(robot_state, rstate1.get());
        robot_state.getJointStateGroup(pc->getJointModelGroupName())->updateLinkTransforms();
        moveit_msgs::DisplayRobotState state_msg;
        moveit::core::robotStateToRobotStateMsg(robot_state, state_msg.state);
        pub_valid_states_.publish(state_msg);
        n = (n + 1) % 2;
        if (n == 0)
        {
          robot_trajectory::RobotTrajectory traj(pc->getRobotModel(), pc->getJointModelGroupName());
          unsigned int g = ss.getSpaceInformation()->getMotionStates(rstate1.get(), rstate2.get(), sts, 10, true, true);
          ROS_INFO("Generated a motion with %u states", g);
          for (std::size_t i = 0 ; i < g ; ++i)
          {
            pc->getOMPLStateSpace()->copyToRobotState(robot_state, sts[i]);
            traj.addSuffixWayPoint(robot_state, 0.0);
          }
          moveit_msgs::DisplayTrajectory msg;
          msg.model_id = pc->getRobotModel()->getName();
          msg.trajectory.resize(1);
          traj.getRobotTrajectoryMsg(msg.trajectory[0]);
          moveit::core::robotStateToRobotStateMsg(traj.getFirstWayPoint(), msg.trajectory_start);
          pub_valid_traj_.publish(msg);
        }
        rstate2 = rstate1;
        wait.sleep();
      }
  }

  void displayPlannerData(const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const std::string &link_name) const
  {
    ompl_interface::ModelBasedPlanningContextPtr pc = ompl_interface_->getLastPlanningContext();
    if (pc)
    {
      ompl::base::PlannerData pd(pc->getOMPLSimpleSetup()->getSpaceInformation());
      pc->getOMPLSimpleSetup()->getPlannerData(pd);
      moveit::core::RobotState robot_state = planning_scene->getCurrentState();
      visualization_msgs::MarkerArray arr;
      std_msgs::ColorRGBA color;
      color.r = 1.0f;
      color.g = 0.25f;
      color.b = 1.0f;
      color.a = 1.0f;
      unsigned int nv = pd.numVertices();
      for (unsigned int i = 0 ; i < nv ; ++i)
      {
        pc->getOMPLStateSpace()->copyToRobotState(robot_state, pd.getVertex(i).getState());
        robot_state.getJointStateGroup(pc->getJointModelGroupName())->updateLinkTransforms();
        const Eigen::Vector3d &pos = robot_state.getLinkState(link_name)->getGlobalLinkTransform().translation();

        visualization_msgs::Marker mk;
        mk.header.stamp = ros::Time::now();
        mk.header.frame_id = planning_scene->getPlanningFrame();
        mk.ns = "planner_data";
        mk.id = i;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z();
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = mk.scale.z = 0.025;
        mk.color = color;
        mk.lifetime = ros::Duration(30.0);
        arr.markers.push_back(mk);
      }
      pub_markers_.publish(arr);
    }
  }
	 */

	void dynamicReconfigureCallback(OMPLDynamicReconfigureConfig& config, uint32_t /*level*/)
	{
		if (config.link_for_exploration_tree.empty() && !planner_data_link_name_.empty())
		{
			pub_markers_.shutdown();
			planner_data_link_name_.clear();
			ROS_INFO("Not displaying OMPL exploration data structures.");
		}
		else if (!config.link_for_exploration_tree.empty() && planner_data_link_name_.empty())
		{
			pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("ompl_planner_data_marker_array", 5);
			planner_data_link_name_ = config.link_for_exploration_tree;
			ROS_INFO("Displaying OMPL exploration data structures for %s", planner_data_link_name_.c_str());
		}

		ompl_interface_->simplifySolutions(config.simplify_solutions);
		ompl_interface_->getPlanningContextManager().setMaximumSolutionSegmentLength(config.maximum_waypoint_distance);
		ompl_interface_->getPlanningContextManager().setMinimumWaypointCount(config.minimum_waypoint_count);
		if (display_random_valid_states_ && !config.display_random_valid_states)
		{
			display_random_valid_states_ = false;
			if (pub_valid_states_thread_)
			{
				pub_valid_states_thread_->join();
				pub_valid_states_thread_.reset();
			}
			pub_valid_states_.shutdown();
			pub_valid_traj_.shutdown();
		}
		else if (!display_random_valid_states_ && config.display_random_valid_states)
		{
			pub_valid_states_ = nh_.advertise<moveit_msgs::DisplayRobotState>("ompl_planner_valid_states", 5);
			pub_valid_traj_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("ompl_planner_valid_trajectories", 5);
			display_random_valid_states_ = true;
			//    pub_valid_states_thread_.reset(new boost::thread(boost::bind(&OMPLPlannerManager::displayRandomValidStates,
			//    this)));
		}
	}

	ros::NodeHandle nh_;
	std::unique_ptr<dynamic_reconfigure::Server<OMPLDynamicReconfigureConfig>> dynamic_reconfigure_server_;
	std::unique_ptr<ompl_interface::OMPLInterface> ompl_interface_;
	std::unique_ptr<std::thread> pub_valid_states_thread_;
	bool display_random_valid_states_{ false };
	ros::Publisher pub_markers_;
	ros::Publisher pub_valid_states_;
	ros::Publisher pub_valid_traj_;
	std::string planner_data_link_name_;
	std::shared_ptr<ompl::msg::OutputHandler> output_handler_;

	/** ================ Volga Data ================== */
	int log; // 0=none, 1=main messages only, 2=verbose
	double volga_epsilon;
	volga_core::SerialRobot *volga_model;
	volga_moveit::MotionValidator *motion_validator;

	// todo: try to make it non-mutable
	mutable std::vector<volga_core::Mesh*> obstacles;
};

}  // namespace volga_moveit

// register the VolgaPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(volga_moveit::VolgaPlannerManager, planning_interface::PlannerManager);
