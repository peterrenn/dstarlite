#include <dsl_global_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dsl_global_planner::DStarLite, nav_core::BaseGlobalPlanner);

using namespace std;
using namespace base_local_planner;

//extern Config config;
tf::TransformListener* listener;

namespace dsl_global_planner {

/// ==================================================================================
/// transformPose(geometry_msgs::PoseStamped init_pose)
/// Transforms the init_pose in the costmap_frame_
/// ==================================================================================
geometry_msgs::PoseStamped DStarLite::transformPose(geometry_msgs::PoseStamped init_pose){

    geometry_msgs::PoseStamped res;
    tf::StampedTransform transform;

    try{
        // will transform data in the goal_frame into the costmap_frame_
        listener->waitForTransform( costmap_frame_, init_pose.header.frame_id, ros::Time::now(), ros::Duration(0.20));
        listener->lookupTransform( costmap_frame_,  init_pose.header.frame_id, ros::Time::now(), transform);
    }
    catch(tf::TransformException){
        ROS_ERROR("Failed to transform the given pose in the D*Lite Planner frame_id");
        return init_pose;
    }

    tf::Pose source;
    tf::Quaternion q= tf::createQuaternionFromRPY(0,0,tf::getYaw(init_pose.pose.orientation));
    tf::Matrix3x3 base(q);
    source.setOrigin(tf::Vector3(init_pose.pose.position.x, init_pose.pose.position.y, 0));
    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result = transform*source;

    res.pose.position.x = result.getOrigin().x() ;
    res.pose.position.y = result.getOrigin().y() ;
    res.pose.position.z = result.getOrigin().z() ;

    tf::quaternionTFToMsg(result.getRotation(), res.pose.orientation);
    res.header = init_pose.header;
    res.header.frame_id = costmap_frame_;
    return res;

}


/// ==================================================================================
/// setGoal(double x, double y, double theta,double toll)
/// Method to store the Goal region description into the instance of the class
/// ==================================================================================
void DStarLite::setGoal(double x, double y, double theta, double toll, std::string goal_frame){

  ROS_DEBUG("Setting Goal, Goal Frame %s", goal_frame.c_str());

    tf::StampedTransform transform;

    try{
        // will transform data in the goal_frame into the costmap_frame_
        listener->waitForTransform( costmap_frame_, goal_frame, ros::Time::now(), ros::Duration(0.20));
        listener->lookupTransform( costmap_frame_, goal_frame, ros::Time::now(), transform);
    }
    catch(tf::TransformException){
        ROS_ERROR("Failed to transform Goal Transform in D*Lite Planner");
        return;
    }

    tf::Pose source;
    tf::Quaternion q = tf::createQuaternionFromRPY(0,0, theta);
    tf::Matrix3x3 base(q);
    source.setOrigin(tf::Vector3(x, y, 0));
    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result = transform*source;

    this->goal_theta_= tf::getYaw( result.getRotation());
    this->goal_x_ = result.getOrigin().x();
    this->goal_y_ = result.getOrigin().y();
    this->toll_goal_ = toll;
    this->goal_init_ = true;

    ROS_DEBUG("GOAL SET!!");

    goal_pose_.position.x = result.getOrigin().x();
    goal_pose_.position.y = result.getOrigin().y();
    goal_pose_.position.z = result.getOrigin().z();

    tf::quaternionTFToMsg(result.getRotation(), goal_pose_.orientation);

    this->goal_init_=true;
}


/// ==================================================================================
/// publishPath(std::vector< geometry_msgs::PoseStamped > grid_plan)
/// method to publish the path
/// ==================================================================================
void DStarLite::publishPath(std::vector< geometry_msgs::PoseStamped > grid_plan){

    nav_msgs::Path path_;
    ROS_DEBUG("Publishing a path");

    int path_size = (int)grid_plan.size();

    path_.header.frame_id = costmap_frame_;
    path_.header.stamp = ros::Time();
    path_.poses.resize(path_size);

    visualization_msgs::Marker path_marker_;
    path_marker_.header.frame_id = costmap_frame_;
    path_marker_.header.stamp = ros::Time();
    path_marker_.ns = "DStarLite";
    path_marker_.id = 1;

    path_marker_.type = visualization_msgs::Marker::POINTS;
    path_marker_.color.a = 1;
    path_marker_.color.r = 0.0;
    path_marker_.color.g = 1.0;
    path_marker_.color.b = 0.0;

    path_marker_.scale.x = 0.5;
    path_marker_.scale.y = 0.5;
    path_marker_.scale.z = 0.5;


    path_marker_.action = 0;  // add or modify


    for (int i = 0; i < path_size; i++) {

      geometry_msgs::PoseStamped posei = grid_plan[i];
      path_.poses[i] = posei;
      geometry_msgs::Point p;
      p.x = posei.pose.position.x;
      p.y = posei.pose.position.y;
      p.z = 0.05;
      path_marker_.points.push_back(p);
    }

    if(grid_plan.size()>0){

        pub_path_.publish(path_);
        pub_path_dedicated_.publish(path_marker_);

        ROS_DEBUG("Path Published");
    }
}


/// ==================================================================================
/// plan(std::vector< geometry_msgs::PoseStamped > &grid_plan, geometry_msgs::PoseStamped& start)
/// method to solve a planning problem.
/// ==================================================================================

int DStarLite::plan(std::vector< geometry_msgs::PoseStamped > &grid_plan, geometry_msgs::PoseStamped& start){
  // start and goal positions
  unsigned int start_mx;
  unsigned int start_my;
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  costmap_->worldToMap ( start_x, start_y, start_mx, start_my);
  ROS_DEBUG("Update Start Point %f %f to %d %d", start_x, start_y, start_mx, start_my);
  dsl_planner_->updateStart(start_mx, start_my);
  unsigned int goal_mx;
  unsigned int goal_my;
  costmap_->worldToMap (goal_x_, goal_y_, goal_mx, goal_my);
  ROS_DEBUG("Update Goal Point %f %f to %d %d", goal_x_, goal_y_, goal_mx, goal_my);
  dsl_planner_->updateGoal(goal_mx, goal_my);

  /// 1. Grid size
  int nx_cells, ny_cells;
  nx_cells = costmap_->getSizeInCellsX();
  ny_cells = costmap_->getSizeInCellsY();


  unsigned char* grid = costmap_->getCharMap();
  for(int x=0; x<nx_cells; x++){
      for(int y=0; y<ny_cells; y++){
          int index = costmap_->getIndex(x,y);

          double c = (double)grid[index];

          if( c >= COST_POSSIBLY_CIRCUMSCRIBED)
              dsl_planner_->updateCell(x, y, -1);
          else if (c == costmap_2d::FREE_SPACE){
              dsl_planner_->updateCell(x, y, 1);
          }else
          {
              dsl_planner_->updateCell(x, y, c);
          }
        }
      }

  ROS_DEBUG("Replan");
  dsl_planner_->replan();

  ROS_DEBUG("Get Path");
  //list<state> path = dsl_planner_->getPath();
  list<node*> path = dsl_planner_->getPath();

  /// 4. Returning the path generated
  grid_plan.clear();
  grid_plan.push_back(start);

  double costmap_resolution = costmap_->getResolution();
  double origin_costmap_x = costmap_->getOriginX();
  double origin_costmap_y = costmap_->getOriginY();

  //std::list<state>::const_iterator iterator;
  std::list<node*>::const_iterator iterator;

  for (iterator = path.begin(); iterator != path.end(); ++iterator) {
      //state node = *iterator;
      node* cur = *iterator;

      geometry_msgs::PoseStamped next_node;
      next_node.header.seq = cnt_make_plan_;
      next_node.header.stamp = ros::Time::now();
      next_node.header.frame_id = costmap_frame_;
      //next_node.pose.position.x = (node.x+0.5)*costmap_resolution + origin_costmap_x;
      //next_node.pose.position.y = (node.y+0.5)*costmap_resolution + origin_costmap_y;
      // what's with the 0.5 shift?
      next_node.pose.position.x = (cur->x+0.5)*costmap_resolution + origin_costmap_x;
      next_node.pose.position.y = (cur->y+0.5)*costmap_resolution + origin_costmap_y;
      grid_plan.push_back(next_node);
    }


  if(path.size()>0){
      publishPath(grid_plan);
      return true;
  }
  else
      return false;
}



/// ==================================================================================
/// makePlan()
/// ==================================================================================
bool DStarLite::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){


      if(this->initialized_){
        this->setGoal((double)goal.pose.position.x, (double)goal.pose.position.y, (double)tf::getYaw(goal.pose.orientation), toll_goal_ , goal.header.frame_id );
        /// Reading in Odom Frame
        std::vector< geometry_msgs::PoseStamped > grid_plan;
        geometry_msgs::PoseStamped s;
        s = transformPose(start);

        if(this->plan(grid_plan, s)){
          plan.clear();
          cnt_make_plan_++ ;

          for (size_t i = 0; i < grid_plan.size(); i++) {
            geometry_msgs::PoseStamped posei;
            posei.header.seq = cnt_make_plan_;
            posei.header.stamp = ros::Time::now();
            posei.header.frame_id = costmap_frame_;
            posei.pose.position.x = grid_plan[i].pose.position.x;
            posei.pose.position.y = grid_plan[i].pose.position.y;
            posei.pose.position.z = 0;
            posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,grid_plan[i].pose.position.z);
            plan.push_back(posei);
          }

          ROS_DEBUG("Path found");
          return true;
      }
      else{
        ROS_WARN("No path found");
        return false;
      }

}



/// ==================================================================================
/// initialize()
/// Method to initialize all the publishers and subscribers
/// ==================================================================================
void  DStarLite::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
        this->node_name_ = name;
        this->initialized_ = false;
        this->goal_theta_=0;
        this->goal_x_=0;
        this->goal_y_=0;
        this->cellheight_=1.5;
        this->cellwidth_=1.5;
        this->goal_init_=false;
        this->cnt_make_plan_ = 0;
        ros::NodeHandle node("~/DStarLite");
        nh_ =  node;

        ROS_INFO("DStarLite start initializing");

        listener = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        costmap_frame_ = "map";


        try{
              footprint_spec_ = costmap_ros_->getRobotFootprint();
              if( (int)(footprint_spec_.size()) > 0)
                  ROS_INFO("footprint_spec_ loaded with %d elements", (int)footprint_spec_.size());
              world_model_ = new CostmapModel(*costmap_);
              dsl_planner_ = new DSLPlanner();
              dsl_planner_->init(0, 0, 10, 10, costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX()); // First initialization
        }

        catch (exception& e)
        {
            ROS_ERROR("An exception occurred. Exception Nr. %s", e.what() );
        }

        /// all the ROS DATA
        // setup publishers
        pub_path_ = nh_.advertise<nav_msgs::Path>("dstar_planner_path", 1);
        pub_path_dedicated_=nh_.advertise<visualization_msgs::Marker>("dstar_path_dedicated",1000);

        ROS_INFO("ROS publishers and subscribers initialized");
        /// Initializing number of read maps
        cnt_map_readings=0;

        /// define dim of scene
        double x1,x2, y1,y2, csx,csy;

        nh_.getParam("cell_size_x", csx);
        nh_.getParam("cell_size_y", csy);
        nh_.getParam("GOAL_TOLL", this->toll_goal_);
        int tcount,firstsol,deburrt;

        nh_.getParam("max_map_loading_time",this->max_map_loading_time);
        nh_.getParam("planner_frame",this->planner_frame_);

        /// store dim of scene
        this->xscene_ = x2-x1;
        this->yscene_ = y2-y1;
        /// store sizes
        this->cellwidth_ = csx;
        this->cellheight_ = csy;

        ROS_INFO("Planner initialized");

        int size_footprint = (int)footprint_spec_.size();
        ROS_INFO("Current size of the footprint %d", size_footprint);

        initialized_ = true;
    }
    else{
        ROS_WARN("D* Lite planner not initialized");
    }
}
}
