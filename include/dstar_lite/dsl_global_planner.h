
#ifndef DSTAR_PLANNER_CPP
#define DSTAR_PLANNER_CPP


#include <cstdlib>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>

#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>

// #include <srl_dstar_lite/world_model.h>
#include <dstar_lite/world_model.h>
#include <dstar_lite/costmap_model.h>
// #include <data_structures.h>
#include <dstar_lite/DSLPlanner.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/base_global_planner.h>

#define COST_POSSIBLY_CIRCUMSCRIBED 128



namespace dsl_global_planner {


///<  @brief DStarLite class
class DStarLite : public nav_core::BaseGlobalPlanner {

private:
    ros::NodeHandle nh_;
    // Publishers
    ros::Publisher pub_path_;
    ros::Publisher pub_goal_;
    ros::Publisher pub_tree_;
    ros::Publisher pub_tree_dedicated_;
    ros::Publisher pub_path_dedicated_;
    ros::Publisher pub_samples_;
    ros::Publisher pub_graph_;
    ros::Publisher pub_no_plan_;
    ros::Publisher pub_obstacle_markers_;

    ros::Subscriber sub_obstacles_;
    ros::Subscriber sub_all_agents_;

public:


    /**
    * @brief Constructor
    * @param node, Ros NodeHandle
    * @param world_model, Cost Map model to load informaiton of the Global Cost map
    * @param footprint_spec, footprint of the robot
    * @param costmap_ros, cost_map ros wrapper
    */
    DStarLite() : costmap_ros_(NULL), initialized_(false),  world_model_(NULL) {

    }

    /**
    * @brief Constructor of the Srl_global_planner
    * @param name, name to associate to the node
    * @param costmap_ros, costmap_ros
    */
    DStarLite(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    /**
    * @brief Initialize the ros handle
    * @param name, Ros NodeHandle name
    * @param costmap_ros, cost_map ros wrapper
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
    * @brief makePlan, follows the virtual method of the base class
    * @param start, Start pose
    * @param goal, goal pose
    * @param plan, generated path
    * @return bool, true
    */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan );


    /**
    * @brief publishPath, Publish path
    * @return void
    */
    void publishPath(std::vector< geometry_msgs::PoseStamped > grid_plan);


    /**
    * @brief plan a path
    * @param, grid_plan Plan generated
    * @param, start pose
    * @return true, if the plan was found
    */
    int plan(std::vector< geometry_msgs::PoseStamped > &grid_plan, geometry_msgs::PoseStamped& start);

    /**
    * @brief set the Goal region
    * @param, x coordinate of the goal pose
    * @param, y coordinate of the goal pose
    * @param, theta yaw angle of the goal pose
    * @param, toll yaw angle of the goal pose
    * @param, goal_frame goal frame
    * @return void
    */
    void setGoal(double x, double y, double theta, double toll, std::string goal_frame);


    /**
    * @brief Transform pose in planner_frame
    * @param, init_pose initial pose to transform
    * @return Pose transformed
    */
    geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped init_pose);

    bool initialized_;
    int cnt_make_plan_;

    double cellwidth_;  ///<  @brief Cell width
    double cellheight_; ///<  @brief Cell height

    base_local_planner::CostmapModel* world_model_; ///<  @brief World Model associated to the costmap
    std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief FootPrint list of points of the robot
    costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
    costmap_2d::Costmap2D* costmap_; ///< @brief The ROS wrapper for the costmap the controller will use
    std::string node_name_;  ///<  @brief Node name

    double goal_x_; ///<  @brief x coordinate of the goal pose
    double goal_y_; ///<  @brief y coordinate of the goal pose
    double goal_theta_; ///<  @brief yaw angle of the goal pose
    double toll_goal_; ///<  @brief toll the goal region
    double rx,ry,rz; ///<  @brief robot coordinates

    double xscene_; ///<  @brief Width of the scene in meters
    double yscene_; ///<  @brief Height of the scene in meters

    // std::vector<Tobstacle> obstacle_positions; ///<  @brief Obstacles
    // std::vector<Thuman> agents_position; ///<  @brief agents

    bool goal_init_; ///<  @brief Flag that indicates if the goal is initialized

    geometry_msgs::Pose start_pose_; ///<  @brief Start pose
    geometry_msgs::Pose goal_pose_; ///<  @brief Goal pose

    ros::Time begin; ///<  @brief fMap loading only during the first Nsecs
    double initialization_time; ///<  @brief initialization time for map
    double map_loading_time; ///<  @brief map loading time
    double max_map_loading_time; ///<  @brief max map loading time

    int cnt_map_readings; ///<  @brief counter of map readings
    std::string costmap_frame_;  ///<  @brief costmap frame
    std::string planner_frame_; ///<  @brief planner frame

    double width_map_;  ///<  @brief Width of the 2D space where to sample
    double height_map_; ///<  @brief Height of the 2D space where to sample
    double center_map_x_; ///<  @brief x coordinate of the center of 2D space where to sample
    double center_map_y_; ///<  @brief y coordinate of the center of 2D space where to sample

    DSLPlanner *dsl_planner_; ///<  Dstar planner
};

}

#endif
