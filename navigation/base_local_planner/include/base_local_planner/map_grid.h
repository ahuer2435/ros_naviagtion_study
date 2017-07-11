/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
#ifndef TRAJECTORY_ROLLOUT_MAP_GRID_H_
#define TRAJECTORY_ROLLOUT_MAP_GRID_H_

#include <vector>
#include <iostream>
#include <base_local_planner/trajectory_inc.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <base_local_planner/map_cell.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace base_local_planner{
  /**
   * @class MapGrid
   * @brief A grid of MapCell cells that is used to propagate path and goal distances for the trajectory controller.
   * grid代表局部规划路径与全局路径的距离和目标点的距离。
   */
  class MapGrid{
    public:
      /**
       * @brief  Creates a 0x0 map by default
       * 创建0x0的山歌地图。
       */
      MapGrid();

      /**
       * @brief  Creates a map of size_x by size_y
       * @param size_x The width of the map 
       * @param size_y The height of the map 
       * 创建size_x X size_y_ 大小的地图。
       */
      MapGrid(unsigned int size_x, unsigned int size_y);


      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A reference to the desired cell
       * 获取一个MapCell元素的引用
       */
      inline MapCell& operator() (unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Returns a map cell accessed by (col, row)
       * @param x The x coordinate of the cell 
       * @param y The y coordinate of the cell 
       * @return A copy of the desired cell
       * 获取一个MapCell元素的拷贝
       */
      inline MapCell operator() (unsigned int x, unsigned int y) const {
        return map_[size_x_ * y + x];
      }

      //获取一个MapCell元素的引用
      inline MapCell& getCell(unsigned int x, unsigned int y){
        return map_[size_x_ * y + x];
      }

      /**
       * @brief  Destructor for a MapGrid
       */
      ~MapGrid(){}

      /**
       * @brief  Copy constructor for a MapGrid
       * @param mg The MapGrid to copy 
       * 拷贝构造函数
       */
      MapGrid(const MapGrid& mg);

      /**
       * @brief  Assignment operator for a MapGrid
       * @param mg The MapGrid to assign from 
       */
      MapGrid& operator= (const MapGrid& mg);

      /**
       * @brief reset path distance fields for all cells
       */
      void resetPathDist();

      /**
       * @brief  check if we need to resize
       * @param size_x The desired width
       * @param size_y The desired height
       */
      void sizeCheck(unsigned int size_x, unsigned int size_y);

      /**
       * @brief Utility to share initialization code across constructors
       */
      void commonInit();

      /**
       * @brief  Returns a 1D index into the MapCell array for a 2D index
       * @param x The desired x coordinate
       * @param y The desired y coordinate
       * @return The associated 1D index 
       */
      size_t getIndex(int x, int y);

      /**
       * return a value that indicates cell is in obstacle
       * 不明白
       */
      inline double obstacleCosts() {
        return map_.size();
      }

      /**
       * returns a value indicating cell was not reached by wavefront
       * propagation of set cells. (is behind walls, regarding the region covered by grid)
       * 不明白
       */
      inline double unreachableCellCosts() {
        return map_.size() + 1;
      }

      /**
       * @brief  Used to update the distance of a cell in path distance computation
       * @param  current_cell The cell we're currently in 
       * @param  check_cell The cell to be updated
       * 更新一个cell的路径距离。
       */
      inline bool updatePathCell(MapCell* current_cell, MapCell* check_cell,
          const costmap_2d::Costmap2D& costmap);

      /**
       * increase global plan resolution to match that of the costmap by adding points linearly between global plan points
       * This is necessary where global planners produce plans with few points.
       * @param global_plan_in input
       * @param global_plan_output output
       * @param resolution desired distance between waypoints
       * 更改全局路径的resolution
       */
      static void adjustPlanResolution(const std::vector<geometry_msgs::PoseStamped>& global_plan_in,
            std::vector<geometry_msgs::PoseStamped>& global_plan_out, double resolution);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the planned path
       * @param dist_queue A queue of the initial cells on the path 
       * 计算局部路径上每个点到全局路径的距离
       */
      void computeTargetDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      /**
       * @brief  Compute the distance from each cell in the local map grid to the local goal point
       * @param goal_queue A queue containing the local goal cell 
       * 计算局部地图上每一个cell到局部地图上的目标点的距离
       */
      void computeGoalDistance(std::queue<MapCell*>& dist_queue, const costmap_2d::Costmap2D& costmap);

      /**
       * @brief Update what cells are considered path based on the global plan 
       * 基于全局路径更新局部路径的下一个cell
       */
      void setTargetCells(const costmap_2d::Costmap2D& costmap, const std::vector<geometry_msgs::PoseStamped>& global_plan);

      /**
       * @brief Update what cell is considered the next local goal
       * 基于全局路径更新下一个目标点
       */
      void setLocalGoal(const costmap_2d::Costmap2D& costmap,
            const std::vector<geometry_msgs::PoseStamped>& global_plan);

      double goal_x_, goal_y_; /**< @brief The goal distance was last computed from */ //局部路径与目标点的距离。

      unsigned int size_x_, size_y_; ///< @brief The dimensions of the grid         //山歌地图的大小

    private:

      std::vector<MapCell> map_; ///< @brief Storage for the MapCells               //山歌地图内容数据。

  };
};

#endif
