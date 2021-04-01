#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <random_numbers/random_numbers.h>
#include <ros/ros.h>

namespace kobold_crawler
{

/*
 * Potential Improvements:
 * - write highly-parallelized kernel convolution
 * - speed-test custom kernel convolution vs opencv
 * - change from visited vs unvisited maps to a boolean array
 * - speed-test visited vs unvisited maps vs boolean array
 * - actual GUI
 * - allow imput of different kernels - horizontal striations could indicate 'cross section'
 * - try 'overworld maps' - elevation differences, cost going up, no negative cost going down
 * - nodes of 'value'
 * - parallelization of Dijkstra's
 * - if nothing else, parallelize planning of multiple routes - although there will be 'merging'
 *   required and it may result in slightly more dense paths
 *   - simple merging algorithm:
 *     for (each 'new map')
 *     {
 *       partial_update = new[travelled] - old[travelled];
 *       total_update += partial_update;
 *     }
 *     map = old + total_update;
 * - display of new paths as they appear
 * - maps are fairly small memory-wise - being able to step back and forth between iterations
 * - parameterize map size
 * - allow 'reset' to new map
 */

// Represents the location of a cell in a map/image
struct CoordinatePair
{
  int i;
  int j;

  bool operator==(const CoordinatePair& rhs)
  {
    return (i == rhs.i) && (j == rhs.j);
  }

  bool operator!=(const CoordinatePair& rhs)
  {
    return (i != rhs.i) || (j != rhs.j);
  }

  bool operator<(const CoordinatePair& rhs) const
  {
    return (i < rhs.i) || ((i == rhs.i) && (j < rhs.j));
  }
};

// represents a single cell in the map/image
struct Cell
{
  CoordinatePair loc;   // location of this cell
  CoordinatePair prev;  // Location of previous cell
  int r;                // resistance / toughness of this cell
  int d;                // distance from start of path

  bool operator==(const Cell& rhs)
  {
    return (loc == rhs.loc) && (prev == rhs.prev) && (r == rhs.r) && (d == rhs.d);
  }

  bool operator!=(const Cell& rhs)
  {
    return (loc != rhs.loc) || (prev != rhs.prev) || (r != rhs.r) || (d != rhs.d);
  }
};

/**
 * @brief init_map - original implementation of init_map, using hand-made kernel convolution
 * @param i_extent
 * @param j_extent
 * @param rng
 * @return
 */
cv::Mat init_map(int i_extent, int j_extent, random_numbers::RandomNumberGenerator& rng)
{
  // Initialize entire map to 0 usage and 255 resistance
  cv::Mat map (i_extent, j_extent, CV_8UC3, cv::Scalar(0,255,0));
  ROS_INFO_STREAM("rows: " << map.rows);
  ROS_INFO_STREAM("columns: " << map.cols);

  // Fill most of the map with random resistance, leaving high-resistance border
  for (int i = 2; i < i_extent-2; ++i)
  {
    for (int j = 2; j < j_extent-2; ++j)
    {
      map.at<cv::Vec3b>(i,j)[1] = rng.uniformInteger(50, 200);
    }
  }

  // Smooth out the random resistance somewhat by passing a weighted 5x5 kernel over the image, and
  // return the smoothed image
  cv::Mat map2 = map.clone();
  for (int i = 2; i < i_extent-2; ++i)
  {
    for (int j = 2; j < j_extent-2; ++j)
    {
      map2.at<cv::Vec3b>(i,j)[1] =
          (
          map.at<cv::Vec3b>(i+2,j+2)[1] +   map.at<cv::Vec3b>(i+2,j+1)[1] +    map.at<cv::Vec3b>(i+2,j)[1] +   map.at<cv::Vec3b>(i+2,j-1)[1] + map.at<cv::Vec3b>(i+2,j-2)[1] +
          map.at<cv::Vec3b>(i+1,j+2)[1] + 2*map.at<cv::Vec3b>(i+1,j+1)[1] +  2*map.at<cv::Vec3b>(i+1,j)[1] + 2*map.at<cv::Vec3b>(i+1,j-1)[1] + map.at<cv::Vec3b>(i+1,j-2)[1] +
          map.at<cv::Vec3b>(i,j+2)[1]   + 2*map.at<cv::Vec3b>(i,j+1)[1]   + 16*map.at<cv::Vec3b>(i,j)[1]   + 2*map.at<cv::Vec3b>(i,j-1)[1]   + map.at<cv::Vec3b>(i,j-2)[1] +
          map.at<cv::Vec3b>(i-1,j+2)[1] + 2*map.at<cv::Vec3b>(i-1,j+1)[1] +  2*map.at<cv::Vec3b>(i-1,j)[1] + 2*map.at<cv::Vec3b>(i-1,j-1)[1] + map.at<cv::Vec3b>(i-1,j-2)[1] +
          map.at<cv::Vec3b>(i-2,j+2)[1] +   map.at<cv::Vec3b>(i-2,j+1)[1] +    map.at<cv::Vec3b>(i-2,j)[1] +   map.at<cv::Vec3b>(i-2,j-1)[1] + map.at<cv::Vec3b>(i-2,j-2)[1]
          ) / 48;
    }
  }

  return map2;
}

/**
 * @brief init_map_cv - simplified map initialization using opencv kernel convolution
 * @param i_extent
 * @param j_extent
 * @param rng
 * @return
 */
cv::Mat init_map_cv(int i_extent, int j_extent, random_numbers::RandomNumberGenerator& rng)
{
  // Create our kernel - static to avoid re-instantiation
  static float kernel_data[] =
  {
    1, 1,  1, 1, 1,
    1, 2,  2, 2, 1,
    1, 2, 16, 2, 1,
    1, 2,  2, 2, 1,
    1, 1,  1, 1, 1
  };
  static cv::Mat kernel = cv::Mat(5, 5, CV_32F, kernel_data) / 48.0f;

  // Initialize entire map to 0 usage and 255 resistance
  cv::Mat map (i_extent, j_extent, CV_8UC3, cv::Scalar(0,255,0));
  ROS_INFO_STREAM("rows: " << map.rows);
  ROS_INFO_STREAM("columns: " << map.cols);

  // Fill most of the map with random resistance, leaving high-resistance border
  for (int i = 2; i < i_extent-2; ++i)
  {
    for (int j = 2; j < j_extent-2; ++j)
    {
      map.at<cv::Vec3b>(i,j)[1] = rng.uniformInteger(50, 200);
    }
  }

  // Smooth out the random resistance somewhat by passing a weighted 5x5 kernel over the image, and
  // return the smoothed image
  cv::Mat map2 = map.clone();
  cv::filter2D(map, map2, -1, kernel, cv::Point(-1, 1), 0, cv::BORDER_DEFAULT);
  return map2;
}

CoordinatePair findMinDist(const std::map<CoordinatePair, Cell>& cells)
{
  // Initialize min_pair to something outside of range
  std::pair<CoordinatePair, Cell> min;

  // Verify there are elements in the map
  if (cells.begin() != cells.end())
  {
    // If there are, min_pair is the first one
    min = *(cells.begin());

    // Now crawl through looking for the min
    for (const std::pair<CoordinatePair, Cell>& p : cells)
    {
      if (p.second.d < min.second.d)
      {
        min = p;
      }
    }
  }
  else
  {
    min.first.i = min.first.j = -1;
  }

  // return the lowest distance we found
  return min.first;
}

std::vector<CoordinatePair> pathfind(const CoordinatePair& start, const CoordinatePair& end, const cv::Mat& map)
{
  // initialize visited and unvisited cell lists
  std::map<CoordinatePair, Cell> visited_cells;
  std::map<CoordinatePair, Cell> unvisited_cells;

  // Make all cells unvisited
  for (int i = 2; i < map.rows-2; ++i)
  {
    for (int j = 2; j < map.cols-2; ++j)
    {
      Cell p;
      p.loc.i = i;
      p.loc.j = j;
      p.r = map.at<cv::Vec3b>(i,j)[1];
      p.d = std::numeric_limits<int>::max();
      p.prev.i = -1;
      p.prev.j = -1;
      unvisited_cells[p.loc] = p;
    }
  }

  // Verify the start and end points
  if (unvisited_cells.find(start) == unvisited_cells.end())
  {
    ROS_ERROR_STREAM("Error, invalid start location ");
    std::vector<CoordinatePair> empty;
    return empty;
  }
  if (unvisited_cells.find(end) == unvisited_cells.end())
  {
    ROS_ERROR("Error, invalid end location");
    std::vector<CoordinatePair> empty;
    return empty;
  }

  // Set the start point
  unvisited_cells[start].d = 0;

  // run dijkstra's algorithm, using resistance of each cell as the distance to it
  bool keep_going = true;
  CoordinatePair c;
  while (keep_going)
  {
    // find the unvisited point with the least cost
    c = findMinDist(unvisited_cells);

    // Test the result
    if (unvisited_cells.find(c) == unvisited_cells.end())
    {
      // if the point does not exist, we are done
      ROS_DEBUG("Path to end found");
      keep_going = false;
    }
    else if (unvisited_cells[c].d == std::numeric_limits<int>::max())
    {
      // if the point has max distance (it is unreachable) we are done
      keep_going = false;
      ROS_ERROR("Dead end");
    }
    else
    {
      // if the point was good, check each unvisited neighbor point
      std::vector<CoordinatePair> test_list =
      {
        {c.i+1, c.j},
        {c.i-1, c.j},
        {c.i, c.j+1},
        {c.i, c.j-1},
        {c.i+1, c.j+1},
        {c.i+1, c.j-1},
        {c.i-1, c.j+1},
        {c.i-1, c.j-1}
      };
      for (const CoordinatePair& test_point : test_list)
      {
        // Make sure neighbor point is unvisited
        if (unvisited_cells.find(test_point) != unvisited_cells.end())
        {
          // If unvisited, check if the neighbor point can be reached with shorter distance from here.
          if (unvisited_cells[c].d + unvisited_cells[test_point].r < unvisited_cells[test_point].d)
          {
            // If we have found a new shortest path to the neighbor point, update its previous and distance
            unvisited_cells[test_point].prev = c;
            unvisited_cells[test_point].d = unvisited_cells[c].d + unvisited_cells[test_point].r;
          }
        }
      }
    }

    // Whatever happened, move this point from unvisited to visited
    visited_cells[c] = unvisited_cells[c];
    unvisited_cells.erase(c);
  }

  // Now that we have exited the first loop, we can see if we formed a complete path
  std::vector<CoordinatePair> path;
  if (visited_cells.find(end) != visited_cells.end())
  {
    // if the endpoint is in the visited cells, we must have reached it. Crawl along `prev` fields
    // and build a path, but include a guard against infinite loops.
    CoordinatePair p = end;
    std::size_t timer = visited_cells.size();
    while (p != start && 0 < timer--)
    {
      path.push_back(p);
      p = visited_cells[p].prev;
    }
  }
  else
  {
    ROS_ERROR("End was not found!");
  }

  /*
  ROS_INFO_STREAM("Start " << start.i << " " << start.j);
  ROS_INFO_STREAM("End " << end.i << " " << end.j);
  ROS_INFO_STREAM("C " << c.i << " " << c.j);
  ROS_INFO_STREAM("unvisited " << unvisited_cells.size());
  ROS_INFO_STREAM("visited " << visited_cells.size());
  */

  return path;
}

cv::Mat explore(const cv::Mat& map, random_numbers::RandomNumberGenerator& rng)
{
  // Generate random start and end points
  CoordinatePair start;
  start.i = rng.uniformInteger(2, map.rows-2);
  start.j = rng.uniformInteger(2, map.cols-2);
  CoordinatePair end;
  end.i = rng.uniformInteger(2, map.rows-2);
  end.j = rng.uniformInteger(2, map.cols-2);

  // Find a path
  std::vector<CoordinatePair> path = pathfind(start, end, map);
  ROS_DEBUG_STREAM("path length: " << path.size());

  // Make the Kobolds 'mine out' the path
  cv::Mat new_map = map.clone();
  for (const CoordinatePair& p : path)
  {
    if (new_map.at<cv::Vec3b>(p.i, p.j)[0] < 250) // prevent overflow errors
    {
      new_map.at<cv::Vec3b>(p.i, p.j)[1] = 5;
      new_map.at<cv::Vec3b>(p.i, p.j)[0] += 5;
    }
    else
    {
      new_map.at<cv::Vec3b>(p.i, p.j)[1] += 5;
      new_map.at<cv::Vec3b>(p.i, p.j)[0] = 255;
    }
  }

  return new_map;
}

} // namespace kobold_crawler

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kobold_crawler");
  ros::NodeHandle nh;
  random_numbers::RandomNumberGenerator rng;

  cv::Mat map = kobold_crawler::init_map_cv(200, 200, rng);

  bool keep_going = true;
  while (keep_going)
  {
    cv::imshow("Map", map);
    cv::waitKey(10000);
    cv::destroyAllWindows();

    // Get number of paths from user
    int num_paths = 0;
    std::cout << "\nPlease enter a number of paths to generate,\n    0 to save, or a negative number to quit: ";
    std::cin >> num_paths;

    if (num_paths == 0)
    {
      cv::imwrite("/tmp/map.png", map);
    }
    else if (num_paths < 0)
    {
      keep_going = false;
    }
    else
    {
      for (int i = 0; i < num_paths; ++i)
      {
        ROS_INFO_STREAM("explore " << i);
        map = kobold_crawler::explore(map, rng);
      }
    }

  }

  return 0;
}
