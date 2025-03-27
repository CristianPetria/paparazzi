/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "modules/black_magic_green_avoider/black_magic_green_avoider.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);
static uint8_t chooseRandomIncrementAvoidanceProb(void);

// Update navigation_state_t enum to include new states (remove STUCK)
enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  RIGHT,  // New state for when top segment has low count
  CENTER, // New state for when middle segment has low count
  LEFT, // New state for when bottom segment has low count
};

// Add variables to track segment counts
static uint32_t segment_counts[3] = {0, 0, 0}; // Top (0), Middle (1), Bottom (2)


// define settings
float oa_color_count_frac = 0.16f;

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
int32_t stuck_state = 0;                // stuck state
float heading_increment = 5.f;          // heading angle increment [deg]
float maxDistance = 2.25;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;

// Modify the color detection callback to store segment information
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                              int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                              int16_t pixel_width, int16_t pixel_height,
                              int32_t quality, int16_t __attribute__((unused)) extra)
{
  // Store the segment counts that are coming in through the ABI message parameters
  segment_counts[0] = pixel_width;  // Top segment
  segment_counts[1] = pixel_height; // Middle segment
  segment_counts[2] = quality;      // Bottom segment

  // Total color count is the sum of all segments
//  color_count = segment_counts[0] + segment_counts[1] + segment_counts[2];
//
//  VERBOSE_PRINT("Segments - Top: %d, Middle: %d, Bottom: %d, Total: %d\n",
//                segment_counts[0], segment_counts[1], segment_counts[2], color_count);
}



/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void orange_avoider_init(void)
{VERBOSE_PRINT("BLACK MAGIC\n");
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}


static uint32_t segment_threshold = 3300; // Adjust based on testing
static int steps_since_direction_change = 0;
static int min_steps_before_check = 10;





/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
// Update orange_avoider_periodic() to remove STUCK state references and use segment counts
void orange_avoider_periodic(void)
{
  // Only evaluate our state machine if we are flying
  if(!autopilot_in_flight()) {
    return;
  }

  // Calculate total color count from all segments
  color_count = segment_counts[0] + segment_counts[1] + segment_counts[2];
  VERBOSE_PRINT("Segments - LEFT: %d, CENTER: %d, RIGHT: %d, Total: %d\n",
                segment_counts[0], segment_counts[1], segment_counts[2], color_count);
  // Compute current color thresholds
  //int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  // Update our safe confidence using color threshold
//  if(color_count >= color_count_threshold) {
//    obstacle_free_confidence++;
//  } else if (color_count < 18000) {
//    obstacle_free_confidence = 0;
//  } else {
//    obstacle_free_confidence -= 2; // Be more cautious with positive obstacle detections
//  }

  // Bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.5f);

  // Increment step counter if we're in SAFE state
  if (navigation_state == SAFE) {
    steps_since_direction_change++;
    VERBOSE_PRINT("Steps since direction change: %d\n", steps_since_direction_change);
  }


  // First check if we need a random direction change (priority)
  if (navigation_state == SAFE && steps_since_direction_change >= 125) {
    // Make a medium adjustment
    waypoint_move_here_2d(WP_GOAL);
    waypoint_move_here_2d(WP_RETREAT);
    waypoint_move_here_2d(WP_TRAJECTORY);

    heading_increment = 10.0f; // Medium turn
//    chooseRandomIncrementAvoidanceProb();
    increase_nav_heading(heading_increment);
    VERBOSE_PRINT("RANDOMNESS IS BY OUR SIDE, increasing heading by %f\n", heading_increment);

    // Reset step counter and stay in SAFE state
    steps_since_direction_change = min_steps_before_check + 1;
    // Don't set navigation_state = RANDOM, just do the turn directly
  }
  // Only check segments if we didn't just do a random turn
  else if (navigation_state == SAFE && steps_since_direction_change >= min_steps_before_check) {
    // Check each segment and respond if any is below threshold
    if(segment_counts[0] < segment_threshold) {
      // Low count in left segment - turn right
      navigation_state = LEFT;
      VERBOSE_PRINT("Left segment low count (%d), switching to LEFT state\n", segment_counts[0]);

    } else if(segment_counts[1] < segment_threshold) {
      // Low count in middle segment - turn randomly
      navigation_state = CENTER;
      VERBOSE_PRINT("Center segment low count (%d), switching to CENTER state\n", segment_counts[1]);

    } else if(segment_counts[2] < segment_threshold) {
      // Low count in right segment - turn left
      navigation_state = RIGHT;
      VERBOSE_PRINT("Right segment low count (%d), switching to RIGHT state\n", segment_counts[2]);

    }
  }

  switch (navigation_state) {
    case SAFE:
      // Move waypoint forward
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
        heading_increment = +13.0f;
        steps_since_direction_change = 0;
      }
      else {
        moveWaypointForward(WP_GOAL, moveDistance);
        moveWaypointForward(WP_RETREAT, -1.0f * moveDistance);
      }
      break;

    case RIGHT:
      // Turn right when obstacle detected in top segment
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      heading_increment = -13.0f; // Large turn to the right
      increase_nav_heading(heading_increment);
      VERBOSE_PRINT("RIGHT state, increasing heading by %f\n", heading_increment);
      navigation_state = SAFE;
      steps_since_direction_change = 0;
      break;

    case CENTER:
      // Make a medium adjustment
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      chooseRandomIncrementAvoidance();
//      heading_increment = 60.0f; // Medium turn
      increase_nav_heading(heading_increment);
      VERBOSE_PRINT("CENTER state, increasing heading by %f\n", heading_increment);
      navigation_state = SAFE;
      steps_since_direction_change = 0;
      break;

    case LEFT:
      // Turn left when obstacle detected in bottom segment
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_RETREAT);
      waypoint_move_here_2d(WP_TRAJECTORY);

      heading_increment = 13.0f; // Large turn to the left
      increase_nav_heading(heading_increment);
      VERBOSE_PRINT("LEFT state, increasing heading by %f\n", heading_increment);
      navigation_state = SAFE;
      steps_since_direction_change = 0;
      break;



//    case OBSTACLE_FOUND:
//      waypoint_move_here_2d(WP_GOAL);
//      waypoint_move_here_2d(WP_RETREAT);
//      waypoint_move_here_2d(WP_TRAJECTORY);
//      chooseRandomIncrementAvoidance();
//      navigation_state = SAFE;
//      break;

//    case SEARCH_FOR_SAFE_HEADING:
//      waypoint_move_here_2d(WP_GOAL);
//      waypoint_move_here_2d(WP_RETREAT);
//      waypoint_move_here_2d(WP_TRAJECTORY);
//      increase_nav_heading(heading_increment);
//      VERBOSE_PRINT("SEARCH FOR SAFE HEADING: %f\n", heading_increment);
//      if (obstacle_free_confidence > 0) {
//        navigation_state = SAFE;
//      }
//      break;

    case OUT_OF_BOUNDS:
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);
      moveWaypointForward(WP_RETREAT, -1.0f);
      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        increase_nav_heading(heading_increment);
//        obstacle_free_confidence = 0;
        navigation_state = SAFE;
      }
      break;

    default:
      break;
  }
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  nav.heading = new_heading;

//  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
//  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
//                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y),
//                stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, DegOfRad(heading));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
//  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
//                POS_FLOAT_OF_BFP(new_coor->y));
waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (segment_counts[2] < segment_threshold) {
    heading_increment = -35.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else if(segment_counts[0] < segment_threshold) {
    heading_increment = +35.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  else if (rand() % 2 == 0) {
    heading_increment = 35.f;
  }
  else {
    heading_increment = -35.f;
  }
  return false;
}

uint8_t chooseRandomIncrementAvoidanceProb(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = -25.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = +25.f;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

