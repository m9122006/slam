#include <stdint-gcc.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "slam.h"

/*s
 *
 * Code starts here.
 *
 */
ts_map_t vuw_global_map;

/**
 * Initialises (resets) the slam system.
 */
void vuw_slam_init() {
  //TODO maybe accept parameters for things like search space
  ts_map_init(&vuw_global_map);
}

/**
 * Copies the current SLAM map to the input array
 * @param map an array of unsigned short of size TS_MAP_SIZE^2
 */
void vuw_get_slam_map(ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE]) {
  memcpy(map, &vuw_global_map, sizeof(ts_map_pixel_t) * TS_MAP_SIZE * TS_MAP_SIZE);
}

/**
 * Fixes the error created from moving the lidar while scanning.
 * @param magnitude a pointer to raw magnitude values in mm.
 * @param theta a pointer to raw theta values in radians.
 * @param nb_points the number of values in m and t.
 * @param d_linear the delta in linear movement during the lidar scan in mm.
 * @param d_theta the delta in rotation during the lidar scan in radians.
 */
void vuw_process_lidar(float *magnitude, float *theta, float *x_out, float *y_out, int nb_points, float d_x, float d_y, float d_theta) {
  int i;
  for (i = 0; i != nb_points; i++) {
	  float d_x_at_i = d_x * ((float)i) / ((float)(nb_points-1));
	  float d_y_at_i = d_y * ((float)i) / ((float)(nb_points-1));
	  float rotation_at_i = d_theta * ((float)i) / ((float)(nb_points-1));
	  float dx =  d_x - d_x_at_i;
	  float dy =  d_y - d_y_at_i;
	  float dt =  d_theta - rotation_at_i;
	  x_out[i] = magnitude[i]*cos(theta[i] + dt) + dx;
	  y_out[i] = magnitude[i]*sin(theta[i] + dt) + dy;
  }
}

/**
 * Updates the slam map with the given lidar scan data.
 * @param x a pointer to raw x values.
 * @param y a pointer to raw y values.
 * @param nb_points the number of values in x and y.
 * @param est_x estimated current x position in mm.
 * @param est_y estimated current y position in mm.
 * @param est_theta estimated current rotation in radians.
 */
void vuw_slam_update(float *x, float *y, int nb_points, float *est_x, float *est_y, float *est_theta, int iterations) {
  ts_scan_t scan;
  scan.nb_points = nb_points;
  int i;
  for(i = 0; i != nb_points; i++) {
    scan.x[i] = x[i];
    scan.y[i] = y[i];
    float sq_dist = x[i]*x[i] + y[i]*y[i];
    if(sq_dist >= TS_DISTANCE_NO_DETECTION*TS_DISTANCE_NO_DETECTION || sq_dist <= TS_DISTANCE_ERROR*TS_DISTANCE_ERROR) {
      scan.value[i] = TS_NO_OBSTACLE;
    } else {
      scan.value[i] = TS_OBSTACLE;
    }
  }

  ts_position_t pos;
  pos.x = *est_x;
  pos.y = *est_y;
  pos.theta = *est_theta;

  ts_position_t search_size;
  search_size.x = VUW_SEARCH_SIZE_DISTANCE;
  search_size.y = VUW_SEARCH_SIZE_DISTANCE;
  search_size.theta = VUW_SEARCH_SIZE_ANGLE;

  int p_score = ts_distance_scan_to_map(&scan, &vuw_global_map, &pos);
  pos = vuw_slam_optimise(&scan, &vuw_global_map, pos, p_score, search_size, iterations);

  *est_x = pos.x;
  *est_y = pos.y;
  *est_theta = pos.theta;

  ts_map_update(&scan, &vuw_global_map, &pos, QUALITY);
}

/**
 * Optimise the robot position according to the map.
 * @param scan the lidar scan data.
 * @param map the slam map to check against.
 * @param p the estimated initial position of the robot.
 * @param p_score the distance score of the initial position.
 * @param s the size of the search space to look in.
 * @param n the number of iterations to do.
 */
ts_position_t vuw_slam_optimise(ts_scan_t *scan, ts_map_t *map, ts_position_t p, int p_score, ts_position_t s, int n) {
  if(n == 0) {
    return p;
  }
  int x, y, t;
  ts_position_t np;
  ts_position_t best_np = p;
  int best_np_score = 2000000000; // TODO p_score;
  int best_is_inside = 1;
  for(x = 0; x <= VUW_SEARCH_DIVISIONS; x++) {
    np.x = p.x + (((float)x)/VUW_SEARCH_DIVISIONS - 0.5) * s.x;
    for(y = 0; y <= VUW_SEARCH_DIVISIONS; y++) {
      np.y = p.y + (((float)y)/VUW_SEARCH_DIVISIONS - 0.5) * s.y;
      for(t = 0; t <= VUW_SEARCH_DIVISIONS; t++) {
        np.theta = p.theta + (((float)t)/VUW_SEARCH_DIVISIONS - 0.5) * s.theta;
        int score = ts_distance_scan_to_map(scan, map, &np);
        if(score < best_np_score) {
          best_np_score = score;
          best_np = np;
          best_is_inside = x > 0 && x < VUW_SEARCH_DIVISIONS
                        && y > 0 && y < VUW_SEARCH_DIVISIONS
                        && t > 0 && t < VUW_SEARCH_DIVISIONS;
        }
      }
    }
  }
  if(best_is_inside) {
    s.x /= VUW_SEARCH_DIVISIONS;
    s.y /= VUW_SEARCH_DIVISIONS;
    s.theta /= VUW_SEARCH_DIVISIONS;
  }
  return vuw_slam_optimise(scan, map, best_np, best_np_score, s, n-1);
}

/*
 *
 * ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^ ^
 * | | | | | | | | | | | | | | | | | | | | | | | |
 * | | | | | | | | | | | | | | | | | | | | | | | |
 * | | | | | | | | | | | | | | | | | | | | | | | |
 * Above is VUW code
 * Below is code from the tiny-slam paper
 * TODO put tiny-slam into it's own file
 *
 */

int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos) {
  float c, s;
  int i, x, y, nb_points = 0;
  int64_t sum;
  c = cos(pos->theta);
  s = sin(pos->theta);
  // Translate and rotate scan to robot position
  // and compute the distance
  for (i = 0, sum = 0; i != scan->nb_points; i++) {
    if (scan->value[i] != TS_NO_OBSTACLE) {
      x = (int) floor((pos->x + c * scan->x[i] - s * scan->y[i]) * TS_MAP_SCALE + 0.5);
      y = (int) floor((pos->y + s * scan->x[i] + c * scan->y[i]) * TS_MAP_SCALE + 0.5);
      // Check boundaries
      if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
        sum += map->map[y * TS_MAP_SIZE + x];
        nb_points++;
      }
    }
  }
  if (nb_points) sum = sum * 1024 / nb_points;
  else sum = 2000000000;
  return (int) sum;
}

void ts_map_init(ts_map_t *map) {
  int x, y, initval;
  ts_map_pixel_t *ptr;
  initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;
  for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
    for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
      *ptr = initval;
    }
  }
}

void ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos, int quality) {
  float c, s, q;
  float x2p, y2p;
  int i, x1, y1, x2, y2, xp, yp, value;
  float add, dist;
  c = cos(pos->theta);
  s = sin(pos->theta);
  x1 = (int) floor(pos->x * TS_MAP_SCALE + 0.5);
  y1 = (int) floor(pos->y * TS_MAP_SCALE + 0.5);
  // Translate and rotate scan to robot position
  for (i = 0; i != scan->nb_points; i++) {
    x2p = c * scan->x[i] - s * scan->y[i];
    y2p = s * scan->x[i] + c * scan->y[i];
    xp = (int) floor((pos->x + x2p) * TS_MAP_SCALE + 0.5);
    yp = (int) floor((pos->y + y2p) * TS_MAP_SCALE + 0.5);
    dist = sqrt(x2p * x2p + y2p * y2p);
    add = TS_HOLE_WIDTH / 2 / dist;
    x2p *= TS_MAP_SCALE * (1 + add);
    y2p *= TS_MAP_SCALE * (1 + add);
    x2 = (int) floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
    y2 = (int) floor(pos->y * TS_MAP_SCALE + y2p + 0.5);
    if (scan->value[i] == TS_NO_OBSTACLE) {
      q = quality / 2;
      value = TS_NO_OBSTACLE;
    } else {
      q = quality;
      value = TS_OBSTACLE;
    }
    ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
  }
}

#define SWAP(x, y) \
{\
	int temp = x;\
	x = y;\
	y = temp;\
}

void ts_map_laser_ray(ts_map_t *map, int x1, int y1, int x2, int y2, int xp, int yp, int value, int alpha) {
  int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
  int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
  ts_map_pixel_t *ptr;
  if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
    return; // Robot is out of map
  x2c = x2;
  y2c = y2;
  // Clipping
  if (x2c < 0) {
    if (x2c == x1) return;
    y2c += (y2c- y1) * (-x2c) / (x2c- x1);
    x2c = 0;
  }
  if (x2c >= TS_MAP_SIZE) {
    if (x1 == x2c) return;
    y2c += (y2c- y1) * (TS_MAP_SIZE- 1- x2c) / (x2c- x1);
    x2c = TS_MAP_SIZE- 1;
  }
  if (y2c < 0) {
    if (y1 == y2c) return;
    x2c += (x1- x2c) * (-y2c) / (y1- y2c);
    y2c = 0;
  }
  if (y2c >= TS_MAP_SIZE) {
    if (y1 == y2c) return;
    x2c += (x1- x2c) * (TS_MAP_SIZE- 1- y2c) / (y1- y2c);
    y2c = TS_MAP_SIZE- 1;
  }
  dx = abs(x2- x1);
  dy = abs(y2- y1);
  dxc = abs(x2c- x1);
  dyc = abs(y2c- y1);
  incptrx = (x2 > x1) ? 1 : -1;
  incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
  sincv = (value > TS_NO_OBSTACLE) ? 1 : -1;
  if (dx > dy) {
    derrorv = abs(xp- x2);
  } else {
	SWAP(dx, dy);
	SWAP(dxc, dyc);
	SWAP(incptrx, incptry);
    derrorv = abs(yp- y2);
  }
  error = 2 * dyc- dxc;
  horiz = 2 * dyc;
  diago = 2 * (dyc- dxc);
  errorv = derrorv / 2;
  incv = (value- TS_NO_OBSTACLE) / derrorv;
  incerrorv = value- TS_NO_OBSTACLE- derrorv * incv;
  ptr = map->map + y1 * TS_MAP_SIZE + x1;
  pixval = TS_NO_OBSTACLE;
  for (x = 0; x <= dxc; x++, ptr += incptrx) {
    if (x > dx- 2 * derrorv) {
      if (x <= dx- derrorv) {
        pixval += incv;
        errorv += incerrorv;
        if (errorv > derrorv) {
          pixval += sincv;
          errorv -= derrorv;
        }
      } else {
        pixval -= incv;
        errorv -= incerrorv;
        if (errorv < 0) {
          pixval -= sincv;
          errorv += derrorv;
        }
      }
    }
    // Integration into the map
    *ptr = ((256- alpha) * (*ptr) + alpha * pixval) >> 8;
    if (error > 0) {
      ptr += incptry;
      error += diago;
    }
    else error += horiz;
  }
}
