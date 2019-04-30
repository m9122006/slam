#ifndef _CoreSLAM_H_
#define _CoreSLAM_H_

/* slam parameters */
#define TS_SCAN_SIZE 1024
#define TS_MAP_SIZE 512
#define TS_MAP_SCALE 0.1
#define TS_DISTANCE_NO_DETECTION 4000
#define TS_NO_OBSTACLE 65500
#define TS_OBSTACLE 0
#define TS_HOLE_WIDTH 600
#define QUALITY 50
#define TS_DISTANCE_ERROR 20

/* optimisation parameters */
#define VUW_SEARCH_DIVISIONS 3.0 // actually this is divisions-1
#define VUW_SEARCH_SIZE_DISTANCE 20.0 // mm
#define VUW_SEARCH_SIZE_ANGLE (2.0*3.14/180.0) // radians

/* typedefs */
typedef unsigned short ts_map_pixel_t;
typedef struct {
  ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE];
} ts_map_t;

typedef struct {
  float x[TS_SCAN_SIZE], y[TS_SCAN_SIZE];
  int value[TS_SCAN_SIZE];
  int nb_points;
} ts_scan_t;

typedef struct {
  float x, y; // mm
  float theta; // radians
} ts_position_t;

/* vuw functions */
void vuw_slam_init();
int vuw_slam_test() { return 42; }
void vuw_get_slam_map(ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE]);
void vuw_process_lidar(float *magnitude, float *theta, float *x_out, float *y_out, int nb_points, float d_x, float d_y, float d_theta);
void vuw_slam_update(float *x, float *y, int nb_points, float *est_x, float *est_y, float *est_theta, int iterations);
ts_position_t vuw_slam_optimise(ts_scan_t *scan, ts_map_t *map, ts_position_t p, int p_score, ts_position_t s, int n);

/* tiny slam functions */
void ts_map_init(ts_map_t *map);
int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos);
void ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *position, int quality);
void ts_map_laser_ray(ts_map_t *map, int x1, int y1, int x2, int y2, int xp, int yp, int value, int alpha);

#endif //_CoreSLAM_H_
