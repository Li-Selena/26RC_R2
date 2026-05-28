#ifndef __SPEED_PLANNER_H
#define __SPEED_PLANNER_H

#include "stdint.h"

typedef struct{
    float target_v; //rpm
    float current_v;
    float accel;    //rpm/s2
    float decel;
    float dt;       //ms
} speedPlanner_t;

/* S-curve (jerk-limited) speed planner */
typedef struct {
    float target_v;     /* rpm     - target velocity */
    float current_v;    /* rpm     - current velocity */
    float current_a;    /* rpm/s²  - current acceleration */
    float max_accel;    /* rpm/s²  - max acceleration (positive limit) */
    float max_decel;    /* rpm/s²  - max deceleration (positive magnitude) */
    float jerk;         /* rpm/s³  - jerk limit (rate of accel change per dt) */
    float dt;           /* s       - cycle time (same unit convention as speedPlanner_t) */
} SCurvePlanner_t;

extern speedPlanner_t SP_user;
extern float dt_flag;

void speedPlanner_Init(speedPlanner_t *planner, float accel, float decel, float cycle_time);
void speedPlanner_SetTarget(speedPlanner_t *planner, float target_v);
float speedPlanner_Update(speedPlanner_t *planner);

void SCurvePlanner_Init(SCurvePlanner_t *p, float max_accel, float max_decel, float jerk, float dt);
void SCurvePlanner_SetTarget(SCurvePlanner_t *p, float target_v);
float SCurvePlanner_Update(SCurvePlanner_t *p);

#endif
