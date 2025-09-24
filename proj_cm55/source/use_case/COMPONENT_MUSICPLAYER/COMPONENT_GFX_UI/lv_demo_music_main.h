/**
 * @file lv_demo_music_main.h
 *
 */

#ifndef LV_DEMO_MUSIC_MAIN_H
#define LV_DEMO_MUSIC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lv_demo_music.h"

#if LV_USE_DEMO_MUSIC

#if LV_USE_GRID == 0
#error "LV_USE_GRID needs to be enabled"
#endif

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/
extern int32_t vol_level_index;


/**********************
 * GLOBAL PROTOTYPES
 **********************/
lv_obj_t * lv_demo_music_main_create(lv_obj_t * parent);
void lv_demo_music_play(uint32_t id);
void lv_demo_music_resume(void);
void lv_demo_music_pause(void);
void lv_demo_music_album_next(bool next);
void update_volume_level(int32_t new_volume_index);
void reset_seek_bar(void);

/**********************
 *      MACROS
 **********************/
#endif /*LV_USE_DEMO_MUSIC*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_DEMO_MUSIC_MAIN_H*/
