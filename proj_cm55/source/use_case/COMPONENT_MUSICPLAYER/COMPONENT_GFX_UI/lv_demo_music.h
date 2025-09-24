/**
 * @file lv_demo_music.h
 *
 */

#ifndef LVGL_DEMO_MUSIC_H
#define LVGL_DEMO_MUSIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "lv_demos.h"


/*********************
 *      DEFINES
 *********************/

#if LV_USE_DEMO_MUSIC

#if LV_DEMO_MUSIC_LARGE
#define LV_DEMO_MUSIC_HANDLE_SIZE   40
#else
#define LV_DEMO_MUSIC_HANDLE_SIZE   20
#endif

#ifdef ENABLE_MP3_PLAYBACK
    #define ACTIVE_TRACK_CNT        5
#else
    #define ACTIVE_TRACK_CNT        3
#endif /* ENABLE_MP3_PLAYBACK */

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void lv_demo_music(void);
const char * lv_demo_music_get_title(uint32_t track_id);
const char * lv_demo_music_get_artist(uint32_t track_id);
const char * lv_demo_music_get_genre(uint32_t track_id);
uint32_t lv_demo_music_get_track_length(uint32_t track_id);

/**********************
 *      MACROS
 **********************/

#endif /*LV_USE_DEMO_MUSIC*/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LVGL_DEMO_MUSIC_H*/
