/*****************************************************************************
* \file localvoice_music_nonum.h
*****************************************************************************
* \copyright
* Copyright 2024, Infineon Technologies.
* All rights reserved.
*****************************************************************************/

#ifndef LOCALVOICE_MUSIC_NONUM_H
#define LOCALVOICE_MUSIC_NONUM_H

#include <stdint.h>

#define LOCALVOICE_MUSIC_NONUM_NUM_INTENTS 15
#define LOCALVOICE_MUSIC_NONUM_NUM_COMMANDS 65
#define LOCALVOICE_MUSIC_NONUM_NUM_VARIABLES 0
#define LOCALVOICE_MUSIC_NONUM_NUM_VARIABLE_PHRASES 0
#define LOCALVOICE_MUSIC_NONUM_NUM_UNIT_PHRASES 16
#define LOCALVOICE_MUSIC_NONUM_INTENT_MAP_ARRAY_TOTAL_SIZE 130
#define LOCALVOICE_MUSIC_NONUM_UNIT_PHRASE_MAP_ARRAY_TOTAL_SIZE 65

extern const char* localvoice_music_nonum_intent_name_list[LOCALVOICE_MUSIC_NONUM_NUM_INTENTS];

extern const char* localvoice_music_nonum_variable_name_list[LOCALVOICE_MUSIC_NONUM_NUM_VARIABLES];

extern const char* localvoice_music_nonum_variable_phrase_list[LOCALVOICE_MUSIC_NONUM_NUM_VARIABLE_PHRASES];

extern const char* localvoice_music_nonum_unit_phrase_list[LOCALVOICE_MUSIC_NONUM_NUM_UNIT_PHRASES];

extern const int localvoice_music_nonum_intent_map_array[LOCALVOICE_MUSIC_NONUM_INTENT_MAP_ARRAY_TOTAL_SIZE];

extern const int localvoice_music_nonum_intent_map_array_sizes[LOCALVOICE_MUSIC_NONUM_NUM_COMMANDS];

extern const int localvoice_music_nonum_variable_phrase_sizes[LOCALVOICE_MUSIC_NONUM_NUM_VARIABLES];

extern const int localvoice_music_nonum_unit_phrase_map_array[LOCALVOICE_MUSIC_NONUM_UNIT_PHRASE_MAP_ARRAY_TOTAL_SIZE];

extern const int localvoice_music_nonum_unit_phrase_map_array_sizes[LOCALVOICE_MUSIC_NONUM_NUM_COMMANDS];

#endif // LOCALVOICE_MUSIC_NONUM_H
