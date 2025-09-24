/*******************************************************************************
* File Name        : lv_draw_vg_lite.c
*
* Description      : This file provides implementation of LVGL's drawing operations
*                    ported to VGLite library.
*
* Related Document : See README.md
*
********************************************************************************
* Copyright 2025-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*********************
 *      INCLUDES
 *********************/
#include "lv_draw_vg_lite.h"

#if LV_USE_DRAW_VG_LITE

#include "../lv_draw_private.h"
#include "lv_draw_vg_lite_type.h"
#include "lv_vg_lite_path.h"
#include "lv_vg_lite_utils.h"
#include "lv_vg_lite_decoder.h"
#include "lv_vg_lite_grad.h"
#include "lv_vg_lite_pending.h"
#include "lv_vg_lite_stroke.h"

/*********************
* Macros
*********************/
#define VG_LITE_DRAW_UNIT_ID 2

/**********************
 *  STATIC PROTOTYPES
 **********************/
int32_t draw_dispatch(lv_draw_unit_t * draw_unit, lv_layer_t * layer);
int32_t draw_evaluate(lv_draw_unit_t * draw_unit, lv_draw_task_t * task);
int32_t draw_delete(lv_draw_unit_t * draw_unit);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void LV_ATTRIBUTE_FAST_MEM lv_draw_vg_lite_init(void)
{
#if LV_VG_LITE_USE_GPU_INIT
    extern void gpu_init(void);
    static bool inited = false;
    if(!inited) {
        gpu_init();
        inited = true;
    }
#endif

    lv_vg_lite_dump_info();

    lv_draw_buf_vg_lite_init_handlers();

    lv_draw_vg_lite_unit_t * unit = lv_draw_create_unit(sizeof(lv_draw_vg_lite_unit_t));
    unit->base_unit.dispatch_cb = draw_dispatch;
    unit->base_unit.evaluate_cb = draw_evaluate;
    unit->base_unit.delete_cb = draw_delete;

    lv_vg_lite_image_dsc_init(unit);
#if LV_USE_VECTOR_GRAPHIC
    lv_vg_lite_grad_init(unit, LV_VG_LITE_GRAD_CACHE_CNT);
    lv_vg_lite_stroke_init(unit, LV_VG_LITE_STROKE_CACHE_CNT);
#endif
    lv_vg_lite_path_init(unit);
    lv_vg_lite_decoder_init();
}

void lv_draw_vg_lite_deinit(void)
{
}

static bool LV_ATTRIBUTE_FAST_MEM check_image_is_supported(const lv_draw_image_dsc_t * dsc)
{
    lv_image_header_t header;
    lv_result_t res = lv_image_decoder_get_info(dsc->src, &header);
    if(res != LV_RESULT_OK) {
        LV_LOG_TRACE("get image info failed");
        return false;
    }

    return lv_vg_lite_is_src_cf_supported((lv_color_format_t) header.cf);
}

static void LV_ATTRIBUTE_FAST_MEM draw_execute(lv_draw_vg_lite_unit_t * u)
{
    lv_draw_task_t * t = u->task_act;
    lv_draw_unit_t * draw_unit = (lv_draw_unit_t *)u;

    lv_layer_t * layer = u->base_unit.target_layer;

    lv_vg_lite_buffer_from_draw_buf(&u->target_buffer, layer->draw_buf);

    /* VG-Lite will output premultiplied image, set the flag correspondingly. */
    lv_draw_buf_set_flag(layer->draw_buf, LV_IMAGE_FLAGS_PREMULTIPLIED);

    vg_lite_identity(&u->global_matrix);
    vg_lite_translate(-layer->buf_area.x1, -layer->buf_area.y1, &u->global_matrix);

#if LV_DRAW_TRANSFORM_USE_MATRIX
    vg_lite_matrix_t layer_matrix;
    lv_vg_lite_matrix(&layer_matrix, &t->matrix);
    lv_vg_lite_matrix_multiply(&u->global_matrix, &layer_matrix);

    /* Crop out extra pixels drawn due to scaling accuracy issues */
    if(vg_lite_query_feature(gcFEATURE_BIT_VG_SCISSOR)) {
        lv_area_t scissor_area = layer->phy_clip_area;
        lv_area_move(&scissor_area, -layer->buf_area.x1, -layer->buf_area.y1);
        lv_vg_lite_set_scissor_area(&scissor_area);
    }
#endif

    switch(t->type) {
        case LV_DRAW_TASK_TYPE_LABEL:
            lv_draw_vg_lite_label(draw_unit, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_FILL:
            lv_draw_vg_lite_fill(draw_unit, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_BORDER:
            lv_draw_vg_lite_border(draw_unit, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_BOX_SHADOW:
            lv_draw_vg_lite_box_shadow(draw_unit, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_IMAGE:
            lv_draw_vg_lite_img(draw_unit, t->draw_dsc, &t->area, false);
            break;
        case LV_DRAW_TASK_TYPE_ARC:
            lv_draw_vg_lite_arc(draw_unit, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_LINE:
            lv_draw_vg_lite_line(draw_unit, t->draw_dsc);
            break;
        case LV_DRAW_TASK_TYPE_LAYER:
            lv_draw_vg_lite_layer(draw_unit, t->draw_dsc, &t->area);
            break;
        case LV_DRAW_TASK_TYPE_TRIANGLE:
            lv_draw_vg_lite_triangle(draw_unit, t->draw_dsc);
            break;
        case LV_DRAW_TASK_TYPE_MASK_RECTANGLE:
            lv_draw_vg_lite_mask_rect(draw_unit, t->draw_dsc, &t->area);
            break;
#if LV_USE_VECTOR_GRAPHIC
        case LV_DRAW_TASK_TYPE_VECTOR:
            lv_draw_vg_lite_vector(draw_unit, t->draw_dsc);
            break;
#endif
        default:
            break;
    }

    lv_vg_lite_flush(u);
}

int32_t LV_ATTRIBUTE_FAST_MEM draw_dispatch(lv_draw_unit_t * draw_unit, lv_layer_t * layer)
{
    lv_draw_vg_lite_unit_t * u = (lv_draw_vg_lite_unit_t *)draw_unit;

    /* Return immediately if it's busy with draw task. */
    if(u->task_act) {
        return 0;
    }

    /* Try to get an ready to draw. */
    lv_draw_task_t * t = lv_draw_get_next_available_task(layer, NULL, VG_LITE_DRAW_UNIT_ID);

    /* Return 0 is no selection, some tasks can be supported by other units. */
    if(!t || t->preferred_draw_unit_id != VG_LITE_DRAW_UNIT_ID) {
        lv_vg_lite_finish(u);
        return LV_DRAW_UNIT_IDLE;
    }

    /* Return if target buffer format is not supported. */
    if(!lv_vg_lite_is_dest_cf_supported(layer->color_format)) {
        return LV_DRAW_UNIT_IDLE;
    }

    void * buf = lv_draw_layer_alloc_buf(layer);
    if(!buf) {
        return LV_DRAW_UNIT_IDLE;
    }

    t->state = LV_DRAW_TASK_STATE_IN_PROGRESS;
    u->base_unit.target_layer = layer;
    u->base_unit.clip_area = &t->clip_area;
    u->task_act = t;

    draw_execute(u);

    u->task_act->state = LV_DRAW_TASK_STATE_READY;
    u->task_act = NULL;

    /*The draw unit is free now. Request a new dispatching as it can get a new task*/
    lv_draw_dispatch_request();

    return 1;
}

int32_t LV_ATTRIBUTE_FAST_MEM draw_evaluate(lv_draw_unit_t * draw_unit, lv_draw_task_t * task)
{
    LV_UNUSED(draw_unit);

    /* Return if target buffer format is not supported. */
    const lv_draw_dsc_base_t * base_dsc = task->draw_dsc;
    if(!lv_vg_lite_is_dest_cf_supported(base_dsc->layer->color_format)) {
        return -1;
    }

    switch(task->type) {
        case LV_DRAW_TASK_TYPE_FILL:
        case LV_DRAW_TASK_TYPE_BORDER:
#if LV_VG_LITE_USE_BOX_SHADOW
        case LV_DRAW_TASK_TYPE_BOX_SHADOW:
#endif
        case LV_DRAW_TASK_TYPE_LAYER:
        case LV_DRAW_TASK_TYPE_LINE:
        case LV_DRAW_TASK_TYPE_ARC:
        case LV_DRAW_TASK_TYPE_TRIANGLE:
        case LV_DRAW_TASK_TYPE_MASK_RECTANGLE:

#if LV_USE_VECTOR_GRAPHIC
        case LV_DRAW_TASK_TYPE_VECTOR:
#endif
            break;

        case LV_DRAW_TASK_TYPE_IMAGE: {
                if(!check_image_is_supported(task->draw_dsc)) {
                    return 0;
                }
            }
            break;

        default:
            /*The draw unit is not able to draw this task. */
            return 0;
    }

    /* The draw unit is able to draw this task. */
    task->preference_score = 80;
    task->preferred_draw_unit_id = VG_LITE_DRAW_UNIT_ID;
    return 1;
}

int32_t LV_ATTRIBUTE_FAST_MEM draw_delete(lv_draw_unit_t * draw_unit)
{
    lv_draw_vg_lite_unit_t * unit = (lv_draw_vg_lite_unit_t *)draw_unit;

    lv_vg_lite_image_dsc_deinit(unit);
#if LV_USE_VECTOR_GRAPHIC
    lv_vg_lite_grad_deinit(unit);
    lv_vg_lite_stroke_deinit(unit);
#endif
    lv_vg_lite_path_deinit(unit);
    lv_vg_lite_decoder_deinit();
    return 1;
}

#endif /*LV_USE_DRAW_VG_LITE*/
