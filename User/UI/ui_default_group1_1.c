//
// Created by RM UI Designer
//

#include "ui_default_group1_1.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 1

ui_string_frame_t ui_default_group1_1;

ui_interface_string_t* ui_default_group1_Relay_Left = &ui_default_group1_1.option;

void _ui_init_default_group1_1() {
    ui_default_group1_1.option.figure_name[0] = FRAME_ID;
    ui_default_group1_1.option.figure_name[1] = GROUP_ID;
    ui_default_group1_1.option.figure_name[2] = START_ID;
    ui_default_group1_1.option.operate_tpyel = 1;
    ui_default_group1_1.option.figure_tpye = 7;
    ui_default_group1_1.option.layer = 0;
    ui_default_group1_1.option.font_size = 20;
    ui_default_group1_1.option.start_x = 780;
    ui_default_group1_1.option.start_y = 180;
    ui_default_group1_1.option.color = 3;
    ui_default_group1_1.option.str_length = 4;
    ui_default_group1_1.option.width = 2;
    strcpy(ui_default_group1_Relay_Left->string, "LEFT");

    ui_proc_string_frame(&ui_default_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_1, sizeof(ui_default_group1_1));
}

void _ui_update_default_group1_1() {
    ui_default_group1_1.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_default_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_1, sizeof(ui_default_group1_1));
}

void _ui_remove_default_group1_1() {
    ui_default_group1_1.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_default_group1_1);
    SEND_MESSAGE((uint8_t *) &ui_default_group1_1, sizeof(ui_default_group1_1));
}