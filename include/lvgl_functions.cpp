#pragma once
#include "include.cpp"
#define BAR_OFFSET 35
#define BAR_HEIGHT 200
#define BAR_WIDTH 30
#define BAR_RANGE_MIN 0
#define BAR_RANGE_MAX 100
#define ANIM_TIME 200

#define WARNING_THRESH 45
#define DANGER_THRESH 55

constexpr float VAL_LABEL_CONSTANT = BAR_HEIGHT/(BAR_RANGE_MAX - BAR_RANGE_MIN);

lv_obj_t* parent = lv_obj_create(lv_scr_act(), NULL);

lv_obj_t* barFrontLeft = lv_bar_create(parent, NULL);
lv_obj_t* barMidLeft = lv_bar_create(parent, NULL);
lv_obj_t* barBackLeft = lv_bar_create(parent, NULL);
lv_obj_t* barFrontRight = lv_bar_create(parent, NULL);
lv_obj_t* barMidRight = lv_bar_create(parent, NULL);
lv_obj_t* barBackRight = lv_bar_create(parent, NULL);
lv_obj_t* barLift = lv_bar_create(parent, NULL);
lv_obj_t* barConveyor = lv_bar_create(parent, NULL);

lv_obj_t* bar1Lable = lv_label_create(parent, NULL);
lv_obj_t* bar2Lable = lv_label_create(parent, NULL);
lv_obj_t* bar3Lable = lv_label_create(parent, NULL);
lv_obj_t* bar4Lable = lv_label_create(parent, NULL);
lv_obj_t* bar5Lable = lv_label_create(parent, NULL);
lv_obj_t* bar6Lable = lv_label_create(parent, NULL);
lv_obj_t* bar7Lable = lv_label_create(parent, NULL);
lv_obj_t* bar8Lable = lv_label_create(parent, NULL);

lv_obj_t* bar1Val = lv_label_create(parent, NULL);
lv_obj_t* bar2Val = lv_label_create(parent, NULL);
lv_obj_t* bar3Val = lv_label_create(parent, NULL);
lv_obj_t* bar4Val = lv_label_create(parent, NULL);
lv_obj_t* bar5Val = lv_label_create(parent, NULL);
lv_obj_t* bar6Val = lv_label_create(parent, NULL);
lv_obj_t* bar7Val = lv_label_create(parent, NULL);
lv_obj_t* bar8Val = lv_label_create(parent, NULL);

lv_style_t parentStyle;
lv_style_t barStyle;
lv_style_t barIndcDefaultStyle;
lv_style_t barIndcWarningStyle;
lv_style_t barIndcDangerStyle;
lv_style_t textStyle;


void setBarAttributes(lv_obj_t* barObj, lv_obj_t* allignTo, int offSet = BAR_OFFSET){
    lv_bar_set_range(barObj, BAR_RANGE_MIN, BAR_RANGE_MAX);
    lv_bar_set_style(barObj, LV_BAR_STYLE_BG, &barStyle);
    lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcDefaultStyle);
    lv_obj_set_size(barObj, BAR_WIDTH, BAR_HEIGHT);
    lv_obj_align(barObj, allignTo, LV_ALIGN_IN_LEFT_MID, offSet, 0);
}


void setBarValAndColor(lv_obj_t* barObj, int val, lv_obj_t* labelObj){
    lv_bar_set_value_anim(barObj, val, ANIM_TIME);
    if(val >= DANGER_THRESH){lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcDangerStyle);}
    else if(val >= WARNING_THRESH){lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcWarningStyle);}
    else{lv_bar_set_style(barObj, LV_BAR_STYLE_INDIC, &barIndcDefaultStyle);}
    char buffer[5];
    sprintf(buffer, "%i", val);
    lv_label_set_text(labelObj, buffer);
    lv_obj_align(labelObj, barObj, LV_ALIGN_IN_BOTTOM_MID, 0, -(val*VAL_LABEL_CONSTANT));
}


void initBarGraph(){
    //INITIALIZE STYLES
    lv_style_copy(&parentStyle, &lv_style_plain);
    parentStyle.body.opa = 0;
    parentStyle.body.radius = 0;

    lv_style_copy(&barStyle, &lv_style_transp);

    lv_style_copy(&barIndcDefaultStyle, &lv_style_plain);
    barIndcDefaultStyle.body.main_color = LV_COLOR_MAKE(0, 255, 0);
    barIndcDefaultStyle.body.grad_color = LV_COLOR_MAKE(0, 255, 0);
    barIndcDefaultStyle.body.radius = 0;
    barIndcDefaultStyle.body.border.opa = LV_OPA_0;
    barIndcDefaultStyle.body.padding.hor = 0;            
    barIndcDefaultStyle.body.padding.ver = 0;

    lv_style_copy(&barIndcWarningStyle, &barIndcDefaultStyle);
    barIndcWarningStyle.body.main_color = LV_COLOR_MAKE(255, 255, 0);
    barIndcWarningStyle.body.grad_color = LV_COLOR_MAKE(255, 255, 0);

    lv_style_copy(&barIndcDangerStyle, &barIndcDefaultStyle);
    barIndcDangerStyle.body.main_color = LV_COLOR_MAKE(255, 0, 0);
    barIndcDangerStyle.body.grad_color = LV_COLOR_MAKE(255, 0, 0);

    lv_style_copy(&textStyle, &lv_style_plain);
    textStyle.text.color = LV_COLOR_WHITE;
    
    //SET THE PARENT OBJECT ATTRIBUTES

    lv_obj_set_style(parent, &parentStyle);
    lv_obj_set_pos(parent, 0, 0);
    lv_obj_set_size(parent, 340, 240);

    //SET THE BAR ATTRIBUTES

    setBarAttributes(barFrontLeft, NULL, 10);
    setBarAttributes(barMidLeft, barFrontLeft);
    setBarAttributes(barBackLeft, barMidLeft);
    setBarAttributes(barFrontRight, barBackLeft);
    setBarAttributes(barMidRight, barFrontRight);
    setBarAttributes(barBackRight, barMidRight);
    setBarAttributes(barLift, barBackRight);
    setBarAttributes(barConveyor, barLift);

    //SET THE LABEL ATTRIBUTES

    lv_label_set_text(bar1Lable, "FL");
    lv_label_set_style(bar1Lable, &textStyle);
    lv_obj_align(bar1Lable, barFrontLeft, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar2Lable, "ML");
    lv_label_set_style(bar2Lable, &textStyle);
    lv_obj_align(bar2Lable, barMidLeft, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar3Lable, "BL");
    lv_label_set_style(bar3Lable, &textStyle);
    lv_obj_align(bar3Lable, barBackLeft, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar4Lable, "FR");
    lv_label_set_style(bar4Lable, &textStyle);
    lv_obj_align(bar4Lable, barFrontRight, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar5Lable, "MR");
    lv_label_set_style(bar5Lable, &textStyle);
    lv_obj_align(bar5Lable, barMidRight, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar6Lable, "BR");
    lv_label_set_style(bar6Lable, &textStyle);
    lv_obj_align(bar6Lable, barBackRight, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar7Lable, "L");
    lv_label_set_style(bar7Lable, &textStyle);
    lv_obj_align(bar7Lable, barLift, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    lv_label_set_text(bar8Lable, "C");
    lv_label_set_style(bar8Lable, &textStyle);
    lv_obj_align(bar8Lable, barConveyor, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

    //SET STYLE FOR VAL LABELS
    lv_label_set_style(bar1Val, &textStyle);
    lv_label_set_style(bar2Val, &textStyle);
    lv_label_set_style(bar3Val, &textStyle);
    lv_label_set_style(bar4Val, &textStyle);
    lv_label_set_style(bar5Val, &textStyle);
    lv_label_set_style(bar6Val, &textStyle);
    lv_label_set_style(bar7Val, &textStyle);
    lv_label_set_style(bar8Val, &textStyle);


    setBarValAndColor(barFrontLeft, 35, bar1Val);
    setBarValAndColor(barMidLeft, 35, bar2Val);
    setBarValAndColor(barBackLeft, 35, bar3Val);
    setBarValAndColor(barFrontRight, 35, bar4Val);
    setBarValAndColor(barMidRight, 35, bar5Val);
    setBarValAndColor(barBackRight, 35, bar6Val);
    setBarValAndColor(barLift, 35, bar7Val);
    setBarValAndColor(barConveyor, 35, bar8Val);
}

void updateBarGraph(){
    setBarValAndColor(barFrontLeft, frontleft.get_temperature(), bar1Val);
    setBarValAndColor(barMidLeft, midleft.get_temperature(), bar2Val);
    setBarValAndColor(barBackLeft, backleft.get_temperature(), bar3Val);
    setBarValAndColor(barFrontRight, frontright.get_temperature(), bar4Val);
    setBarValAndColor(barMidRight, midright.get_temperature(), bar5Val);
    setBarValAndColor(barBackRight, backright.get_temperature(), bar6Val);
    setBarValAndColor(barLift, lift.get_temperature(), bar7Val);
    setBarValAndColor(barConveyor, conveyor.get_temperature(), bar8Val);
}

void updateBarGraph_fn(void* param){
    uint32_t startTime = pros::millis();
    pros::Task::delay_until(&startTime, 1000);
    while(true){
        setBarValAndColor(barFrontLeft, frontleft.get_temperature(), bar1Val);
        setBarValAndColor(barMidLeft, midleft.get_temperature(), bar2Val);
        setBarValAndColor(barBackLeft, backleft.get_temperature(), bar3Val);
        setBarValAndColor(barFrontRight, frontright.get_temperature(), bar4Val);
        setBarValAndColor(barMidRight, midright.get_temperature(), bar5Val);
        setBarValAndColor(barBackRight, backright.get_temperature(), bar6Val);
        setBarValAndColor(barLift, lift.get_temperature(), bar7Val);
        setBarValAndColor(barConveyor, conveyor.get_temperature(), bar8Val);
        pros::Task::delay_until(&startTime, 1000);  
    }
}