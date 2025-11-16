#include "Process_Control.h"
//点位
const position start={0,-400,0};
const position start_reverse={0,-400,PI};
const position put_start={0,-1600,PI};//放置侧的正中间
const position put_middle={0,-1250,PI};//4,9正中间
const position put_middle_p9={0,-1250,1.5f*PI};//旋转放9号点
const position put_middle_p4={0,-1250,0.5f*PI};//旋转放4号点




const position p1_bak={500,1400,0};
const position p1_cal={500,1700,0};
const position p1_p2_mid={1058,-300,0};

const position p1_cal_ture={500,2100,0};



const position p2_bak={0,1400,0};
const position p2_cal={0,1700,0};
const position p2_p3_mid={-1058,-180,0};

const position p3_bak={-500,1400,0};
const position p3_cal={-500,1700,0};

const position p4_cal={-1250,579,0.5f*PI};//f
const position p5_cal={-1586,684,PI};//e
const position p6_cal={-1582,222,PI};//d
const position p7_cal={-1583,-234,PI};//c
const position p8_cal={-1575,-703,PI};//b
const position p9_cal={-1245,-578,1.5f*PI};//a

const position p_b={-400,0,1.5f*PI};//B区

//高度
const uint16_t shelf_in_1=160;//一层货架进入高度
const uint16_t shelf_catch_1=55;//一层货架夹取高度
const uint16_t shelf_in_2=200;//二层货架进入高度
const uint16_t shelf_catch_2=350;//二层货架夹取高度
const uint16_t stack_1=100;//纸垛一层高度
const uint16_t stack_2=250;//纸垛二层高度
const uint16_t plate_get1=150;//货盘取货
const uint16_t plate_get2=270;//货盘取货
const uint16_t wait_plate=200;//等待货盘转动高度（二层）
const uint16_t plate_put_1=200;//货盘放置(一层)
const uint16_t plate_put_2=300;//货盘放置(二层)

const uint16_t run_time = 7000;//中长距离跑时间（例如起点到123号点后部）
const uint16_t short_run_time = 2000;//短距离跑时间（例如123号点后部到123号点）
const uint16_t calibrate_time = 5000;//识别校准时间
const uint16_t plate_time = 4000;//转盘时间
const uint16_t lift_time = 2000;//升降时间
const uint16_t catch_time = 1000;//抓取时间
const uint16_t reverse_time = 5000;//旋转时间


control_block control_flow[200];
//总控制流程数量
uint8_t control_flow_size=0;
//转盘放置情况记录
uint8_t  cur_plate_occupy[6]={0,0,0,0,0,0};

/**
 * 流程生成
 * @param sequence 顺序
 */
void Process_Generation(const uint8_t *sequence)
{
    for (int i=0;i<6;i++)
        Get_process(sequence[i],i);
    Put_process(sequence[6],sequence[7]);
}
/**
 * @brief 获取可用转盘盘号（取
 * @param layer 层数
 * @return
 */
uint8_t get_plate_num(uint8_t layer)
{
    uint8_t num=0,occupy=0;//盘号及占用位置
    if(layer==1)
    {
        if(cur_plate_occupy[0]==0)//1号没放
        {
            num = 1;
            occupy=0;
        }
        else if(cur_plate_occupy[2]==0)//3号没放
        {
            num = 3;
            occupy=2;
        }
        else
        {
            num = 2;
            occupy=1;
        }
    }
    else
    {
        if(cur_plate_occupy[4]==0)//2号上层没放
        {
            num = 2;
            occupy=4;
        }
        else if(cur_plate_occupy[3]==0)//1号上层没放
        {
            num = 1;
            occupy=3;
        }
        else
        {
            num = 3;
            occupy=5;
        }
    }
    cur_plate_occupy[occupy]=1;
    return num;
}
/**
 * @brief 获取可用转盘盘号（放
 * @param layer 层数
 * @return
 */
uint8_t put_plate_num(uint8_t layer)
{
    uint8_t num=0,non_occupy=0;//盘号及占用位置
    if(layer==2)
    {
        if(cur_plate_occupy[3]==1)//1号上层有货
        {
            num = 1;
            non_occupy=3;
        }
        else if(cur_plate_occupy[5]==1)//3号上层没放
        {
            num = 3;
            non_occupy=5;
        }
        else
        {
            num = 2;
            non_occupy=4;
        }
    }
    else
    {
        if(cur_plate_occupy[0]==1)//1号没放
        {
            num = 1;
            non_occupy=0;
        }
        else if(cur_plate_occupy[2]==1)//3号没放
        {
            num = 3;
            non_occupy=2;
        }
        else
        {
            num = 2;
            non_occupy=1;
        }
    }
    cur_plate_occupy[non_occupy]=0;
    return num;
}
/**
 * @brief
 * @param not_put 几号不放
 * @param number 放几号二层
 */
void Put_process(uint8_t not_put,uint8_t number)
{


    //复位到起点
    control_block action={
            action.pos_target = start,
            action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
            action.steering_engine = 0,
            action.uart = 0,
            action.delay=run_time
    };
    control_flow[control_flow_size++]=action;

    //旋转
    action.pos_target = start_reverse;
    action.delay = reverse_time;
    control_flow[control_flow_size++]=action;
    //起点
    action.pos_target = put_start;
    action.delay = run_time;
    control_flow[control_flow_size++]=action;


    //5号点二层
    if(not_put!=5) {
        //5号点
        action.pos_target = p5_cal;
        action.delay = run_time;
        control_flow[control_flow_size++] = action;
        //5号校准
        action.delay = calibrate_time;
        action.uart = 5;
        control_flow[control_flow_size++] = action;
        //升爪
        action.arm_target.height = plate_get2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 1;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //夹取二层
        action.arm_target.height = shelf_catch_2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //夹取
        action.steering_engine = 1,
                action.delay = catch_time;
        control_flow[control_flow_size++] = action;
        //升起
        action.arm_target.height = wait_plate;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 0;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //放置
        action.arm_target.height = stack_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
    }


    //6号点二层
    if(not_put!=6) {
        //6号点
        action.pos_target = p6_cal;
        action.delay = run_time;
        control_flow[control_flow_size++] = action;
        //6号校准
        action.delay = calibrate_time;
        action.uart = 6;
        control_flow[control_flow_size++] = action;
        //升爪
        action.arm_target.height = plate_get2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 3;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //夹取二层
        action.arm_target.height = shelf_catch_2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //夹取
        action.steering_engine = 1,
                action.delay = catch_time;
        control_flow[control_flow_size++] = action;
        //升起
        action.arm_target.height = wait_plate;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 0;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //放置
        action.arm_target.height = stack_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
    }

    //7号点二层
    if(not_put!=7) {
        //7号点
        action.pos_target = p7_cal;
        action.delay = run_time;
        control_flow[control_flow_size++] = action;
        //7号校准
        action.delay = calibrate_time;
        action.uart = 7;
        control_flow[control_flow_size++] = action;
        //升爪
        action.arm_target.height = plate_get2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 2;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //夹取二层
        action.arm_target.height = shelf_catch_2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //夹取
        action.steering_engine = 1,
                action.delay = catch_time;
        control_flow[control_flow_size++] = action;
        //升起
        action.arm_target.height = wait_plate;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 0;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //放置
        action.arm_target.height = stack_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
    }


    if(not_put!=8) {
        //8号点
        action.pos_target = p8_cal;
        action.delay = run_time;
        control_flow[control_flow_size++] = action;
        //8号校准
        action.delay = calibrate_time;
        action.uart = 8;
        control_flow[control_flow_size++] = action;
        //升爪
        action.arm_target.height = plate_get2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 2;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //夹取一层
        action.arm_target.height = shelf_catch_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //夹取
        action.steering_engine = 1,
                action.delay = catch_time;
        control_flow[control_flow_size++] = action;
        //升起
        action.arm_target.height = wait_plate;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 0;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //放置
        action.arm_target.height = stack_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
    }

    //复位点
    action.pos_target = put_start;
    action.delay = run_time;
    control_flow[control_flow_size++] = action;
    //复位点
    action.pos_target = put_middle;
    action.delay = run_time;
    control_flow[control_flow_size++] = action;

    //9号点放置
    if(not_put!=9)
    {
        //旋转到9号
        action.pos_target = put_middle_p9;
        action.delay = run_time;
        control_flow[control_flow_size++] = action;
        //9号点
        action.pos_target = p9_cal;
        action.delay = run_time;
        control_flow[control_flow_size++] = action;
        //9号校准
        action.delay = calibrate_time;
        action.uart = 9;
        control_flow[control_flow_size++] = action;
        //升爪
        action.arm_target.height = plate_get2;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 2;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //夹取一层
        action.arm_target.height = shelf_catch_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //夹取
        action.steering_engine = 1,
                action.delay = catch_time;
        control_flow[control_flow_size++] = action;
        //升起
        action.arm_target.height = wait_plate;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;
        //转盘
        action.arm_target.state = 0;
        action.delay = plate_time;
        control_flow[control_flow_size++] = action;
        //放置
        action.arm_target.height = stack_1;
        action.delay = lift_time;
        control_flow[control_flow_size++] = action;

        //放置9号二层
        if(number == 9 )
        {
            //9号后
            action.pos_target = put_middle_p9;
            action.delay = run_time;
            control_flow[control_flow_size++] = action;
            //9号点
            action.pos_target = p9_cal;
            action.delay = run_time;
            control_flow[control_flow_size++] = action;
            //9号校准
            action.delay = calibrate_time;
            action.uart = 9;
            control_flow[control_flow_size++] = action;
            //升爪
            action.arm_target.height = plate_get2;
            action.delay = lift_time;
            control_flow[control_flow_size++] = action;
            //转盘
            action.arm_target.state = 2;
            action.delay = plate_time;
            control_flow[control_flow_size++] = action;
            //夹取一层
            action.arm_target.height = shelf_catch_1;
            action.delay = lift_time;
            control_flow[control_flow_size++] = action;
            //夹取
            action.steering_engine = 1,
                    action.delay = catch_time;
            control_flow[control_flow_size++] = action;
            //升起
            action.arm_target.height = wait_plate;
            action.delay = lift_time;
            control_flow[control_flow_size++] = action;
            //转盘
            action.arm_target.state = 0;
            action.delay = plate_time;
            control_flow[control_flow_size++] = action;
            //放置
            action.arm_target.height = stack_1;
            action.delay = lift_time;
            control_flow[control_flow_size++] = action;




        }
    }

    action =(control_block) {
            action.pos_target = start,
            action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
            action.steering_engine = 0,
            action.uart = 0,
            action.delay=run_time
    };
    control_flow[control_flow_size++]=action;




    action.pos_target = p5_cal;
    control_flow[control_flow_size++]=action;
    action.arm_target.state=put_plate_num(2);
}


/**
 * 取单个货箱流程
 * @param number 当前要取的货箱号
 * @param i 当前是第几个货箱
 */
void Get_process(uint8_t number,uint8_t i)
{
    uint16_t j=0;
    control_block action;
    switch (number)
    {
        case 1:
        case 4:
        {
            action=(control_block){
                    action.pos_target = start,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 0,
                    action.delay=30000
            };
            control_flow[control_flow_size++]=action;
            action=(control_block){
                    action.pos_target = start,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 0,
                    action.delay=30000
            };
            control_flow[control_flow_size++]=action;
            //1点位后方
            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 0,
                    action.delay=run_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

//                //1号点
//                action.pos_target=p1_cal;
//                action.delay=short_run_time;
//                control_flow[control_flow_size++]=action;

            //1号点校准
            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 2,
                    action.delay=calibrate_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;


            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 0,
                    action.delay=10000
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

            //下降
            action =(control_block) {
                    action.pos_target = p1_cal,
                    action.arm_target = (arm_control){.height = shelf_catch_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 1,
                    action.delay=lift_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_catch_1;
            control_flow[control_flow_size++]=action;


            //夹取
            action =(control_block) {
                    action.pos_target = p1_cal,
                    action.arm_target = (arm_control){.height = shelf_catch_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=catch_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_catch_1;
            control_flow[control_flow_size++]=action;
            //上升
            action =(control_block) {
                    action.pos_target = p1_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=lift_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;
            //退出
            action =(control_block) {
                    action.pos_target = p1_bak,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=short_run_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

            //升高
            action =(control_block) {
                    action.pos_target = p1_bak,
                    action.arm_target = (arm_control){.height = wait_plate, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=lift_time
            };

            //转转盘
            action.arm_target.state= get_plate_num((i<=3)?1:2);
            action.delay=plate_time;
            control_flow[control_flow_size++]=action;

            //放下
            action.arm_target.height=plate_put_2;
            if(i<=3)
                action.arm_target.height=plate_put_1;
            action.delay=lift_time;
            control_flow[control_flow_size++]=action;
            //松开夹子
            action.steering_engine=0;
            action.delay=catch_time;
            control_flow[control_flow_size++]=action;
            break;
        }
        case 2:
        case 5:
        {
            //2点位后方
            action =(control_block) {
                    action.pos_target = p2_bak,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 0,
                    action.delay=run_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

            //2号点
            action.pos_target=p2_cal;
            action.delay=short_run_time;
            control_flow[control_flow_size++]=action;

            //2号点校准
            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 2,
                    action.delay=calibrate_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;
            //下降
            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_catch_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 2,
                    action.delay=lift_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_catch_1;
            control_flow[control_flow_size++]=action;
            //夹取
            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_catch_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 2,
                    action.delay=catch_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_catch_1;
            control_flow[control_flow_size++]=action;
            //上升
            action =(control_block) {
                    action.pos_target = p2_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 2,
                    action.delay=lift_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;
            //退出
            action =(control_block) {
                    action.pos_target = p2_bak,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=short_run_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

            //升高
            action =(control_block) {
                    action.pos_target = p2_bak,
                    action.arm_target = (arm_control){.height = wait_plate, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=lift_time
            };

            //转转盘
            action.arm_target.state= get_plate_num((i<=3)?3:1);
            action.delay=plate_time;
            control_flow[control_flow_size++]=action;

            //放下
            action.arm_target.height=plate_put_2;
            if(i<=3)
                action.arm_target.height=plate_put_1;
            action.delay=lift_time;
            control_flow[control_flow_size++]=action;
            //松开夹子
            action.steering_engine=0;
            action.delay=catch_time;
            control_flow[control_flow_size++]=action;
            break;
        }
        case 3:
        case 6:
        {
            //3点位后方
            action =(control_block) {
                    action.pos_target = p3_bak,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 0,
                    action.delay=run_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

            //3号点
            action.pos_target=p3_cal;
            action.delay=short_run_time;
            control_flow[control_flow_size++]=action;

            //3号点校准
            action =(control_block) {
                    action.pos_target = p3_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 3,
                    action.delay=calibrate_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;
            //下降
            action =(control_block) {
                    action.pos_target = p3_cal,
                    action.arm_target = (arm_control){.height = shelf_catch_2, .state = 0},
                    action.steering_engine = 0,
                    action.uart = 3,
                    action.delay=lift_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_catch_1;
            control_flow[control_flow_size++]=action;
            //夹取
            action =(control_block) {
                    action.pos_target = p3_cal,
                    action.arm_target = (arm_control){.height = shelf_catch_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 3,
                    action.delay=catch_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_catch_1;
            control_flow[control_flow_size++]=action;
            //上升
            action =(control_block) {
                    action.pos_target = p3_cal,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 3,
                    action.delay=lift_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;
            //退出
            action =(control_block) {
                    action.pos_target = p3_bak,
                    action.arm_target = (arm_control){.height = shelf_in_2, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=short_run_time
            };
            if(i<=3)//下方的货物
                action.arm_target.height=shelf_in_1;
            control_flow[control_flow_size++]=action;

            //升高
            action =(control_block) {
                    action.pos_target = p3_bak,
                    action.arm_target = (arm_control){.height = wait_plate, .state = 0},
                    action.steering_engine = 1,
                    action.uart = 0,
                    action.delay=lift_time
            };

            //转转盘
            action.arm_target.state= get_plate_num((i<=3)?2:3);
            action.delay=plate_time;
            control_flow[control_flow_size++]=action;

            //放下
            action.arm_target.height=plate_put_2;
            if(i<=3)
                action.arm_target.height=plate_put_1;
            action.delay=lift_time;
            control_flow[control_flow_size++]=action;
            //松开夹子
            action.steering_engine=0;
            action.delay=catch_time;
            control_flow[control_flow_size++]=action;
            break;
        }
        default: break;
    }
}
