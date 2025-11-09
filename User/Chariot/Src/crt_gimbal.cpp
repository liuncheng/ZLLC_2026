/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint32_t Count_test = 0;
/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega_Angle / Omega_Max * Output_Max);
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        //力矩环
        PID_Torque.Set_Target(Target_Torque);
        PID_Torque.Set_Now(Data.Now_Torque);
        PID_Torque.TIM_Adjust_PeriodElapsedCallback();

        Set_Out(PID_Torque.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        //角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status()==IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Yaw*180.f/PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
    }
    break;
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);
        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {
            //角度环
            PID_Angle.Set_Now(True_Angle_Yaw);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();
            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Yaw*180.f/PI);

        }
        else
        {
            Target_Omega_Angle = 0.0f;
            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
        
        Set_Out(-PID_Omega.Get_Out());//由于电机的输出值
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Yaw_Encoder_Angle.Set_Target(Target_Angle);
        PID_Yaw_Encoder_Angle.Set_Now(Get_True_Angle_Yaw_From_Encoder());
        PID_Yaw_Encoder_Angle.TIM_Adjust_PeriodElapsedCallback();
        Target_Omega_Angle = PID_Yaw_Encoder_Angle.Get_Out();

        PID_Yaw_Encoder_Omega.Set_Target(Target_Omega_Angle);
        PID_Yaw_Encoder_Omega.Set_Now(-Data.Now_Omega_Angle);
        PID_Yaw_Encoder_Omega.TIM_Adjust_PeriodElapsedCallback();
        Set_Out(-PID_Yaw_Encoder_Omega.Get_Out());
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Yaw_Motor_GM6020::Transform_Angle()
{
    True_Rad_Yaw = IMU->Get_Rad_Yaw();
    True_Gyro_Yaw = IMU->Get_Gyro_Yaw(); 
    True_Angle_Yaw = IMU->Get_Angle_Yaw();
}
void Class_Gimbal_Yaw_Motor_GM6020::Transform_EmcoderAngle_To_TrueAngle()
{
    const float Reference_Angle = 1.07800508f;
    float GM6020_Angle_Rad = ((float)Get_Now_Total_Encoder()) / 8191 * 2 * PI / 2;
    float Yaw_Angle_Rad = fabsf(fmodf(GM6020_Angle_Rad, 2.0f * PI));
    if(Get_Now_Total_Round() < 0)
        Yaw_Angle_Rad = 2.0f * PI - Yaw_Angle_Rad;
    if(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= 2 * PI;//得到角度范围为[-PI,PI]
    
    Yaw_Angle_Rad -=  Reference_Angle;

    while(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= PI * 2.0f;
    while(Yaw_Angle_Rad < -PI)
        Yaw_Angle_Rad += PI * 2.0f;
    
    EmcoderAngle_To_TrueAngle = -Yaw_Angle_Rad / PI * 180.0f;
}
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal_Pitch_Motor_M2006::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        //默认开环速度控制
        Set_Out(Target_Omega_Angle);
    }
    break;
    case (DJI_Motor_Control_Method_IMU_OMEGA):
    {
        //角速度环
        PID_Omega.Set_Target(Target_Omega_Angle);
        if (IMU->Get_IMU_Status()==IMU_Status_DISABLE)
        {
            PID_Omega.Set_Now(Data.Now_Omega_Angle);
        }
        else
        {
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);
        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out());
				
    }
    break;
		
    case (DJI_Motor_Control_Method_IMU_ANGLE):
    {
        PID_Angle.Set_Target(Target_Angle);

        if (IMU->Get_IMU_Status()!=IMU_Status_DISABLE)
        {

            //角度环
            PID_Angle.Set_Now(True_Angle_Pitch);
            PID_Angle.TIM_Adjust_PeriodElapsedCallback();

            Target_Omega_Angle = PID_Angle.Get_Out();

            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(True_Gyro_Pitch*180.f/PI);

        }
        else
        {
            Target_Omega_Angle = 0.0f;
            
            //速度环
            PID_Omega.Set_Target(Target_Omega_Angle);
            PID_Omega.Set_Now(Data.Now_Omega_Angle);

        }
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();
    
        Target_Torque = PID_Omega.Get_Out();
        Set_Out(PID_Omega.Get_Out() + Gravity_Compensate);
    }
    break;
    default:
    {
        Set_Out(0.0f);
    }
    break;
    }
    Output();
}

/**
 * @brief 根据不同c板的放置方式来修改这个函数
 *
 */
void Class_Gimbal_Pitch_Motor_M2006::Transform_Angle()
{
    True_Rad_Pitch = 1 * IMU->Get_Rad_Roll();
    True_Gyro_Pitch = 1 * IMU->Get_Gyro_Roll(); 
    True_Angle_Pitch = 1 * IMU->Get_Angle_Roll();  
}


/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    //吊射参数
    // Motor_Yaw.PID_Angle.Init(80.0f, 0.016f, 0.04f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.001f);
    // Motor_Yaw.PID_Omega.Init(150.0f, 0.15f, 0.0075f, 0.0f, 2000.0f, 20000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    //较好的随动参数
    Motor_Yaw.PID_Angle.Init(70.0f, 0.016f, 1.5f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Yaw.PID_Omega.Init(80.0f, 0.15f, 0.01f, 0.0f, 2000.0f, 20000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    //编码器PID初始化
    Motor_Yaw.PID_Yaw_Encoder_Angle.Init(80.0f, 0.1f, 0.3f, 0.0f, 0.0f, 0.0f,0.0f, 0.0f, 0.0f, 0.001f,0.005f);
    Motor_Yaw.PID_Yaw_Encoder_Omega.Init(85.0f, 0.5f, 0.0f, 0.0f, 7000.0f, 20000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Yaw.IMU = &Boardc_BMI;
    Motor_Yaw.Init(&hfdcan1, DJI_Motor_ID_0x205,  DJI_Motor_Control_Method_IMU_ANGLE, 2);
    //test_DM4310
    Motor_Yaw_test.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION);
    CAN_Send_Data(&hfdcan1, DM_Motor_ID_0xA1+0xf0, DM_Motor_CAN_Message_Save_Zero, 8);

    //Motor_Yaw.PID_Angle.Init(30.f, 0.0f, 0.0f, 0.0f, 500, 500);
    //Motor_Yaw.PID_Omega.Init(60.0f, 15.0f, 0.0f, 0.0f, 6000, Motor_Yaw.Get_Output_Max(), 10.f, 50.f);
    //Motor_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    //Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);
    
    // pitch轴电机
    //Motor_Pitch.PID_Angle.Init(22.f, 0.0f, 0.001f, 0.0f, 2.f, 650.f);
    //Motor_Pitch.PID_Omega.Init(90.0f, 20.0f, 0.0f, 0.0f, 6000, Motor_Pitch.Get_Output_Max(),0.f,0.f,40.f);
    //Motor_Pitch.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    //Motor_Pitch.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);
    Motor_Pitch.PID_Angle.Init(50.0f, 1.5f, 2.0f, 0.0f, 0.0f, 150.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Pitch.PID_Omega.Init(150.0f, 0.5f, 0.0f, 0.0f, 3000.0f, 10000.0f,0.0f, 0.0f, 0.0f, 0.001f);
    Motor_Pitch.IMU = &Boardc_BMI;
    Motor_Pitch.Init(&hfdcan1, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_IMU_ANGLE);
}


/**
 * @brief 输出到电机
 *
 */
float Tmp_Target_Yaw_Angle = 0.0f,Tmp_Ture_Yaw_Angle = 0.0f;
void Class_Gimbal::Output()
{
    Gimbal_Control_Type == Gimbal_Control_Type_NORMAL;
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        /*// 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);


        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);

        Motor_Yaw.Set_Out(0.0f);
        Motor_Pitch.Set_Out(0.0f);*/
        //云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.Set_Target_Omega_Angle(0.0f);
        Motor_Pitch.Set_Target_Omega_Angle(0.0f);
        
        Motor_Yaw_test.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        
        Tmp_Ture_Yaw_Angle = Motor_Yaw_test.Get_Now_Angle();
        while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)>Max_Yaw_Angle_Radian)
        {
        Tmp_Target_Yaw_Angle -= (2 * Max_Yaw_Angle_Radian);
        }
        while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)<Min_Yaw_Angle_Radian)
        {
        Tmp_Target_Yaw_Angle += (2 * Max_Yaw_Angle_Radian);
        }
        //Motor_Yaw_test.Set_Target_Angle((Count_test / 1000) % 2 == 0 ? 0.0f : (PI/2));
        


    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw_test.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            //Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);
            //处理yaw轴180度问题
            Tmp_Target_Yaw_Angle = Target_Yaw_Angle * PI / 180.0f;
            Tmp_Ture_Yaw_Angle = Motor_Yaw_test.Get_Now_Angle();
            while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)>Max_Yaw_Angle_Radian)
            {
            Tmp_Target_Yaw_Angle -= (2 * Max_Yaw_Angle_Radian);
            }
            while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)<Min_Yaw_Angle_Radian)
            {
            Tmp_Target_Yaw_Angle += (2 * Max_Yaw_Angle_Radian);
            }

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            Motor_Yaw_test.Set_Target_Angle(Tmp_Target_Yaw_Angle);
            Motor_Yaw_test.Set_Target_Omega(0.0f);
            Motor_Yaw_test.Set_Target_Torque(0.0f);
            Motor_Yaw_test.Set_MIT_K_P(6.0f);
            Motor_Yaw_test.Set_MIT_K_D(0.2f);

        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Motor_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());
            Motor_Pitch.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle());
           
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {

            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

        }
       /*
       //pitch yaw轴控制方式
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
        switch (Get_Launch_Mode())//吊射模式 拨杆左上 不影响自瞄
        {
        case Launch_Disable:
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_IMU_ANGLE);
            Tmp_Target_Yaw_Angle = Target_Yaw_Angle;
            Tmp_Ture_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw();//IMU获取的真实角度
        }
        break;
        case Launch_Enable:
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Tmp_Target_Yaw_Angle = Target_Yaw_Encoder_Angle;
            Tmp_Ture_Yaw_Angle = Motor_Yaw.Get_True_Angle_Yaw_From_Encoder();//编码器获取的真实角度
        }
        break;
        }

        //限制角度范围 处理yaw轴180度问题
        while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)>Max_Yaw_Angle)
        {
            Tmp_Target_Yaw_Angle -= (2 * Max_Yaw_Angle);
        }
        while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)<-Max_Yaw_Angle)
        {
            Tmp_Target_Yaw_Angle += (2 * Max_Yaw_Angle);
        }
        //pitch限位
        Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
        //设置yaw轴与pitch轴目标角度
        Motor_Yaw.Set_Target_Angle(Tmp_Target_Yaw_Angle);
        Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
        */
    }
    
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    Count_test ++;
    //控制模式
    Output();
    //根据不同c板的放置方式来修改这几个函数
    Motor_Yaw.Transform_Angle();
    Motor_Yaw.Transform_EmcoderAngle_To_TrueAngle();
    Motor_Pitch.Transform_Angle();
    

    //滑模控制
    // static uint8_t mod10 = 0;
    // if(mod10 == 2){
    //     //注意电机正转角度应该增大，IMU坐标系应该和该坐标系一致，不然会负反馈
    //     Motor_Yaw.Set_Transform_Angle(-Boardc_BMI.Get_Angle_Yaw());
    //     Motor_Yaw.Set_Transform_Omega(-Boardc_BMI.Get_Gyro_Yaw() * 57.3f);          //陀螺仪这里的角度得是度每秒
    //     Motor_Yaw.TIM_SMC_PeriodElapsedCallback();
    //     mod10 = 0;
    // }
    // mod10 ++;
    
    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
    Motor_Yaw_test.TIM_Process_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
