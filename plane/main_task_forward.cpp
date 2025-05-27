#include <ros/ros.h>
#include <fly_pkg/pos_srv.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <fly_pkg/cv_pos_data.h>

#define PI 3.1415926f
#define ON 1 //飞行模式
#define Land -1 //降落
#define NO 0
#define Ready 2  //进入待机模式
#define EEROR 3  //位置失灵
#define fly_z 0.5

bool flag = false;

//位置信息订阅
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
}
//状态订阅回调函数
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}
//接受视觉 A 1 code 指令 回调函数
fly_pkg::cv_pos_data cv_pos_flag;
void cv_position_control_cb(const fly_pkg::cv_pos_data::ConstPtr& msg){
    if(fabs(local_pos.pose.position.x-(3.14)) < 0.1 &&
    fabs(local_pos.pose.position.y-(-3.15)) < 0.1 &&
    fabs(local_pos.pose.position.z-(1.75)) < 0.1 
    ){
        cv_pos_flag = *msg;
    }
}

//飞行控制  pos_x 为x轴位置 yaw为偏航角 control_flag 为飞行控制指令
void fly_control(fly_pkg::pos_srv::Request& aim_pos,float pos_x,float pos_y,float pos_z,float yaw,short control_flag,
                    float angle_accuracy,float pos_accuracy,float cv_angle_accuracy,float cv_pos_accuracy,float cv_times){
    
    aim_pos.control_flag = control_flag;
    aim_pos.x   = pos_x;
    aim_pos.y   = pos_y;
    aim_pos.z   = pos_z;
    aim_pos.yaw = yaw;
    
    aim_pos.angle_accuracy = angle_accuracy;
    aim_pos.pos_accuracy = pos_accuracy;

    aim_pos.cv_angle_accuracy = cv_angle_accuracy;
    aim_pos.cv_pos_accuracy = cv_pos_accuracy;
    aim_pos.cv_times = cv_times;
}

//时间的单位是s 功能为到达目的地，并且保持一定的时间  times 为保持时间 hz为在此期间订阅话题的hz
void fly_control_with_time(fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client,float pos_x,float pos_y,float pos_z,
                            float yaw,short control_flag,float times,float hz,
                            float angle_accuracy,float pos_accuracy,float cv_angle_accuracy,float cv_pos_accuracy,float cv_times){
    ros::Time last_times = ros::Time::now();
    ros::Rate pos_rate(hz);
    fly_control(aim_pos.request, pos_x , pos_y , pos_z , yaw , control_flag,angle_accuracy,pos_accuracy,cv_angle_accuracy,cv_pos_accuracy,cv_times); 
    while(ros::ok()){         
        while(ros::ok()){
            if(pos_client.call(aim_pos)) //到达后即返回True
                break;
        }
        if(control_flag==Land){  //降落模式
            if(!current_state.armed){ //等待模式切换完毕
                break;
            }
        }
        //处理持续时间
        else{
            if(aim_pos.response.arrive_flag){
                if(ros::Time::now() -last_times> ros::Duration(times)){
                    // 已到达攻击区域 准备射击
                    if(fabs(pos_x - (111)) < 0.1 && fabs(pos_y - (111)) < 0.1 && flag == false){
                        system("echo 1 > /sys/class/gpio/PN.01/value");
                        system("echo 1 > /sys/class/gpio/PCC.04/value");
                        flag = true;
                    }
                }
            else{
                last_times = ros::Time::now();
            }
            }
        }
        ros::spinOnce();
        pos_rate.sleep();
    }
}

//飞机进入待机模式 指令
void plane_to_ready(fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client){
    ros::Rate pos_rate(100);
    aim_pos.request.control_flag = Ready;
    while(ros::ok()){
        pos_client.call(aim_pos);
        if(current_state.mode == "OFFBOARD"&&current_state.armed){ //等待模式切换完毕
            break;
        }
        ros::spinOnce();
        pos_rate.sleep();
    }
}

void PID_revise(float x_kp,float x_ki,float x_kd,float x_outputMax,float x_outputMin,
                float y_kp,float y_ki,float y_kd,float y_outputMax,float y_outputMin,
                float z_kp,float z_ki,float z_kd,float z_outputMax,float z_outputMin,
                float yaw_kp,float yaw_ki,float yaw_kd,float yaw_outputMax,float yaw_outputMin,
                float cv_x_kp,float cv_x_ki,float cv_x_kd,float cv_y_kp,float cv_y_ki,float cv_y_kd,
                float cv_z_kp,float cv_z_ki,float cv_z_kd,float cv_yaw_kp,float cv_yaw_ki,float cv_yaw_kd,
                fly_pkg::pos_srv::Request& aim_pos){
    aim_pos.x_kp = x_kp;
    aim_pos.x_ki = x_ki;
    aim_pos.x_kd = x_kd;        
    aim_pos.x_outputMax = x_outputMax;
    aim_pos.x_outputMin = x_outputMin;

    aim_pos.y_kp = y_kp;
    aim_pos.y_ki = y_ki;
    aim_pos.y_kd = y_kd;        
    aim_pos.y_outputMax = y_outputMax;
    aim_pos.y_outputMin = y_outputMin;

    aim_pos.z_kp = z_kp;
    aim_pos.z_ki = z_ki;
    aim_pos.z_kd = z_kd;        
    aim_pos.z_outputMax = z_outputMax;
    aim_pos.z_outputMin = z_outputMin;

    aim_pos.yaw_kp = yaw_kp;
    aim_pos.yaw_ki = yaw_ki;
    aim_pos.yaw_kd = yaw_kd;        
    aim_pos.yaw_outputMax = yaw_outputMax;
    aim_pos.yaw_outputMin = yaw_outputMin;

    aim_pos.cv_x_kp = cv_x_kp;
    aim_pos.cv_x_ki = cv_x_ki;
    aim_pos.cv_x_kd = cv_x_kd;

    aim_pos.cv_y_kp = cv_y_kp;
    aim_pos.cv_y_ki = cv_y_ki;
    aim_pos.cv_y_kd = cv_y_kd;

    aim_pos.cv_z_kp = cv_z_kp;
    aim_pos.cv_z_ki = cv_z_ki;
    aim_pos.cv_z_kd = cv_z_kd;

    aim_pos.cv_yaw_kp = cv_yaw_kp;
    aim_pos.cv_yaw_ki = cv_yaw_ki;
    aim_pos.cv_yaw_kd = cv_yaw_kd;
}
void wait_time(float times,float hz,fly_pkg::pos_srv& aim_pos,ros::ServiceClient &pos_client){
    ros::Time last_times = ros::Time::now();
    while(ros::ok()){
        if(ros::Time::now() -last_times> ros::Duration(times)){
            break;
        }
        pos_client.call(aim_pos);
        ros::spinOnce();
        ros::Rate(hz).sleep();
    }
}


int main (int argc,char** argv){
    
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"main_task");
    ros::NodeHandle nh;
    //客户端
    ros::ServiceClient pos_client = nh.serviceClient<fly_pkg::pos_srv>("pos_control");
    //订阅
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>  //接收无人机位置信息
		("mavros/local_position/pose", 10, local_pos_cb);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>			//接收无人机模式状态
		("mavros/state", 10, state_cb);
    ros::Subscriber cv_position_control = nh.subscribe<fly_pkg::cv_pos_data>  //接受视觉控制指令
        ("cv_task",10,cv_position_control_cb);

    fly_pkg::pos_srv aim_pos;

    cv_pos_flag.cv_position_flag = 0;

    ros::Rate pos_rate(100); //100hz

    //无用
    aim_pos.request.roll =  0;
    aim_pos.request.pitch = 0;
    
    float x_kp,x_ki,x_kd,x_speed_max,x_speed_min;
    float y_kp,y_ki,y_kd,y_speed_max,y_speed_min;
    float z_kp,z_ki,z_kd,z_speed_max,z_speed_min;
    float yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min;
    float cv_x_kp,cv_x_ki,cv_x_kd;
    float cv_y_kp,cv_y_ki,cv_y_kd;
    float cv_z_kp,cv_z_ki,cv_z_kd;
    float cv_yaw_kp,cv_yaw_ki,cv_yaw_kd;
    
    //PID初始参数读取
    nh.getParam("x_kp", x_kp);nh.getParam("x_ki", x_ki);nh.getParam("x_kd", x_kd);nh.getParam("x_speed_max", x_speed_max);nh.getParam("x_speed_min", x_speed_min);
    nh.getParam("y_kp", y_kp);nh.getParam("y_ki", y_ki);nh.getParam("y_kd", y_kd);nh.getParam("y_speed_max", y_speed_max);nh.getParam("y_speed_min", y_speed_min);
    nh.getParam("z_kp", z_kp);nh.getParam("z_ki", z_ki);nh.getParam("z_kd", z_kd);nh.getParam("z_speed_max", z_speed_max);nh.getParam("z_speed_min", z_speed_min);
    nh.getParam("yaw_kp", yaw_kp);nh.getParam("yaw_ki", yaw_ki);nh.getParam("yaw_kd", yaw_kd);nh.getParam("yaw_speed_max", yaw_speed_max);nh.getParam("yaw_speed_min", yaw_speed_min);
    
    nh.getParam("cv_x_kp", cv_x_kp);nh.getParam("cv_x_ki", cv_x_ki);nh.getParam("cv_x_kd", cv_x_kd);
    nh.getParam("cv_y_kp", cv_y_kp);nh.getParam("cv_y_ki", cv_y_ki);nh.getParam("cv_y_kd", cv_y_kd);
    nh.getParam("cv_z_kp", cv_z_kp);nh.getParam("cv_z_ki", cv_z_ki);nh.getParam("cv_z_kd", cv_z_kd);
    nh.getParam("cv_yaw_kp", cv_yaw_kp);nh.getParam("cv_yaw_ki", cv_yaw_ki);nh.getParam("cv_yaw_kd", cv_yaw_kd);

    ROS_INFO("--------------------------------PID-------------------------------------------");
    ROS_INFO("x_kp = %f   x_ki=%f  x_kd=%f x_speed_max=%f x_speed_min=%f ",x_kp,x_ki,x_kd,x_speed_max,x_speed_min);
    ROS_INFO("y_kp = %f   y_ki=%f  y_kd=%f y_speed_max=%f y_speed_min=%f ",y_kp,y_ki,y_kd,y_speed_max,y_speed_min);
    ROS_INFO("z_kp = %f   z_ki=%f  z_kd=%f z_speed_max=%f z_speed_min=%f ",z_kp,z_ki,z_kd,z_speed_max,z_speed_min);
    ROS_INFO("yaw_kp = %f   yaw_ki=%f  yaw_kd=%f yaw_speed_max=%f yaw_speed_min=%f ",yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min);

    ROS_INFO("--------------------------------PID-------------------------------------------");
    ros::service::waitForService("pos_control");  //等待位置服务器开启
    /*-----------进入待机模式-----------------------------------------------------------------------------*/  
    plane_to_ready(aim_pos,pos_client);
    /*-----------PID参数修改 赋予至服务通信中-----------------------------------------------------------------------------*/ 
    PID_revise(x_kp,x_ki,x_kd,x_speed_max,x_speed_min,                  //x_kp x_ki x_kd x_speed_max x_speed_min
               y_kp,y_ki,y_kd,y_speed_max,y_speed_min ,                //y_kp y_ki y_kd y_speed_max y_speed_min
               z_kp,z_ki,z_kd,z_speed_max,z_speed_min,                //z_kp z_ki z_kd z_speed_max z_speed_min
               yaw_kp,yaw_ki,yaw_kd,yaw_speed_max,yaw_speed_min,      //yaw_kp yaw_ki yaw_kd yaw_speed_max yaw_speed_min
               cv_x_kp,cv_x_ki,cv_x_kd,
               cv_y_kp,cv_y_ki,cv_y_kd,
               cv_z_kp,cv_z_ki,cv_z_kd,
               cv_yaw_kp,cv_yaw_ki,cv_yaw_kd,
               aim_pos.request);    
    
    /*-------------------------------------------------------------------------------------------------*/  
    fly_control_with_time(aim_pos,pos_client, 0 , 0 , 1.75 , 0*PI/180 , ON,1.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
    /*-------------------------------------------------------------------------------------------------*/  
    fly_control_with_time(aim_pos,pos_client, 0 , -2.55 , 1.75 , 0*PI/180 , ON,1.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 1.82 , -3.83, 1.75 , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff 
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 3.14 ,  -3.83, 1.75 , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff 
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 3.14 ,  -3.15, 1.75 , 0*PI/180 , ON,1.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff                                                   
    /*-------------------------------------------------------------------------------------------------*/

    //到达识别区域 准备进入攻击区域
    ROS_INFO("射击位置:",cv_pos_flag.cv_position_flag);
    if(cv_pos_flag.cv_position_flag==1){ //1指向最左边区域
        fly_control_with_time(aim_pos,pos_client, 111 , 111, 1.75 , 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
            3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff                                                   
    }
    else if(cv_pos_flag.cv_position_flag==2){ //2指向中间区域
        fly_control_with_time(aim_pos,pos_client, 111 , 111, 1.75 , 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
            3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff                                                   
    }
    else if(cv_pos_flag.cv_position_flag==3){ //3指向最右边区域
        fly_control_with_time(aim_pos,pos_client, 111 , 111, 1.75 , 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
            3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);          //角度精度   位置精度 ff                                                   
    }

    /*-------------------------------------------------------------------------------------------------*/   
    fly_control_with_time(aim_pos,pos_client, 2.56 , -1.30 , 1.75 , 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);           //角度精度   位置精度  
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 1.00 , -3.19 , 1.75 , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);           //角度精度   位置精度  
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 0 , -3.19 , 1.75 , 0*PI/180 , ON,0.5f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);           //角度精度   位置精度  
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 0 , 0 , 1.75 , 0*PI/180 , ON,1.0f,100, //x y z yaw model time hz
                          3*PI/180 , 0.1f, 3*PI/180, 25.0f, 25.0f);           //角度精度   位置精度  
    /*-------------------------------------------------------------------------------------------------*/ 
    fly_control_with_time(aim_pos,pos_client, 0 , 0 , 0 , 0*PI/180 , Land,1.5f,100, //x y z yaw model time hz
                         3*PI/180 , 0.07f, 3*PI/180, 25.0f, 15.0f);           //角度精度   位置精度  
   
    ROS_INFO("fly_task is close");
    while(ros::ok()){
        ros::spinOnce();
        pos_rate.sleep();
    }
}