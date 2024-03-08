time stamp                     #时间戳
int8 TaskID                    #任务ID
int8 ObjectID                  #识别物体的id
int8 GripperControlModel       #机械抓控制状态 0(无任务)，1（抓取），2（放置），3（回到home）
int8 XarmControlModel          #机械臂控制状态 0（若上一个任务执行过程中，忽略本次任务），1（强制执行最新任务）
int8 XarmSpeedLevel            # 0～5等级
int8 GripSpeedLevel            # 0~5等级

float32 target_x 
float32 target_y
float32 target_z
float32 target_roll 
float32 target_pitch 
float32 target_yaw 