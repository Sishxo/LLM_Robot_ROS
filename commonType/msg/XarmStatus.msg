time stamp          #时间戳
int32 TaskId        #当前执行任务的ID号，（-1）表示没任务
int8 PrimaryId      #机械臂当前处于主要状态，（0）表示home状态
int8 SecondaryId    #机械臂当前处于次要状态
int8 Status         #机械臂当前状态：0:空闲（IDLE）,1:工作中（WORKING），-1:异常（ERROR）