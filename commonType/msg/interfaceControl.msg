uint8 ObjectID  #控制物体，0表示车，1表示机械臂, 2表示仅给出目标物体
string start    #控制物体移动的开始点，空表示从当前位置出发
string end      #控制物体移动的结束点，空表示不移动
float32 speed   #控制物体移动的速度，0表示不移动
uint8 gripFunction    #控制物体操作，特别指定了机械臂膀，0表示抓，1表示放
uint8 gripObjectID  #控制机械臂抓取的物体id
string gripObjectName #控制机械臂抓取的物体名称
float32 delyaTime   #动作完成后等待时间