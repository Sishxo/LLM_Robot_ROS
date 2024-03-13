uint8 ActionType 
# 0代表指定物品，算法自动规划路线，抓取，识别顺序
# 1代表移动到TargetPos中的位置
# 2代表检测视野范围内的TargetObject并抓取
# 3代表前往TargetPos中的位置，并放下物品

string TargetObjectName
uint8 TargetObjectID
uint8 TargetPos
uint8 SpeedLevel

