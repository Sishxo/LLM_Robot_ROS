import torch
import sys
sys.path.append('/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/yolov5_depend')

from utils.torch_utils import select_device, smart_inference_mode
from models.common import DetectMultiBackend
import numpy as np

from utils.augmentations import letterbox

import json
import time

from utils.general import (
    check_img_size,
    non_max_suppression,
    scale_boxes,
    xyxy2xywh,
)

def img_transform(image,model):

    image = letterbox(image, 640, stride=32, auto=True)[0]  # padded resize
    image = image.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    image = np.ascontiguousarray(image) 

    image = torch.from_numpy(image).to(model.device)
    image = image.float()
    image /= 255

    if len(image.shape) == 3:
        image = image[None] # expand dimensions to 1,R,G,B
    return image

def model_init(
    weights='/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/yolov5_depend/weights/best.pt',
    data='/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/yolov5_depend/data/LLM_robotic.yaml',
    device="0",
):
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=False, data=data, fp16=False)
    print(type(model))
    return model

@smart_inference_mode()
def run(
    image, # it would be a CXX cv::mat format image
    model, #yolo model
    imgsz=(640, 640),  # inference size (height, width)
):
    # model Init
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    gn = torch.tensor(image.shape)[[1, 0, 1, 0]]

    bs = 1 

    #Inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup

    transformed_image = img_transform(image,model)
    
    pred = model(transformed_image, augment=False, visualize=False)

    pred = non_max_suppression(pred, 0.25, 0.45, None, False, max_det=9) #(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

    # annotator = Annotator(image, line_width=3, example=str(names))

    # breakpoint()

    res=[]
    det=pred[0]
    if len(det):
        det[:, :4] = scale_boxes(transformed_image.shape[2:], det[:, :4], image.shape).round()
        for *xyxy, conf, cls in reversed(det):
            c=int(cls)
            label=names[c]
            confidence = float(conf)
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
            dict={'object':label,'object_id':c,'conf':confidence,'center':[xywh[0],xywh[1]],'width':xywh[2],'height':xywh[3]}
            res.append(dict)
    
    json_res=json.dumps(res)
    return json_res





if __name__=='__main__':
    import cv2
    image = cv2.imread('/home/orin1/data/yolov5/data/images/test.jpg')
    init_start=time.time()
    model=model_init()
    init_end=time.time()
    print(init_end-init_start)
    run(model,image)
    run_end=time.time()
    print(run_end-init_end)