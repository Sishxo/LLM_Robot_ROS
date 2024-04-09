#!/home/orin1/anaconda3/envs/alpaca/bin/python
from commonType.srv import APP_LLM_AlgoResponse
from commonType.msg import interfaceControl
from .attn_and_long_ctx_patches import apply_attention_patch
import os
# from transformers.generation import UnbatchedClassifierFreeGuidanceLogitsProcessor
import torch
from transformers import AutoModelForCausalLM, LlamaTokenizer
from transformers import GenerationConfig
from transformers import BitsAndBytesConfig
import time
import numpy as np
import rospy

import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

apply_attention_patch(use_memory_efficient_attention=True)


AFFORDANCE_OBJECTS = ['饼干', '蛋糕', '可乐', '柠檬茶', '牛奶', '蘑菇汤', '胶带''纸巾',  '威化饼']

AFFORDANCE_POSITIONS = ['A', 'B', 'C', 'D', '用户']

# DEFAULT_SYSTEM_PROMPT = """You are a helpful assistant. 你是一个乐于助人的助手。"""\
DEFAULT_SYSTEM_PROMPT = """
你是一个服务员助手,你叫IPHAC Assistant,由IPHAC Lab开发。
我们的饮料"只有":<牛奶、蘑菇汤、柠檬茶,可乐>；我们的食物"只有":<饼干、威化饼、蛋糕>；我们的工具"只有":<胶带，纸巾>。
我们"只有尖括号中提到的物品"，当用户要求这些东西之外的物品时，你应该告诉他我们没有这些东西之外的物品。你的回答"不能超出这些物品"!
用户也可以直接发出请求来控制机器人的移动，或者抓取或放下物体。 当认识到这样的需求时，需要提取诸如：<移动到A点>、<抓取物体>、<将物体放置在B点>等信息直接输出。
我们的场景中"只有":<A,B,C,D>四个地点可以到达，我们"只有尖括号中提到的地点"，当用户要求去往这些地点之外的地方时，你应该告诉他我们无法去往其他地点。
"""
# You are a waiter assistant, your name is IPHAC Assistant, developed by IPHAC lab. Your function is to provide customers with the items they need and satisfy their needs. The drinks in the scene are now "only": <milk, mushroom soup, orange juice>; the foods in the scene are "only": <bread, biscuits, wafers, cake>; the tools in the scene are "only": <paper towel, tape , wafer, development board>.
# Note "Only items mentioned in angle brackets", when the user asks for items other than these things, you should tell him that we don't have items other than these things. Your answer is "no more than these items"!
# The user will also directly issue a request to control the movement of the robot, or to grab or put down an object. When you recognize such a requirement, you need to extract information such as: <move to point A>, <grab an object>, <will The information of an object placed at point B> is output directly."""

TEMPLATE = (
    "[INST] <<SYS>>\n"
    "{system_prompt}\n"
    "<</SYS>>\n\n"
    "{instruction} [/INST]"
)

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

generation_config = GenerationConfig(
    temperature=0.1,
    top_k=20,
    top_p=0.9,
    do_sample=True,
    num_beams=1,
    repetition_penalty=1.1,
    max_new_tokens=400,
    output_scores=True
)


class Arguments:
    def __init__(self):
        self.base_model = '/home/orin1/data/chinese-alpaca-2-7b-hf/models'
        self.tokenizer_path = '/home/orin1/data/chinese-alpaca-2-7b-hf/models'
        self.load_in_4bit = False
        self.load_in_8bit = False
        self.system_prompt = DEFAULT_SYSTEM_PROMPT
        self.negative_prompt = None
        self.guidance_scale = 1.1
        self.speculative_sampling = False
        self.draft_k = -1
        self.draft_base_model = None
        self.draft_model_load_in_8bit = False
        self.draft_model_load_in_4bit = True
        self.use_flash_attention_2 = False


def generate_prompt(instruction, system_prompt=DEFAULT_SYSTEM_PROMPT):
    return TEMPLATE.format_map({'instruction': instruction, 'system_prompt': system_prompt})


def generate_multi_round_prompt(instruction, prompt_last_step):
    return prompt_last_step+" </s><s>[INST] "+instruction+" [/INST]"


def get_object_check(raw_input_text, affordance_list=AFFORDANCE_OBJECTS):

    object_check_flag = False
    request_item = ""

    if raw_input_text.find('我想喝')+1 or raw_input_text.find('我想要')+1 or raw_input_text.find('我想吃')+1 or raw_input_text.find('请帮我')+1 or raw_input_text.find('我要')+1 or raw_input_text.find('拿')+1 or raw_input_text.find('取来')+1:  # .find返回值为-1与0
        for item in affordance_list:
            if item in raw_input_text:
                object_check_flag = True
                request_item += item
                break
        if object_check_flag == False:  # 若匹配到请求，但物品不存在，返回True与""，警告用户物品不存在
            object_check_flag = True

    return object_check_flag, request_item


def get_move_check(raw_input_text, affordance_list=AFFORDANCE_POSITIONS):

    move_check_flag = False
    request_pos = ""

    if raw_input_text.find('前往')+1 or raw_input_text.find('去往')+1 or raw_input_text.find('去到')+1 or raw_input_text.find('移动到')+1:  # .find返回值为-1与0
        for pos in affordance_list:
            if pos in raw_input_text:
                move_check_flag = True
                request_pos += pos
                break
        if move_check_flag == False:  # 若匹配到请求，但物品不存在，返回True与""，警告用户物品不存在
            move_check_flag = True

    return move_check_flag, request_pos


def get_grip_check(raw_input_text, affordance_list=AFFORDANCE_OBJECTS):

    grip_check_flag = False
    request_grip_item = ""

    if raw_input_text.find('抓取')+1 or raw_input_text.find('抓起')+1 or raw_input_text.find('拿起')+1 or raw_input_text.find('抓')+1:  # .find返回值为-1与0
        for item in affordance_list:
            if item in raw_input_text:
                grip_check_flag = True
                request_grip_item += item
                break
        if grip_check_flag == False:  # 若匹配到请求，但物品不存在，返回True与""，警告用户物品不存在
            grip_check_flag = True

    return grip_check_flag, request_grip_item


def get_drop_check(raw_input_text, affordance_list=AFFORDANCE_POSITIONS):

    drop_check_flag = False
    request_drop_pos = ""

    if raw_input_text.find('放到')+1 or raw_input_text.find('放下')+1:  # .find返回值为-1与0
        for item in affordance_list:
            if item in raw_input_text:
                drop_check_flag = True
                request_drop_pos += item
                break
        if drop_check_flag == False:  # 若匹配到请求，但物品不存在，返回True与""，警告用户物品不存在
            drop_check_flag = True

    return drop_check_flag, request_drop_pos


def detect_command(raw_input_text):
    flag = False
    object_check_flag, request_object = get_object_check(raw_input_text)
    move_check_flag, request_pos = get_move_check(raw_input_text)
    grip_check_flag, request_grip_object = get_grip_check(raw_input_text)
    drop_check_flag, request_drop_pos = get_drop_check(raw_input_text)

    response = ""

    if (object_check_flag and request_object != "") or (move_check_flag and request_pos != "") or (grip_check_flag and request_grip_object != "") or (drop_check_flag and request_drop_pos != ""):

        if move_check_flag:
            if request_pos != "":
                response += f"即将移动至{request_pos};"
            else:
                response += f"未指定目标地点！"

        if object_check_flag:
            if request_object != "":
                response += f"即将为您取来{request_object};"
            else:
                response += f"未指定目标物体！"
        elif grip_check_flag:
            if request_grip_object != "":
                response += f"即将为您抓取{request_grip_object};"
            else:
                response += f"未指定目标物体！"
        elif drop_check_flag:
            if request_drop_pos != "":
                response += f"即将为您将物品放置到{request_drop_pos};"
            else:
                response += f"未指定目标物体！"

    return response, object_check_flag, move_check_flag, grip_check_flag, drop_check_flag, request_object, request_pos, request_grip_object, request_drop_pos


def control_to_sequence(request_list, ActionType, app_llm_algo_service_res):

    control_message = interfaceControl()

    if ActionType == 0:
        control_message.ObjectID = 2
        control_message.gripObjectName = request_list["request_object"]
        control_message.gripObjectID = AFFORDANCE_OBJECTS.index(
            control_message.gripObjectName)
        app_llm_algo_service_res.control.append(control_message)
    elif ActionType == 1:
        control_message.ObjectID = 0
        control_message.end = request_list["request_pos"]
        app_llm_algo_service_res.control.append(control_message)
    elif ActionType == 2:
        control_message.ObjectID = 1
        control_message.gripObjectName = request_list["request_grip_object"]
        control_message.gripObjectID = AFFORDANCE_OBJECTS.index(
            control_message.gripObjectName)
        control_message.gripFunction = 0
        app_llm_algo_service_res.control.append(control_message)
    elif ActionType == 3:
        # step 1
        control_message.ObjectID = 0
        control_message.end = request_list["request_drop_pos"]
        app_llm_algo_service_res.control.append(control_message)

        # step 2
        control_message = interfaceControl()
        control_message.ObjectID = 1
        control_message.gripFunction = 1
        app_llm_algo_service_res.control.append(control_message)
    elif ActionType == 4:
        # step 1
        control_message.ObjectID = 0
        control_message.end = request_list["request_pos"]
        app_llm_algo_service_res.control.append(control_message)

        # step 2
        control_message = interfaceControl()
        control_message.ObjectID = 1
        control_message.gripObjectName = request_list["request_grip_object"]
        control_message.gripObjectID = AFFORDANCE_OBJECTS.index(
            control_message.gripObjectName)
        control_message.gripFunction = 0
        app_llm_algo_service_res.control.append(control_message)

    return app_llm_algo_service_res


def generate_ros_response(
    llm_response,
    flags={"object_check_flag": False, "move_check_flag": False,
           "grip_check_flag": False, "drop_check_flag": False},
    request_list={"request_object": "", "request_pos": "",
                  "request_grip_object": "", "request_drop_pos": ""},
):
    app_llm_algo_service_res = APP_LLM_AlgoResponse()

    if flags["object_check_flag"]:

        app_llm_algo_service_res.ActionType = 0

    elif flags["move_check_flag"]:
        if flags["grip_check_flag"]:
            app_llm_algo_service_res.ActionType = 4
        else:
            app_llm_algo_service_res.ActionType = 1
    elif flags["drop_check_flag"]:
        app_llm_algo_service_res.ActionType = 3
    elif flags["grip_check_flag"]:
        app_llm_algo_service_res.ActionType = 2
    else:
        app_llm_algo_service_res.ActionType = 5

    app_llm_algo_service_res = control_to_sequence(
        request_list, app_llm_algo_service_res.ActionType, app_llm_algo_service_res)

    app_llm_algo_service_res.message = llm_response

    return app_llm_algo_service_res


def saycan_match():
    pass


def run():
    load_type = torch.float16

    if torch.cuda.is_available():
        device = torch.device(0)
    else:
        device = torch.device('cpu')

    args = Arguments()

    tokenizer = LlamaTokenizer.from_pretrained(
        args.tokenizer_path, legacy=True)
    if args.load_in_4bit or args.load_in_8bit:
        quantization_config = BitsAndBytesConfig(
            load_in_4bit=args.load_in_4bit,
            load_in_8bit=args.load_in_8bit,
            bnb_4bit_compute_dtype=load_type,
        )
    base_model = AutoModelForCausalLM.from_pretrained(
        args.base_model,
        torch_dtype=load_type,
        low_cpu_mem_usage=True,
        device_map='auto',
        load_in_4bit=args.load_in_4bit,
        load_in_8bit=args.load_in_8bit,
        quantization_config=quantization_config if (
            args.load_in_4bit or args.load_in_8bit) else None,
        trust_remote_code=True,
        output_attentions=True
    )

    model_vocab_size = base_model.get_input_embeddings().weight.size(0)
    tokenizer_vocab_size = len(tokenizer)
    print(f"Vocab of the base model: {model_vocab_size}")
    print(f"Vocab of the tokenizer: {tokenizer_vocab_size}")
    if model_vocab_size != tokenizer_vocab_size:
        print("Resize model embeddings to fit tokenizer")
        base_model.resize_token_embeddings(tokenizer_vocab_size)

    model = base_model

    if device == torch.device('cpu'):
        model.float()
    model.eval()

    with torch.no_grad():
        print("Start inference with instruction mode.")

        output = None

        while True:
            raw_input_text = input("Input:")
            if len(raw_input_text.strip()) == 0:
                break
            if output is not None:
                input_text = generate_multi_round_prompt(
                    instruction=raw_input_text, prompt_last_step=output)
                negative_text = None if args.negative_prompt is None \
                    else generate_prompt(instruction=raw_input_text, system_prompt=args.negative_prompt)
            else:
                input_text = generate_prompt(
                    instruction=raw_input_text, system_prompt=args.system_prompt)
                negative_text = None if args.negative_prompt is None \
                    else generate_prompt(instruction=raw_input_text, system_prompt=args.negative_prompt)

            # add_special_tokens=False ?
            inputs = tokenizer(input_text, return_tensors="pt")
            if args.guidance_scale == 1:
                generation_output = model.generate(
                    input_ids=inputs["input_ids"].to(device),
                    attention_mask=inputs['attention_mask'].to(device),
                    eos_token_id=tokenizer.eos_token_id,
                    pad_token_id=tokenizer.pad_token_id,
                    generation_config=generation_config
                )
            else:  # enable CFG sampling
                if negative_text is None:
                    negative_prompt_ids = None
                    negative_prompt_attention_mask = None
                else:
                    negative_inputs = tokenizer(
                        negative_text, return_tensors="pt")
                    negative_prompt_ids = negative_inputs["input_ids"].to(
                        device)
                    negative_prompt_attention_mask = negative_inputs["attention_mask"].to(
                        device)

                time1 = time.time()
                generation_output = model.generate(
                    input_ids=inputs["input_ids"].to(device),
                    attention_mask=inputs['attention_mask'].to(device),
                    eos_token_id=tokenizer.eos_token_id,
                    pad_token_id=tokenizer.pad_token_id,
                    generation_config=generation_config,
                    guidance_scale=args.guidance_scale,
                    negative_prompt_ids=negative_prompt_ids,
                    negative_prompt_attention_mask=negative_prompt_attention_mask,
                    return_dict_in_generate=True,
                    output_scores=True
                )
                time2 = time.time()
                transition_scores = model.compute_transition_scores(
                    generation_output.sequences, generation_output.scores, normalize_logits=True)
                # breakpoint()

            input_length = 1 if model.config.is_encoder_decoder else inputs.input_ids.shape[
                1]
            generated_tokens = generation_output.sequences[:, input_length:]
            response = tokenizer.decode(
                generated_tokens[0], skip_special_tokens=True)

            # for tok, score in zip(generated_tokens[0], transition_scores[0]):
            #     print(f"| {tok:5d} | {tokenizer.decode(tok):8s} | {score.cpu().numpy():.3f} | {np.exp(score.cpu().numpy()):.2%}")

            object_check_flag, request_object = get_object_check(
                raw_input_text)
            move_check_flag, request_pos = get_move_check(raw_input_text)
            grip_check_flag, request_grip_object = get_grip_check(
                raw_input_text)
            drop_check_flag, request_drop_pos = get_drop_check(raw_input_text)

            print(
                f"object_check_flag:{object_check_flag} request_object:{request_object}")
            print(
                f"move_check_flag:{move_check_flag} request_pos:{request_pos}")
            print(
                f"grip_check_flag:{grip_check_flag} request_grip_object:{request_grip_object}")
            print(
                f"drop_check_flag:{drop_check_flag} request_drop_pos:{request_drop_pos}")

            if (object_check_flag and request_object != "") or (move_check_flag and request_pos != "") or (grip_check_flag and request_grip_object != "") or (drop_check_flag and request_drop_pos != ""):
                response = ""
                if move_check_flag:
                    if request_pos != "":
                        response += f"即将移动至{request_pos}"
                    else:
                        response += f"未指定目标地点！"

                if object_check_flag:
                    if request_object != "":
                        response += f"即将为您取来{request_object}"
                    else:
                        response += f"未指定目标物体！"
                elif grip_check_flag:
                    if request_grip_object != "":
                        response += f"即将为您抓取{request_grip_object}"
                    else:
                        response += f"未指定目标物体！"
                elif drop_check_flag:
                    if request_drop_pos != "":
                        response += f"即将为您将物品放置到{request_drop_pos}"
                    else:
                        response += f"未指定目标物体！"

            print("Response: ", response)

            # if check_flag and request_item!="":
            #     response=f"即将为您取来{request_item}"

            #     print("Response: ",response)
            #     print("\n")
            #     execute(request_item) # waiting ROS execution result

            #     output = input_text + response
            # elif check_flag and request_item=="":
            #     response=f"未找到您指定的物品"
            #     print("Response: ",response)
            #     print("\n")
            #     output = input_text + response
            # elif check_flag == False:
            #     print("Response: ",response)
            #     output = input_text + response
            #     print("\n")


if __name__ == '__main__':
    run()
