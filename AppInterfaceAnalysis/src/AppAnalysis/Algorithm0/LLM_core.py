#!/home/orin1/anaconda3/envs/alpaca/bin/python
import argparse
import json, os
# from transformers.generation import UnbatchedClassifierFreeGuidanceLogitsProcessor
import torch
from transformers import AutoModelForCausalLM, LlamaTokenizer
from transformers import GenerationConfig
from transformers import BitsAndBytesConfig
import sys
import time
import numpy as np

from attn_and_long_ctx_patches import apply_attention_patch
apply_attention_patch(use_memory_efficient_attention=True)


AFFORDANCE_ITEMS = ['牛奶','蘑菇汤','橙汁','面包','饼干','威化饼','蛋糕','纸巾','胶带','晶圆','开发板']

# DEFAULT_SYSTEM_PROMPT = """You are a helpful assistant. 你是一个乐于助人的助手。"""\
DEFAULT_SYSTEM_PROMPT = """
你是一个服务员助手，你叫IPHAC Assistant，由IPHAC Lab开发。
我们的饮料"只有"：<牛奶、蘑菇汤、橙汁>；我们的食物"只有"：<面包、饼干、威化饼、蛋糕>；我们的工具"只有"：<纸巾、胶带、晶圆、开发板>。
我们"只有尖括号中提到的物品"，当用户要求这些东西之外的物品时，你应该告诉他我们没有这些东西之外的物品。你的回答"不能超出这些物品"！
用户也可以直接发出请求来控制机器人的移动，或者抓取或放下物体。 当认识到这样的需求时，需要提取诸如：<移动到A点>、<抓取物体>、<将物体放置在B点>等信息直接输出。"""
# You are a waiter assistant, your name is IPHAC Assistant, developed by IPHAC lab. Your function is to provide customers with the items they need and satisfy their needs. The drinks in the scene are now "only": <milk, mushroom soup, orange juice>; the foods in the scene are "only": <bread, biscuits, wafers, cake>; the tools in the scene are "only": <paper towel, tape , wafer, development board>.
# Note "Only items mentioned in angle brackets", when the user asks for items other than these things, you should tell him that we don't have items other than these things. Your answer is "no more than these items"!
# The user will also directly issue a request to control the movement of the robot, or to grab or put down an object. When you recognize such a requirement, you need to extract information such as: <move to point A>, <grab an object>, <will The information of an object placed at point B> is output directly."""


OPTION_SYSTEM_PROMPT="""
"""

TEMPLATE = (
    "[INST] <<SYS>>\n"
    "{system_prompt}\n"
    "<</SYS>>\n\n"
    "{instruction} [/INST]"
)

os.environ["CUDA_VISIBLE_DEVICES"] = '0'

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)

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
        self.system_prompt= DEFAULT_SYSTEM_PROMPT
        self.negative_prompt = None
        self.guidance_scale = 1.1
        self.speculative_sampling = False
        self.draft_k = -1
        self.draft_base_model = None
        self.draft_model_load_in_8bit = False
        self.draft_model_load_in_4bit = True
        self.use_flash_attention_2 = False

def generate_prompt(instruction, system_prompt=DEFAULT_SYSTEM_PROMPT):
    return TEMPLATE.format_map({'instruction': instruction,'system_prompt': system_prompt})

def generate_multi_round_prompt(instruction,prompt_last_step):
    return prompt_last_step+" </s><s>[INST] "+instruction+" [/INST]"

def get_object_check(raw_input_text,affordance_list=AFFORDANCE_ITEMS):
    
    check_flag = False
    request_item = ""
    
    if raw_input_text.find('我想喝')+1 or raw_input_text.find('我想要')+1 or raw_input_text.find('我想吃')+1 or raw_input_text.find('请帮我')+1 or raw_input_text.find('我要')+1 or raw_input_text.find('拿')+1: # .find返回值为-1与0
        for item in affordance_list:
            if item in raw_input_text:
                check_flag=True
                request_item+= item
                break
        if check_flag == False: #若匹配到请求，但物品不存在，返回True与""，警告用户物品不存在
            check_flag=True
    
    return check_flag,request_item

def get_move_check():
    pass
def get_grip_check():
    pass

def execute(request_item):
    input(f"GETTING {request_item}....")
    
def init_model():
    load_type = torch.float16

    if torch.cuda.is_available():
        device = torch.device(0)
    else:
        device = torch.device('cpu')

    args = Arguments()

    tokenizer = LlamaTokenizer.from_pretrained(args.tokenizer_path, legacy=True)
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
        quantization_config=quantization_config if (args.load_in_4bit or args.load_in_8bit) else None,
        trust_remote_code=True
    )

    model_vocab_size = base_model.get_input_embeddings().weight.size(0)
    tokenizer_vocab_size = len(tokenizer)
    print(f"Vocab of the base model: {model_vocab_size}")
    print(f"Vocab of the tokenizer: {tokenizer_vocab_size}")
    if model_vocab_size!=tokenizer_vocab_size:
        print("Resize model embeddings to fit tokenizer")
        base_model.resize_token_embeddings(tokenizer_vocab_size)

    model=base_model

    
    return model,tokenizer,device,args
    
def single_round_infer(model,tokenizer,device,args,input_text):
    
    if device==torch.device('cpu'):
        model.float()
    model.eval()
    
    with torch.no_grad():

        if len(input_text.strip())==0:
            return -1
        
        input_text = generate_prompt(instruction=input_text, system_prompt=args.system_prompt)
        
        negative_text = None if args.negative_prompt is None \
                    else generate_prompt(instruction=args.negative_prompt, system_prompt=args.negative_prompt)

        inputs = tokenizer(input_text,return_tensors="pt")  #add_special_tokens=False ?
        if negative_text is None:
            negative_prompt_ids = None
            negative_prompt_attention_mask = None
        else:
            negative_inputs = tokenizer(negative_text,return_tensors="pt")
            negative_prompt_ids = negative_inputs["input_ids"].to(device)
            negative_prompt_attention_mask = negative_inputs["attention_mask"].to(device)
        generation_output = model.generate(
                input_ids = inputs["input_ids"].to(device),
                attention_mask = inputs['attention_mask'].to(device),
                eos_token_id=tokenizer.eos_token_id,
                pad_token_id=tokenizer.pad_token_id,
                generation_config = generation_config,
                guidance_scale = args.guidance_scale,
                negative_prompt_ids = negative_prompt_ids,
                negative_prompt_attention_mask = negative_prompt_attention_mask,
                return_dict_in_generate=True,
                output_scores=True
            )
            
        # print(generation_output)
        s = generation_output[0]
        output = tokenizer.decode(s,skip_special_tokens=True)
        response = output.split("[/INST]")[-1].strip()

        return response
    
def saycan_match():
    pass

def run():
    load_type = torch.float16

    if torch.cuda.is_available():
        device = torch.device(0)
    else:
        device = torch.device('cpu')

    args = Arguments()

    tokenizer = LlamaTokenizer.from_pretrained(args.tokenizer_path, legacy=True)
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
        quantization_config=quantization_config if (args.load_in_4bit or args.load_in_8bit) else None,
        trust_remote_code=True,
        output_attentions=True
    )

    model_vocab_size = base_model.get_input_embeddings().weight.size(0)
    tokenizer_vocab_size = len(tokenizer)
    print(f"Vocab of the base model: {model_vocab_size}")
    print(f"Vocab of the tokenizer: {tokenizer_vocab_size}")
    if model_vocab_size!=tokenizer_vocab_size:
        print("Resize model embeddings to fit tokenizer")
        base_model.resize_token_embeddings(tokenizer_vocab_size)

    model=base_model

    if device==torch.device('cpu'):
        model.float()
    model.eval()

    with torch.no_grad():
        print("Start inference with instruction mode.")

        output=None

        while True:
            raw_input_text = input("Input:")
            if len(raw_input_text.strip())==0:
                break
            if output is not None:
                input_text = generate_multi_round_prompt(instruction=raw_input_text,prompt_last_step=output)
                negative_text = None if args.negative_prompt is None \
                    else generate_prompt(instruction=raw_input_text, system_prompt=args.negative_prompt)
            else:
                input_text = generate_prompt(instruction=raw_input_text, system_prompt=args.system_prompt)
                negative_text = None if args.negative_prompt is None \
                    else generate_prompt(instruction=raw_input_text, system_prompt=args.negative_prompt)


            inputs = tokenizer(input_text,return_tensors="pt")  #add_special_tokens=False ?
            if args.guidance_scale == 1:
                generation_output= model.generate(
                    input_ids = inputs["input_ids"].to(device),
                    attention_mask = inputs['attention_mask'].to(device),
                    eos_token_id=tokenizer.eos_token_id,
                    pad_token_id=tokenizer.pad_token_id,
                    generation_config = generation_config
                )
            else: # enable CFG sampling
                if negative_text is None:
                    negative_prompt_ids = None
                    negative_prompt_attention_mask = None
                else:
                    negative_inputs = tokenizer(negative_text,return_tensors="pt")
                    negative_prompt_ids = negative_inputs["input_ids"].to(device)
                    negative_prompt_attention_mask = negative_inputs["attention_mask"].to(device)
                    
                time1=time.time()
                generation_output = model.generate(
                    input_ids = inputs["input_ids"].to(device),
                    attention_mask = inputs['attention_mask'].to(device),
                    eos_token_id=tokenizer.eos_token_id,
                    pad_token_id=tokenizer.pad_token_id,
                    generation_config = generation_config,
                    guidance_scale = args.guidance_scale,
                    negative_prompt_ids = negative_prompt_ids,
                    negative_prompt_attention_mask = negative_prompt_attention_mask,
                    return_dict_in_generate=True,
                    output_scores=True
                )
                time2=time.time()
                transition_scores  = model.compute_transition_scores(generation_output.sequences, generation_output.scores, normalize_logits=True)
                # breakpoint()
                
            input_length = 1 if model.config.is_encoder_decoder else inputs.input_ids.shape[1]
            generated_tokens = generation_output.sequences[:, input_length:]
            output = tokenizer.decode(generated_tokens[0],skip_special_tokens=True)
            response = output.split("[/INST]")[-1].strip()
            
            # for tok, score in zip(generated_tokens[0], transition_scores[0]):
            #     print(f"| {tok:5d} | {tokenizer.decode(tok):8s} | {score.cpu().numpy():.3f} | {np.exp(score.cpu().numpy()):.2%}")

            check_flag,request_item=get_object_check(raw_input_text)        

            if check_flag and request_item!="":
                response=f"即将为您取来{request_item}"
                
                print("Response: ",response)
                print("\n")
                execute(request_item) # waiting ROS execution result
                
                output = input_text + response
            elif check_flag and request_item=="":
                response=f"未找到您指定的物品"
                print("Response: ",response)
                print("\n")
                output = input_text + response
            elif check_flag == False:
                print("Response: ",response)
                output = input_text + response
                print("\n")



if __name__ == '__main__':
    run()