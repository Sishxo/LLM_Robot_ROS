#!/home/orin1/anaconda3/envs/alpaca/bin/python
from .LLM_core import generate_prompt, generate_multi_round_prompt, detect_command, generate_ros_response
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
from commonType.srv import APP_LLM_Algo
from commonType.msg import interfaceControl
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

apply_attention_patch(use_memory_efficient_attention=True)


AFFORDANCE_OBJECTS = ['牛奶', '蘑菇汤', '橙汁', '面包',
                      '饼干', '威化饼', '蛋糕', '纸巾', '胶带', '晶圆', '开发板']

AFFORDANCE_POSITIONS = ['A', 'B', 'C', 'D', '用户']

# DEFAULT_SYSTEM_PROMPT = """You are a helpful assistant. 你是一个乐于助人的助手。"""\
DEFAULT_SYSTEM_PROMPT = """
你是一个服务员助手，你叫IPHAC Assistant，由IPHAC Lab开发。
我们的饮料"只有"：<牛奶、蘑菇汤、橙汁>；我们的食物"只有"：<面包、饼干、威化饼、蛋糕>；我们的工具"只有"：<纸巾、胶带、晶圆、开发板>。
我们"只有尖括号中提到的物品"，当用户要求这些东西之外的物品时，你应该告诉他我们没有这些东西之外的物品。你的回答"不能超出这些物品"！
用户也可以直接发出请求来控制机器人的移动，或者抓取或放下物体。 当认识到这样的需求时，需要提取诸如：<移动到A点>、<抓取物体>、<将物体放置在B点>等信息直接输出。
我们的场景中"只有":<A,B,C,D>四个地点可以到达，我们"只有尖括号中提到的地点"，当用户要求去往这些地点之外的地方时，你应该告诉他我们无法去往其他地点。
"""

TEMPLATE = (
    "[INST] <<SYS>>\n"
    "{system_prompt}\n"
    "<</SYS>>\n\n"
    "{instruction} [/INST]"
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


class LLM:

    def __init__(self) -> None:
        self.args = Arguments()
        self.generation_config = GenerationConfig(
            temperature=0.1,
            top_k=20,
            top_p=0.9,
            do_sample=True,
            num_beams=1,
            repetition_penalty=1.1,
            max_new_tokens=400,
            output_scores=True
        )
        self.allMessages = ""
        self.model=None
        self.tokenizer = None
        self.device = None
        

    def Init(self):
        load_type = torch.float16

        if torch.cuda.is_available():
            device = torch.device(0)
        else:
            device = torch.device('cpu')
        args = self.args

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
            trust_remote_code=True
        )

        model_vocab_size = base_model.get_input_embeddings().weight.size(0)
        tokenizer_vocab_size = len(tokenizer)
        print(
            f"Vocab of the base model: {model_vocab_size}; Vocab of the tokenizer: {tokenizer_vocab_size}")
        rospy.loginfo("*******************************IPHAC Assistant is Ready!*******************************")
        if model_vocab_size != tokenizer_vocab_size:
            print("Resize model embeddings to fit tokenizer")
            base_model.resize_token_embeddings(tokenizer_vocab_size)

        self.model = base_model
        self.tokenizer = tokenizer
        self.device = device

    def LLM_infer(self, raw_input_text):
        model = self.model
        tokenizer = self.tokenizer
        device = self.device
        args = self.args

        if device == torch.device('cpu'):
            model.float()
        model.eval()

        with torch.no_grad():
            if len(raw_input_text.strip()) == 0:
                return -1

            negative_text = negative_text = None if args.negative_prompt is None \
                else generate_prompt(instruction=raw_input_text, system_prompt=args.negative_prompt)

            if self.allMessages != "":
                input_text = generate_multi_round_prompt(
                    instruction=raw_input_text, prompt_last_step=self.allMessages)
                self.allMessages = input_text
            else:
                input_text = generate_prompt(
                    instruction=raw_input_text, system_prompt=args.system_prompt)
                self.allMessages = input_text

            inputs = tokenizer(input_text, return_tensors="pt")

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

            generation_output = model.generate(
                input_ids=inputs["input_ids"].to(device),
                attention_mask=inputs['attention_mask'].to(device),
                eos_token_id=tokenizer.eos_token_id,
                pad_token_id=tokenizer.pad_token_id,
                generation_config=self.generation_config,
                guidance_scale=args.guidance_scale,
                negative_prompt_ids=negative_prompt_ids,
                negative_prompt_attention_mask=negative_prompt_attention_mask,
                return_dict_in_generate=True,
                output_scores=True
            )

            input_length = 1 if model.config.is_encoder_decoder else inputs.input_ids.shape[
                1]
            # only leave the newest generated tokens, induce the time waste on decoding
            generated_tokens = generation_output.sequences[:, input_length:]
            response = tokenizer.decode(
                generated_tokens[0], skip_special_tokens=True)  # single round response

            return response

    def LLMCallback(self, req):
        user_input = req.message
        
        rospy.loginfo(f"User:{user_input}")

        llm_response, object_check_flag, move_check_flag, grip_check_flag, drop_check_flag, request_object, request_pos, request_grip_object, request_drop_pos = detect_command(user_input)

        if llm_response == "":
            llm_response = self.LLM_infer(user_input)
            self.allMessages += llm_response
        else:
            self.allMessages = ""
            
        rospy.loginfo(f"LLM:{llm_response}")

        response = generate_ros_response(llm_response, {"object_check_flag": object_check_flag, "move_check_flag": move_check_flag,
                                                            "grip_check_flag": grip_check_flag, "drop_check_flag": drop_check_flag},
                                                            {"request_object": request_object, "request_pos": request_pos,
                                                            "request_grip_object": request_grip_object, "request_drop_pos": request_drop_pos})
        

        return response

    def Start(self):
        rospy.init_node("AppAnalysis_node")
        rospy.Service('APP_LLM_Algo_topic', APP_LLM_Algo, self.LLMCallback)
        rospy.loginfo("Service server APP_LLM_Algo_topic is ready now!")
        rospy.spin()
        
    def End(self): 
        del self
