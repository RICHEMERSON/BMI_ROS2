####================================================
#只要猴子hold了target足够的时间后，feedback呈现，在呈现过程中或者呈现完后必须离手才有reward
#该代码要求猴子必须离手后才给reward
## 猴子一点上target就呈现feedback
#时间的参数都用帧数，帧率是60Hz，长度单位是cm
#为了保证最后在8个方向的数据量是一致的，我们这里设置了trial_laststep_go（表示猴子触摸target时已经hold到了足够的时间，可能在正确的范围，也可能在错误的范围）
#和trial_laststep_nogo两个变量（go-signal呈现后开始算起，猴子必须要一直hold住）
import argparse
import os

parser = argparse.ArgumentParser(description='behavior script')
parser.add_argument('-f', '--folder', type=str, help='First name', required=False)
args = parser.parse_args()
bhv_folder = args.folder

##################################

task_par = {}
task_par['w']=0 #角速度
task_par['target_degree']=[0,45,90,135,180,225,270,315]#8个位置
task_par['center_size']=1 #中心点半径
task_par['center_radius_size']=2 #中心点容错半径
task_par['radius']=12 #目标点离中心点的距离
task_par['target_size']=1 # 目标点半径
task_par['target_radius_size']=2 #目标点容错半径
task_par['fullscr_area_size']=37  ##勿动！
task_par['feedback_size']=1  #反馈点半径
task_par['gosignal_size']=1  # go-signal 半径
task_par['gosignal_color']=(-1,-1,-1)  # go-signal rgb值，范围是-1到1
task_par['acquire_frame_num']=300 #中心点呈现后必须在该时间内触摸屏幕
task_par['holdcenter_frame_num']=30  #hold center时间
task_par['hold_nogo_frame_num']=60  #nogo时从gosignal出现到feedback出现需要hold center的时间
task_par['delay_frame_num_range']=[24,36]  #delay时长的范围，均匀分布
task_par['RTandMT_frame_num']=70 #go-signal呈现后，必须在该时间内触摸上屏幕
task_par['feedback_on_frame_num']=30 #hold 目标点的时间
task_par['wait_before_reward_frame_num']=3 #feedback呈现的时间
task_par['ITI_frame_num']=60 #inter-trial interval
task_par['Trialnum_in_onesession']=800 #一个session里的trial 数目
task_par['voltage']=5  # 给reward的电压值
task_par['duration']=0.5 #给reward的时间
task_info = {}
task_info['Task parameters'] = task_par
####================================================

for i in task_par:
    tmp = task_par[i]
    exec(f'{i} = {tmp}')
    
    
from psychopy import monitors, visual, core, event
from psychopy.hardware.keyboard import KeyPress
import random, math,time
import numpy as np
import json
import logging
import center_out_go as go
import center_out_nogo as nogo
# from uldaq import (get_daq_device_inventory, DaqDevice, 
                   # InterfaceType, Range, AOutFlag, DigitalPortType, DigitalDirection)
t = int(time.time()*1000)
# 配置logging
f_name = f'test_{t}.log'

try:
    logging.basicConfig(
        filename=os.path.join(bhv_folder,f_name),  # 日志文件名
        #filename=f'/home/amax/psychopy_experiment_data/interception/experiment_{t}.log',  # 日志文件名
        filemode='a',                     # 追加模式
        format='%(asctime)s.%(msecs)03d %(message)s',  # 包含毫秒的时间戳
        datefmt='%Y-%m-%d %H:%M:%S',      # 日期时间格式
        level=logging.INFO                # 日志级别
    )
except:
    logging.basicConfig(
        filename=f_name,  # 日志文件名
        # filename=f'/home/amax/psychopy_experiment_data/interception/experiment_{t}.log',  # 日志文件名
        filemode='a',                     # 追加模式
        format='%(asctime)s.%(msecs)03d %(message)s',  # 包含毫秒的时间戳
        datefmt='%Y-%m-%d %H:%M:%S',      # 日期时间格式
        level=logging.INFO                # 日志级别
    )
    



  
def FrameNum(n):
    while n >= 0:
        yield n
        n += 1

fn = FrameNum(0)

def save_info(info):
    logging.info(json.dumps(info)) 

save_info(task_info)



newdegree_nogo = list(range(8))
random.shuffle(newdegree_nogo)
newdegree_go = list(range(8))
random.shuffle(newdegree_go)


trialnum_nogo=0
trialnum_go=0
trial_laststep_go = 0
trial_laststep_nogo = 0
trial_success_go = 0
trial_success_nogo = 0

## 定义窗口，呈现视觉刺激
screen_index = 1
mon = monitors.Monitor('MyMonitor', width=37.5) 
mon.setSizePix((1280, 1024))
# mon.setSizePix((1280, 1024))
mon.save()
win = visual.Window(fullscr=True, screen=screen_index, allowGUI=True, monitor=mon, color='black', units='cm', waitBlanking=False)

# 定义视觉刺激对象
fullscr_area = visual.Circle(win, radius=fullscr_area_size, name='background', pos=(0, 0), lineColor='red', fillColor='red', opacity=0)
center = visual.Circle(win, radius=center_size, name='center', pos=(0, 0), lineColor='black', fillColor='green')
target_nogo = visual.Circle(win, radius=target_size, name='target', pos=(0, 0), lineColor='black', fillColor='red') #### nogo
target_go = visual.Circle(win, radius=target_size, name='target', pos=(0, 0), lineColor='black', fillColor='green') #### nogo
gosignal = visual.Circle(win, radius=gosignal_size, name='gosignal', pos=(0, 0), colorSpace='rgb', lineColor=gosignal_color, fillColor=gosignal_color)
center_radius = visual.Circle(win, radius=center_radius_size, name='center_radius', pos=(0, 0), lineColor='red', fillColor='red', opacity=0)
target_radius = visual.Circle(win, radius=target_radius_size, name='target_radius', pos=(0, 0), lineColor='green', fillColor='green', opacity=0)
right_feedback = visual.Circle(win, radius=feedback_size, name='right_feedback', pos=(0, 0), lineColor='black', fillColor='black')
wrong_feedback = visual.Circle(win, radius=feedback_size, name='wrong_feedback', pos=(0, 0), lineColor='black', fillColor='black')
mouse = event.Mouse()
win.mouseVisible = False
go_type=[0,0,0,0,0,0,0,0,0,0,0]
nogo_type=[0,0,0,0,0,0,0,0]
global viconflag
viconflag=False
for trialnum in range(Trialnum_in_onesession):
    if random.random()<0.5:
        # continue
        if (trial_laststep_nogo % 8) == 7:
            newdegree_nogo = list(range(8))
            random.shuffle(newdegree_nogo)
        degree = target_degree[newdegree_nogo[trial_laststep_nogo % 8]]
        nogo_type,trial_laststep_nogo,viconflag=nogo.center_out_nogo(nogo_type,
                fn,logging,trialnum_nogo,trial_laststep_nogo,trial_success_nogo,degree,mouse,win,radius,
                fullscr_area,center,center_radius,target_nogo,target_radius,wrong_feedback,right_feedback,gosignal,
                acquire_frame_num,holdcenter_frame_num,hold_nogo_frame_num,
                delay_frame_num_range,RTandMT_frame_num,feedback_on_frame_num,wait_before_reward_frame_num,ITI_frame_num,
                voltage,duration,viconflag
            )
    else:
        # continue
        if (trial_laststep_go % 8) == 7:
            newdegree_go = list(range(8))
            random.shuffle(newdegree_go)
            
        degree = target_degree[newdegree_go[trial_laststep_go % 8]]
        go_type,trial_laststep_go,viconflag=go.center_out_go(go_type,
            fn,logging,trialnum_go,trial_laststep_go,trial_success_go,degree,mouse,win,radius,
            fullscr_area,center,center_radius,target_go,target_radius,wrong_feedback,right_feedback,gosignal,
            acquire_frame_num,holdcenter_frame_num,hold_nogo_frame_num,
            delay_frame_num_range,RTandMT_frame_num,feedback_on_frame_num,wait_before_reward_frame_num,ITI_frame_num,
            voltage,duration,viconflag
        )
        
    if trialnum_nogo+trialnum_go==28:
        break
    
win.close()
# print("TrialNumber_go=",trialnum_go+1,"   success rate=",trial_success_go/(trialnum_go+1))
print("TrialNumber_nogo=",trialnum_nogo,"   success rate=",trial_success_nogo/trialnum_nogo)
# return trialnum_go,trial_laststep_go,trial_success_go