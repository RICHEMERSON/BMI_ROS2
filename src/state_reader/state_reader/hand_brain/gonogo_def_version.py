# -*- coding: utf-8 -*-
"""
Created on Fri Dec 27 10:35:15 2024

@author: polar
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Dec 23 10:31:29 2024

@author: polar
"""

# -*- coding: utf-8 -*-
"""
Created on Mon Sep  2 18:15:12 2024

@author: polar
"""
# parser = argparse.ArgumentParser(description='behavior script')
# parser.add_argument('-f', '--folder', type=str, help='First name', required=False)
# args = parser.parse_args()
# bhv_folder = args.folder

task_par = {}
task_par['w']=0 #角速度
task_par['center_size']=1 #中心点半径
task_par['center_radius_size']=4 #中心点容错半径
task_par['radius']=12 #目标点离中心点的距离
task_par['target_size']=1 # 目标点半径
task_par['target_radius_size']=4#目标点容错半径
task_par['fullscr_area_size']=37  ##勿动！
task_par['feedback_size']=1  #反馈点半径
task_par['gosignal_size']=1  # go-signal 半径
task_par['gosignal_color']=(-1,-1,-1)  # go-signal rgb值，范围是-1到1
task_par['acquire_frame_num']=1.5 #中心点呈现后必须在该时间内触摸屏幕
task_par['holdcenter_frame_num']=1  #hold center时间
task_par['delay_frame_num_range']=[0,0]  #delay时长的范围，均匀分布
task_par['RTandMT_frame_num']=3 #go-signal呈现后，必须在该时间内触摸上屏幕
task_par['holdtarget_frame_num']=0.1667 #hold 目标点的时间
task_par['hold_nogo_frame_num']=0.1667  #nogo时从gosignal出现到feedback出现需要hold center的时间
task_par['feedback_on_frame_num']=0.1667 #feedback呈现的时间
task_par['wait_before_reward_frame_num']=0.1667 #feedback呈现的时间
task_par['ITI_frame_num']=90 #inter-trial interval
task_par['all_degree']=[0,45,90,135,180,225,270,315,0,45,90,135,180,225,270,315] #inter-trial interval
# task_par['all_degree']=[0,120,240,0,120,240] #inter-trial interval
task_par['Trialnum_in_onesession']= 3000 #一个session里的trial 数目
task_par['voltage']=5  # 给reward的电压值
task_par['duration']=0.5 #给reward的时间
task_info = {}
task_info['Task parameters'] = task_par
####================================================


from psychopy import monitors, visual, core, event
from psychopy.hardware.keyboard import KeyPress
import random, math,time
import numpy as np

def FrameNum(n):
    while n >= 0:
        yield n
        n += 1

fn = FrameNum(0)

def target_rand_pos(radius,degree):
    pos = np.zeros((2,))
    initial_angle = degree
    pos[0] = radius * math.cos(math.pi * initial_angle / 180)
    pos[1] = radius * math.sin(math.pi * initial_angle / 180)
    return pos

def GO_NO_GO():

    for i in task_par:
        tmp = task_par[i]
        exec(f'{i} = {tmp}')

    bhv_flag = False
    touch_center_flag = False
    touch_target_flag = False
    hold_feedback_flag = True

    start_time=time.time()
    while time.time()-start_time < acquire_frame_num:
        
        center.draw()
        win.flip()


        if mouse.isPressedIn(fullscr_area, buttons=[0]):
            touch_center_flag = True
            break

    #================================================
    #| hold center
    #================================================
    if not touch_center_flag:
        return bhv_flag
    else:
        hold_center_flag = True
        start_time=time.time()
        while time.time()-start_time < holdcenter_frame_num:
            center.draw()
            win.flip()
            # win_flip_info([center],fn)
            if not mouse.isPressedIn(fullscr_area, buttons=[0]):
                hold_center_flag = False
                break

        if not hold_center_flag:
            return bhv_flag

        if not mouse.isPressedIn(center_radius, buttons=[0]):
            return bhv_flag
    #================================================
    #| target on (need to change color)
    #================================================
    ##################### 
    start_time=time.time()
    while time.time()-start_time < delay_frame_num:
        center.draw()
        target.draw()
        win.flip()

        if not mouse.isPressedIn(center_radius, buttons=[0]):
            # exp_quit(win,go_type[0], go_type[2])
            hold_center_flag = False
            break

    if not hold_center_flag:
        return bhv_flag
        # wrong

    #================================================
    #| GO signal on Go choice
        # eventmarker(4)
        # marker_publisher({"Event":"gosignal on","Marker": 4}) 
    #================================================
    if hand_flag:  # 手动

        start_time=time.time()

        while time.time()-start_time <RTandMT_frame_num:    
            gosignal.draw()
            target.draw()
            win.flip()

            buttons = mouse.getPressed()
            if buttons[0] == 0:         
                RT_frame_num = time.time()-start_time
                hold_center_flag=False
                break
        if hold_center_flag:
            return bhv_flag
    
        #================================================
        #| Movement onset
        #================================================
    
        start_time=time.time()
        while time.time()-start_time < (RTandMT_frame_num - RT_frame_num):
            gosignal.draw()
            target.draw()
            win.flip()
            # win_flip_info([gosignal, target],fn)
            buttons = mouse.getPressed()
            if buttons[0] == 1:
                # eventmarker(6)
                touch_target_flag = True
                MT_frame_num = time.time()-start_time           
                break
        #================================================
        #| Touch target
        #================================================
        if not touch_target_flag:
            return bhv_flag
        else:
            #================================================
            #| Feedback on
            #================================================
            hold_target_flag = True
            if not mouse.isPressedIn(target_radius,buttons=[0]): # 触摸不在容错范围内
                return bhv_flag              
            
            else:# 触摸在容错范围内               
                right_feedback.pos= mouse.getPos()    
                start_time=time.time()
                while time.time()-start_time < feedback_on_frame_num:
                        gosignal.draw()
                        target.draw() 
                        right_feedback.draw()
                        win.flip()
                        buttons = mouse.getPressed()
                        if buttons[0] == 0: # hold target时中途离手
                            hold_target_flag=False
                            break
                if not hold_target_flag:  # hold target时中途离手,错误
                    return bhv_flag
                else:
                    while(True):
                        gosignal.draw()
                        win.flip()
                        buttons = mouse.getPressed()
                        if buttons[0] == 0: # hold target时中途离手,默认不离手
                            break
                    bhv_flag = True
                    return bhv_flag
    else:
        #================================================
        #no Go choice
        #================================================
        start_time=time.time()
        while time.time()-start_time <hold_nogo_frame_num:
            gosignal.draw()
            target.draw()
            win.flip()
            buttons = mouse.getPressed()
            if buttons[0] == 0:
                hold_center_flag= False
                break
        if not hold_center_flag:
            return bhv_flag
        while(True):
            gosignal.draw()
            win.flip()
            buttons = mouse.getPressed()
            if buttons[0] == 0: # hold target时中途离手,默认不离手
                break
        
        #================================================
        #| wait before reward
        #================================================                    
        start_time=time.time()
        while time.time()-start_time <wait_before_reward_frame_num:
                win.flip()
        bhv_flag = True
        return bhv_flag

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
win.mouseVisible = True
        
neworder = list(range(16)) #手动脑控各8个方向，一共16个condition
random.shuffle(neworder)
round_index=0 #每4轮来一次全清，不管之前做错的有没有再做对。
past_trialnum=0
# all_type=[0,0,0,0,0,0]
error=[]
all_type=[0,0,0,0,0,0,0,0,
          0,0,0,0,0,0,0,0]
# degree = degree[neworder[0 % 16]]
trialnum_go=0
trialnum_go_success=0
trialnum_nogo=0
trialnum_nogo_success=0

for trialnum in range(28):

    win.flip()    
    core.wait(1)   
    
####################### 生成新的condition   
    if (trialnum > past_trialnum)  & ((trialnum-past_trialnum ) % len(neworder) == 0):  #新的一轮             
        
        past_trialnum=past_trialnum+len(neworder)
        neworder = list(range(16))
        neworder.extend(error)
        random.shuffle(neworder)
        error=[]
        round_index=round_index+1
        if round_index == 3: #每一轮的错误最多重复3次，然后重新清零
           round_index=0
           neworder = list(range(16)) #手动脑控各8个方向，一共16个condition
           random.shuffle(neworder)
           error=[]       
    
    index = neworder[(trialnum-past_trialnum) % len(neworder)]
    initial_angle = all_degree[index]
    init_pos = target_rand_pos(radius,initial_angle)  
    # if trialnum % 16 > 8:
    if index < 8:
        hand_flag= True            
        target = target_go
        target_go.pos= init_pos
        trialnum_go=trialnum_go+1
        
    else:
        hand_flag= False
        target = target_nogo
        target_nogo.pos= init_pos
        trialnum_nogo=trialnum_nogo+1   
    
    target_radius.pos = init_pos
    init_pos = list(init_pos) # for json dump
    
    delay_frame_num=delay_frame_num_range[0]+(delay_frame_num_range[1]-delay_frame_num_range[0])*random.random()
    ############################### 打印信息
    print(neworder)    
    ##################新的trial开始
    bhv_flag = GO_NO_GO()
    ###################整理结果并打印
    if bhv_flag & hand_flag :
        trialnum_go_success=trialnum_go_success+1  
        all_type[index]=all_type[index]+1
    elif bhv_flag & (not hand_flag):
        trialnum_nogo_success=trialnum_nogo_success+1
        all_type[index]=all_type[index]+1
    elif bhv_flag & hand_flag:
        error.append(index)
    else:
        error.append(index)
    print(all_type)
    if (trialnum_go != 0) & (trialnum_nogo != 0):
        print(trialnum_go, trialnum_go_success,trialnum_go_success/trialnum_go,trialnum_nogo, trialnum_nogo_success,trialnum_nogo_success/trialnum_nogo)