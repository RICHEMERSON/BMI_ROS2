# -*- coding: utf-8 -*-
"""
Created on Mon Sep  2 18:15:12 2024

@author: polar
"""        

delay_frame_num=int(delay_frame_num_range[0]+(delay_frame_num_range[1]-delay_frame_num_range[0])*random.random())
trial_info(info)
touch_center_flag = False
touch_target_flag = False
hold_feedback_flag = True
init_pos, initial_angle = target_rand_pos(radius,degree)
target.pos = init_pos
right_feedback.pos=init_pos+target.radius*0.5
target_radius.pos = init_pos
init_pos = list(init_pos) # for json dump

#================================================
#| target on
#================================================
eventmarker(3)
marker_publisher({"Event":"Target on","Marker": 3})    
start_time=time.time()
while time.time()-start_time <delay_frame_num:

    center.draw()
    target.draw()
    win.update()
    win_flip_info([center, target],fn)

    if not mouse.isPressedIn(center_radius, buttons=[0]):
        exp_quit(win,nogo_type[0], nogo_type[2])
        hold_center_flag = False
        break

if not hold_center_flag:
    nogo_type[6]=nogo_type[6]+1
    nogo_type[1]=nogo_type[2]/nogo_type[0]
    print(nogo_type)
    return nogo_type,trial_laststep_nogo,viconflag

#================================================
#| go signal on
#================================================
eventmarker(4)
marker_publisher({"Event":"gosignal on","Marker": 4}) 
trial_laststep_nogo=trial_laststep_nogo+1
# print("TTTTTTTTTTTTTTTTTT",trial_laststep_nogo)
start_time=time.time()
while time.time()-start_time <hold_nogo_frame_num:

    gosignal.draw()
    target.draw()
    win.update()
    win_flip_info([gosignal, target],fn)
    buttons = mouse.getPressed()
    if buttons[0] == 0:
        hold_center_flag= False
        break
if not hold_center_flag:
    marker_publisher({"Event":"leave screen","Marker": 9}) 
    eventmarker(9)
    nogo_type[7]=nogo_type[7]+1
    nogo_type[1]=nogo_type[2]/nogo_type[0]
    print(nogo_type)
    return nogo_type,trial_laststep_nogo,viconflag

trial_success_nogo=trial_success_nogo+1
while(True):
    gosignal.draw()
    win.update()
    win_flip_info([gosignal],fn)
    buttons = mouse.getPressed()
    if buttons[0] == 0: # hold target时中途离手,默认不离手
        break
marker_publisher({"Event":"leave screen","Marker": 9}) 
eventmarker(9)

#================================================
#| End
#================================================
        
        
        
        
        
        
   