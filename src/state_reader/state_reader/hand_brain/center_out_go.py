# -*- coding: utf-8 -*-
"""
Created on Mon Sep  2 18:15:12 2024

@author: polar
"""

def GO_NO_GO():
    delay_frame_num=delay_frame_num_range[0]+(delay_frame_num_range[1]-delay_frame_num_range[0])*random.random()
    trial_info(info)
    touch_center_flag = False
    touch_target_flag = False
    hold_feedback_flag = True
    init_pos, initial_angle = target_rand_pos(radius,degree)
    target.pos = init_pos
    target_radius.pos = init_pos
    init_pos = list(init_pos) # for json dump

    #================================================
    #| center on
    #================================================
    #print('%%%%%%%%%%%%%%%%%%%%')
    #print("trial start",trialnum_go)
    start_time=time.time()
    while time.time()-start_time < acquire_frame_num:
        center.draw()
        win.flip()
        win_flip_info([center],fn)

        if mouse.isPressedIn(fullscr_area, buttons=[0]):
            touch_center_flag = True
            marker_publisher({"Event":"Center touched","Marker": 2})           
            break

    #================================================
    #| hold center
    #================================================
    if not touch_center_flag:
        go_type[3]=go_type[3]+1
        go_type[1]=go_type[2]/go_type[0]
        print(go_type)
        return go_type,trial_laststep_go,viconflag
    else:
        hold_center_flag = True
        start_time=time.time()
        while time.time()-start_time < holdcenter_frame_num:
            center.draw()
            win.update()
            win_flip_info([center],fn)
            if not mouse.isPressedIn(fullscr_area, buttons=[0]):
                hold_center_flag = False
                break

        if not hold_center_flag:
            go_type[4]=go_type[4]+1
            go_type[1]=go_type[2]/go_type[0]
            print(go_type)
            return go_type,trial_laststep_go,viconflag

        if not mouse.isPressedIn(center_radius, buttons=[0]):
            pos = mouse.getPos()
            go_type[5]=go_type[5]+1
            go_type[1]=go_type[2]/go_type[0]
            print(go_type)
            return go_type,trial_laststep_go,viconflag
    #================================================
    #| target on (need to change color)
    #================================================
    ##################### 
    start_time=time.time()
    while time.time()-start_time < delay_frame_num:
        center.draw()
        target.draw()
        win.update()
        win_flip_info([center, target],fn)
        exp_quit(win,go_type[0], go_type[2])

        if not mouse.isPressedIn(center_radius, buttons=[0]):
            exp_quit(win,go_type[0], go_type[2])
            hold_center_flag = False
            break

    if not hold_center_flag:
        continue
        # wrong

    #================================================
    #| GO signal on
    #================================================
    start_time=time.time()
    while time.time()-start_time <RTandMT_frame_num:

        gosignal.draw()
        target.draw()
        win.update()
        win_flip_info([gosignal, target],fn)
        exp_quit(win,go_type[0], go_type[2])

        buttons = mouse.getPressed()
        if buttons[0] == 0:         
            RT_frame_num = time.time()-start_time
            hold_center_flag=False
            break
    if hold_center_flag:
        go_type[7]=go_type[7]+1
        go_type[1]=go_type[2]/go_type[0]
        print(go_type)
        return go_type,trial_laststep_go,viconflag

    #================================================
    #| Go no Go choice
    #================================================


    #================================================
    #| Movement onset
    #================================================
    start_time=time.time()
    while time.time()-start_time < (RTandMT_frame_num - RT_frame_num):
        gosignal.draw()
        target.draw()
        win.update()
        win_flip_info([gosignal, target],fn)
        buttons = mouse.getPressed()
        if buttons[0] == 1:
            eventmarker(6)
            touch_target_flag = True
            MT_frame_num = frameN            
            break
    #================================================
    #| Touch target
    #================================================
    if not touch_target_flag:
        go_type[8]=go_type[8]+1
        go_type[1]=go_type[2]/go_type[0]
        print(go_type)
        return go_type,trial_laststep_go,viconflag
    else:
        #================================================
        #| Feedback on
        #================================================
        hold_target_flag = True
        trial_laststep_go=trial_laststep_go+1
        if not mouse.isPressedIn(target_radius,buttons=[0]): # 触摸不在容错范围内
            wrong_feedback.pos= mouse.getPos()
            go_type[9]=go_type[9]+1
            go_type[1]=go_type[2]/go_type[0]
            print(go_type)
            return go_type,trial_laststep_go,viconflag                
        
        else:# 触摸在容错范围内               
            right_feedback.pos= mouse.getPos()    
            start_time=time.time()
            while time.time()-start_time < feedback_on_frame_num:
                    gosignal.draw()
                    target.draw() 
                    right_feedback.draw()
                    win.update()
                    win_flip_info([gosignal, target,right_feedback],fn)
                    exp_quit(win,go_type[0], go_type[2])
                    buttons = mouse.getPressed()
                    if buttons[0] == 0: # hold target时中途离手
                        hold_target_flag=False
                        break
            if not hold_target_flag:
                go_type[10]=go_type[10]+1
                go_type[1]=go_type[2]/go_type[0]
                print(go_type)
                return go_type,trial_laststep_go,viconflag
            else:
                trial_success_go=trial_success_go+1
                while(True):
                    gosignal.draw()
                    win.update()
                    win_flip_info([gosignal],fn)
                    buttons = mouse.getPressed()
                    if buttons[0] == 0: # hold target时中途离手,默认不离手
                        break
         
    
           
 
  