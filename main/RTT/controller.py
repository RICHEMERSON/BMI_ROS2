from multiprocessing import shared_memory

assist_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
decoder_coefficient = shared_memory.ShareableList(name='DecoderCoefficient')
training_flag = shared_memory.ShareableList(name='TrainingFlag')
moving_target_flag = shared_memory.ShareableList(name='MovingTargetFlag')
# ff_coefficient = shared_memory.ShareableList(name='FFCoefficient')
# PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    
GenerationSpeed = shared_memory.ShareableList(name='GenerationSpeed')
AssistCoefficient1 = shared_memory.ShareableList(name='AssistCoefficient1')
AssistCoefficient2 = shared_memory.ShareableList(name='AssistCoefficient2')
ring_flag = shared_memory.ShareableList(name='RingFlag')
training_flag = shared_memory.ShareableList(name='TrainingFlag')

DecoderSel = shared_memory.ShareableList(name='DecoderSel')

# ros2 run state_reader psychopy_2D_center_out_continuous 2>/share/$(date +%Y%m%d)_$(date +%H%M%S)_behavior.log

ros2 run state_reader psychopy_2D_center_out_continuous 2>/home/soma/data/bmi_data/leb/$(date +%Y%m%d)_$(date +%H%M%S)_behavior.log

ros2 launch /home/soma/Desktop/BMI/main/RTT/launch.py>/home/soma/data/bmi_data/leb/$(date +%Y%m%d)_$(date +%H%M%S)_decoding_element.log

ros2 launch /home/soma/Desktop/BMI/main/RTT/observation_blackrock_launch.py > /home/soma/data/bmi_data/leb/$(date +%Y%m%d)_$(date +%H%M%S)_observation.log