from multiprocessing import shared_memory

assist_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
decoder_coefficient = shared_memory.ShareableList(name='DecoderCoefficient')
training_flag = shared_memory.ShareableList(name='TrainingFlag')
moving_target_flag = shared_memory.ShareableList(name='MovingTargetFlag')
ff_coefficient = shared_memory.ShareableList(name='FFCoefficient')
PosFlag = shared_memory.ShareableList([1],name='PosFlag')
    
GenerationSpeed = shared_memory.ShareableList(name='GenerationSpeed')
AssistCoefficient1 = shared_memory.ShareableList(name='AssistCoefficient1')
AssistCoefficient2 = shared_memory.ShareableList(name='AssistCoefficient2')
ring_flag = shared_memory.ShareableList(name='RingFlag')
training_flag = shared_memory.ShareableList(name='TrainingFlag')
