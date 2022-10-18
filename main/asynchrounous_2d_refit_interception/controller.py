from multiprocessing import shared_memory

assist_coefficient = shared_memory.ShareableList(name='AssistCoefficient')
decoder_coefficient = shared_memory.ShareableList(name='DecoderCoefficient')
training_flag = shared_memory.ShareableList(name='TrainingFlag')

