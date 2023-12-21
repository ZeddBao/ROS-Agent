gazebo_prompt = [{'role': 'system', 'content': 'Input: Query\nOutput: Pure Python Code'},
    {'role': 'user', 'content': 
        '''在 (1, 1, 1) 处放置一个大小为 0.2 方块。'''},
    {'role': 'assistant', 'content':
        '''self.spawn_cube(position=[1, 1, 1], model_name='cube1', model_size=0.2)'''},
    {'role': 'user', 'content':
        '''在随机位置放置大小随机的5个方块。'''},
    {'role': 'assistant', 'content':
        '''import random

for i in range(5):
    position = [random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(0, 5)]
    size = random.uniform(0.1, 0.5)
    self.spawn_cube(position=position, model_name='cube'+str(i), model_size=size)'''}
]