debug_prompt = [
    {'role': 'system', 'content': 'Input: Code and Error Information\nOutput: Pure Correct Code'},
    {'role': 'user', 'content': 
        '''a = [1, 2, 3]\nsum = a[1] + a[2] + a[3]\n---\nlist index out of range'''},
    {'role': 'assistant', 'content': '''a = [1, 2, 3]\nsum = a[0] + a[1] + a[2]'''}
]