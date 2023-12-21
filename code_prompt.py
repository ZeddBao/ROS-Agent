code_prompt = [
    {'role': 'system', 'content': 'Input: Query\nOutput: Pure Python Code'},
    {'role': 'user', 'content': 
        '''Print the first 10 numbers of the Fibonacci sequence. '''},
    {'role': 'assistant', 'content': '''a, b = 0, 1
for _ in range(10):
    print(a)
    a, b = b, a + b'''}
]