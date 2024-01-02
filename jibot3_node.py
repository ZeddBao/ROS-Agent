from Agents import Jibot3Agent
from prompt_examples.jibot3_prompt import jibot3_prompt

robot = Jibot3Agent()
# robot.load_prompt(jibot3_prompt)
robot.init_node()
robot.listen()