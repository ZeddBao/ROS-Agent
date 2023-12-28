# 基于大语言模型的智能机器人控制系统

## 项目简介

### Agent 框架

本框架由 Python 实现，以 Agent 概念为核心。Agent 概念给了大语言模型一个“具身的实体”，允许其通过编写、执行代码感知环境、操作环境。这个环境可以是计算机的终端环境，也可以是机器人的仿真环境，甚至可以是现实世界的环境。这样的框架让 AI 不再像一个函数或者一个机器只能接收输入然后输出结果，而是可以不断和环境交互，感知环境然后做出行动。

首先实现了一个基类 [`class AgentBase`](./Agents.py)，这个类具有大语言模型 Agent 的公用方法：

- `get_code_response()`：调用 API 从大语言模型服务端获得响应，得到大语言模型的输出。在本框架中输出主要为可执行代码。
- `execute()`：递归执行代码，递归的含义是当执行代码失败以后，会将报错信息和可执行代码重新发送给大语言模型，得到调试后的新的代码。当递归达到最大次数 `retry_max` 会停止递归。
- `init_node()`：初始化 ROS 节点或服务。
- 打印、记录日志，内容主要为大语言模型的输出和执行代码的结果等等。

通过继承基类，子类可以根据机器人不同功能新增特定的方法，例如机械臂可以添加移动关节的方法，四租机器人可以前进、后退的方法，仿真器可以添加放置物体的方法等等。并且每个类别的机器人都有不同的内置初始 prompt，这些 prompt 会给出一些代码案例供大语言模型进行 few-shot 的学习。

### 记忆系统

Agent 框架还包括了记忆存储功能，可以分为短期记忆和长期记忆。

#### 短期记忆

短期记忆主要与大语言模型内置的初始 [prompt](./code_prompt.py) 有关，通过将对话信息不断加入 prompt 列表，即可添加 Agent 的短期记忆。由于大语言模型会将所有的 prompt 发送至服务端，并且这些文字内容都是需要消耗 token 的，因此这种方法的性价比很低。

#### 长期记忆

长期记忆利用了 Embedding 的技术，Embedding 是一种将类别数据（特别是单词或短语）转换为实数向量的技术。这些向量捕捉到了输入数据的特征，如单词的语义和语法属性，以及它们之间的相似性。通过 Embedding 将历史对话，或者机器人编程案例转换为向量数据库（即长期记忆），在向大语言模型发送请求的时候可以先在向量数据库中寻找相关的文档或句子，将其一并发送给大语言模型，大语言模型就能依据所给储存的文档或句子进行回答，就可以模拟长期记忆的效果。

## 项目视频展示

<video width="480" height="360" controls>
  <source src="./doc/基于大语言模型的智能机器人控制系统.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

[基于大语言模型的智能机器人控制系统展示视频](./doc/基于大语言模型的智能机器人控制系统.mp4)


## 环境配置

### `ROS` 安装

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

借助 ROS 社区鱼香肉丝大佬的脚本安装ROS，按照提示安装 `Noetic` 版本即可。

### `MoveIt` 安装

```bash
sudo apt-get install ros-noetic-moveit
```

### MoveIt 相关控制器安装

```bash
sudo apt-get install ros-noetic-*-controller
```

## 创建 ROS 工作空间

创建一个 ROS 工作空间

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

将 `jibot3` 和 `jibot3_moveit_config` 包放入 `~/catkin_ws/src` 目录下

```bash
cd ~/catkin_ws
catkin_make
```
### 安装其他依赖

```bash
pip3 install openai setuptools
```

## 运行机械臂控制节点

1. 首先 source ROS 环境

    ```bash
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    ```

2. 启动 `roscore`

    ```bash
    roscore
    ```

3. 启动 `jibot3` 机械臂仿真环境和 `MoveIt` 控制器

    ```bash
    roslaunch jibot3_moveit_config demo.launch
    ```

4. 以 `jibot3` 机械臂节点为例

    ```bash
    cd path/to/ROS-Agent
    python3 jibot3_node.py
    ```

5. 输入对机械臂的命令，可以参考 [jibot3_prompt.py](./jibot3_prompt.py) 中的内置 prompt

    ```python
    >>> 顺时针旋转第五个关节 0.2 radians。
    ```

## 其他节点

### `speech_node.py`

该节点用于语音识别，将语音转换为文字，然后发送给 Agent 节点。

```bash
cd path/to/ROS-Agent
python3 speech_node.py
```

另外打开一个终端

```bash
cd path/to/ROS-Agent
python3 Agents.py
```

进行对话语音识别控制测试。注意要使用 agent 的 `listen()` 才能订阅到语音识别的结果。
