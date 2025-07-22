# 基于isaaclab的基础上进行自我修改

1. 增加一个Injection multiple CL训练的目录
2. 不使用官方的usd，而是使用自己的机械臂模型。

## 使用自己机械臂模型
1. 首先在sim中导入urdf文件，然后保存为usd文件
2. 不破坏isaaclab的框架结构，所以在assets.py中增加自定义路径USER_BASE_DIR，路径为上述自定义的usd资产存放路径。
