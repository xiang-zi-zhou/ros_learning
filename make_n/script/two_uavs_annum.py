#!/usr/bin/env python3

# 从键盘输入一个整数并赋值给变量

# 使用input()函数获取用户输入，并用int()转换为整数类型
try:
    # 提示用户输入整数
    user_input = input("请输入一个整数: ")
    # 将输入转换为整数并赋值给变量
    number = int(user_input)
    
    # 显示结果
    print(f"你输入的整数是: {number}")
    print(f"变量number的值已被设置为: {number}")
    
except ValueError:
    # 处理输入无法转换为整数的情况
    print("错误: 你输入的不是一个有效的整数，请重试。")
    