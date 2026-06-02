#!/usr/bin/env python3
"""
一个简单的 Python 演示脚本
"""
import sys
import platform
from datetime import datetime

def main():
    print("=" * 50)
    print("Hello from Python!")
    print("=" * 50)

    # 系统信息
    print(f"\n📋 系统信息:")
    print(f"   Python 版本: {sys.version}")
    print(f"   操作系统: {platform.system()} {platform.release()}")
    print(f"   架构: {platform.machine()}")

    # 当前时间
    print(f"\n🕐 当前时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    # 斐波那契数列示例
    print(f"\n📐 前10个斐波那契数:")
    a, b = 0, 1
    fib = []
    for _ in range(10):
        fib.append(a)
        a, b = b, a + b
    print(f"   {fib}")

    # 列表推导式示例
    squares = [x**2 for x in range(1, 11)]
    print(f"\n🔢 1到10的平方:")
    print(f"   {squares}")

    print(f"\n✅ 脚本运行完成!")

if __name__ == "__main__":
    main()
