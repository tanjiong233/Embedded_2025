#!/usr/bin/env python3
"""
CMake编译文件清理脚本
用于删除所有CMake生成的编译文件和文件夹
"""

import os
import shutil
import sys
from pathlib import Path

# 定义要删除的文件夹名称
FOLDERS_TO_DELETE = [
    'CMakeFiles',
    'Testing',
    'cmake-build-debug',
    'cmake-build-release',
    'cmake-build-debug-mingw_stm32',
    'build',
    '_build',
    'out',
]

# 定义要删除的文件名（完全匹配）
FILES_TO_DELETE = [
    'cmake_install.cmake',
    'CMakeCache.txt',
    'Makefile',  # CMake生成的Makefile
    'compile_commands.json',
    'build.ninja',
    'rules.ninja',
    '.ninja_deps',
    '.ninja_log',
]

# 定义要删除的文件扩展名
FILE_EXTENSIONS_TO_DELETE = [
    '.o',
    '.obj',
    '.a',
    '.so',
    '.dll',
    '.lib',
    '.exe',
    '.elf',
    '.bin',
    '.hex',
    '.map',
    '.lst',
]

# 定义要删除的文件模式（包含这些字符串的文件夹）
FOLDER_PATTERNS_TO_DELETE = [
    '.dir',  # 例如 example_led_typeC.elf.dir
    'cmake-build-',  # 任何以cmake-build-开头的文件夹
]


def should_delete_folder(folder_name):
    """判断文件夹是否应该被删除"""
    # 完全匹配
    if folder_name in FOLDERS_TO_DELETE:
        return True

    # 模式匹配
    for pattern in FOLDER_PATTERNS_TO_DELETE:
        if pattern in folder_name:
            return True

    return False


def should_delete_file(file_name):
    """判断文件是否应该被删除"""
    # 完全匹配
    if file_name in FILES_TO_DELETE:
        return True

    # 扩展名匹配
    for ext in FILE_EXTENSIONS_TO_DELETE:
        if file_name.endswith(ext):
            return True

    return False


def clean_cmake_files(root_dir='.'):
    """清理指定目录下的所有CMake编译文件"""
    root_path = Path(root_dir).resolve()
    deleted_items = []

    print(f"开始清理目录: {root_path}")
    print("-" * 50)

    # 遍历目录树
    for current_dir, dirs, files in os.walk(root_path, topdown=True):
        current_path = Path(current_dir)

        # 删除文件
        for file_name in files[:]:  # 使用切片创建副本，避免在迭代时修改
            if should_delete_file(file_name):
                file_path = current_path / file_name
                try:
                    file_path.unlink()
                    deleted_items.append(f"文件: {file_path.relative_to(root_path)}")
                    print(f"删除文件: {file_path.relative_to(root_path)}")
                except Exception as e:
                    print(f"错误: 无法删除文件 {file_path}: {e}")

        # 删除文件夹（从dirs列表中移除，这样os.walk不会进入这些文件夹）
        dirs_to_remove = []
        for dir_name in dirs[:]:
            if should_delete_folder(dir_name):
                dir_path = current_path / dir_name
                try:
                    shutil.rmtree(dir_path)
                    deleted_items.append(f"文件夹: {dir_path.relative_to(root_path)}")
                    print(f"删除文件夹: {dir_path.relative_to(root_path)}")
                    dirs_to_remove.append(dir_name)
                except Exception as e:
                    print(f"错误: 无法删除文件夹 {dir_path}: {e}")

        # 从dirs列表中移除已删除的文件夹，防止os.walk进入
        for dir_name in dirs_to_remove:
            dirs.remove(dir_name)

    print("-" * 50)
    print(f"清理完成！共删除 {len(deleted_items)} 个项目")

    return deleted_items


def main():
    """主函数"""
    # 获取要清理的目录
    if len(sys.argv) > 1:
        target_dir = sys.argv[1]
    else:
        target_dir = '.'

    # 确认操作
    target_path = Path(target_dir).resolve()
    print(f"准备清理目录: {target_path}")
    print("这将删除所有CMake生成的编译文件和文件夹！")

    response = input("确定要继续吗？(y/N): ").strip().lower()
    if response != 'y':
        print("操作已取消")
        return

    # 执行清理
    try:
        deleted_items = clean_cmake_files(target_dir)

        # 可选：将删除的项目列表保存到文件
        log_file = target_path / 'cmake_clean_log.txt'
        with open(log_file, 'w', encoding='utf-8') as f:
            f.write(f"CMake清理日志\n")
            f.write(f"清理时间: {Path.cwd()}\n")
            f.write(f"清理目录: {target_path}\n")
            f.write(f"删除项目数: {len(deleted_items)}\n")
            f.write("-" * 50 + "\n")
            for item in deleted_items:
                f.write(f"{item}\n")

        print(f"\n清理日志已保存到: {log_file}")

    except KeyboardInterrupt:
        print("\n操作被用户中断")
    except Exception as e:
        print(f"\n发生错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()