# 7. Path Tracing

> 作业步骤：
> - 查看[文档](documents/README.md)，内含多个小教程，请先阅读 [documents/README.md](documents/README.md)，其中包含了所有文档的阅读引导
> - 在[项目目录](../../Framework3D/)中编写作业代码
> - 按照[作业规范](../README.md)提交作业

## 作业递交

- 递交内容：程序代码（hd_USTC_CG下全部文件）及实验报告，见[提交文件格式](#提交文件格式)
- 递交时间：2024年4月21日（周日）晚

## 要求

- 实现直接光照积分器
  - 对矩形光源进行重要性采样
- 路径追踪算法
  - Russian Roullete
- (Optional) 添加一种材质，对材质进行重要性采样，并进行多重重要性采样，与单种采样的结果进行比较
- (Optional) 透明材质

## 目的

- 熟悉渲染过程中多个概率空间的转换
- 熟悉路径追踪算法
- 了解多重重要性采样


## 提供的材料

- 对球形光源采样的参考
- 场景相交的数据结构和类型
- 测试场景
依照上述要求和方法，根据说明文档`(1) documents`和作业框架`(2) Framework3D`的内容进行练习。

### (1) 说明文档 `documents` [->](documents/) 

本次作业的要求说明和一些辅助资料

### (2) 作业项目 `Framework3D` [->](../../Framework3D/) 

作业的基础代码框架和测试数据。

测试数据链接：链接：https://rec.ustc.edu.cn/share/18163800-fe1c-11ee-b6af-f9c116738547

## 提交文件格式

文件命名为 `ID_姓名_Homework*.rar/zip`，其中包含：

```
ID_姓名_Homework*/
├── hd_USTC_CG/                        // 渲染器全部代码
│   ├── xxx.h
│   ├── xxx.cpp
|   └── ...
├── report.pdf                    // 实验报告
└── ...                           // 其他补充文件

```

### 注意事项

- 导入数据（网格和纹理）时使用相对路径，将你的数据放在可执行文件目录下，直接通过 `FilePath = 'xxx.usda'` 或者 `FilePath = 'zzz.png'` 访问；