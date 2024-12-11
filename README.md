# Introduction


# Develop Log
### 2024-4-27
考虑到目前（2024-4-27）c++20 modules的支持还并不是很成熟，因此目前使用宏ENMODULES来控制是否引入modules

### 2024-6-7
暂时抛弃modules支持，全面换用headers，准备基于rtklib开发gnss/ins组合导航程序

### 2024-6-8
为了方便程序开发，增加了如下基础库：

##### 基础库
1. [doctest](https://github.com/doctest/doctest)  单元测试 [doc](https://github.com/doctest/doctest/blob/master/doc/markdown/tutorial.md)
2. [matplotlib-cpp](https://github.com/lava/matplotlib-cpp) - matplotlib的cpp接口，便于作图 [doc](https://github.com/lava/matplotlib-cpp)
3. [implot](https://github.com/epezent/implot) 依赖于imgui的实时绘图，目前尚未测试通过
4. [reflect-cpp](https://github.com/getml/reflect-cpp) c++ 反射库 [doc](https://github.com/getml/reflect-cpp?tab=readme-ov-file#simple-example)
5. [magic_enum](https://github.com/Neargye/magic_enum) c++ 智能enum [doc](https://github.com/Neargye/magic_enum/blob/master/doc/reference.md)
6. [exprTk](https://github.com/ArashPartow/exprtk) c++ 数学表达式识别运算库 [doc](https://github.com/ArashPartow/exprtk/blob/master/readme.txt)
7. [plotly](https://github.com/plotly/plotly.py)  python可交互式作图程序，生成html [doc](https://plotly.com/python/)
8. [dash](https://github.com/plotly/dash) 基于plotly的作图程序开发库 [doc](https://dash.plotly.com/layout)
9. [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) c++矩阵运算库 [doc](https://github.com/qixianyu-buaa/EigenChineseDocument?tab=readme-ov-file)
10. [argparse](https://github.com/p-ranav/argparse) 命令行参数解析库 [doc](https://github.com/p-ranav/argparse/blob/master/README.md)
11. [proj](https://github.com/OSGeo/PROJ?tab=readme-ov-file) 坐标转换、投影库 [doc](https://proj.org/en/9.4/download.html)


#### GNSS库
- gnsstk 基础gnss库(无详细注解)

[GNSS算法相关开源代码（含多传感器融合相关项目）](https://blog.csdn.net/dong20081991/article/details/128487851)

#### 因子图优化库
1. [GTSAM](https://github.com/borglab/gtsam?tab=readme-ov-file)(Graph-based SLAM and optimization)
GTSAM 是一个用于处理大规模非线性优化和图优化的库，适用于机器人领域的 SLAM（Simultaneous Localization and Mapping）和视觉/惯性传感器融合等应用。它提供了一套灵活的接口和算法，支持 GNSS 和 INS 数据的处理和融合。 


### 2024-6-9
#### 绘图库更改为matplot++


### 2024-6-10
#### 决定将此库的开发目标定为rtk-ins库，主要研究rtk和rtk与INS的松紧组合
#### 完成了ceres-solver库的增加


### 2024-6-12
#### 学习ceres-solver库
#### 目前clang编译器并不支持expected，暂时使用gcc

### 2024-6-13
#### 准备开发rinex4.0读取功能
#### 开发option库

### 2024-6-14
#### 开发option库，准备支持reference，目前reference的unwarp_or()函数存在问题，可能需要重载或者重构
#### option库的功能尚未开发完全

### 2024-6-16
#### 目前存在多次释放内存问题，可能于构造函数相关


### 2024-7-3
#### focus on what you what to do

### 2024-7-9
#### 目前加入类型floatxx_t，但是clangd无法识别，但不影响代码实际效果

### 2024-7-10
#### 仿照std::chrono::gps_clock写了一个bds_clock，但是目前其formatter未写好
#### 准备使用g2o替代ceres-solvers来实现因子图


### 2024-11-2
#### QCustomPlot [教程](https://lancelot-yagami.github.io/QCustomPlot-Manual/#/README)

### 2024-11-12
#### 精密星历解算，pcv,pco,dcb改正文件读取及改正
#### 因子图优化引入


### 2024-12-11
#### 重构代码：(1) 重构GTime (2) 针对非POD类型，全面禁用拷贝构造及赋值函数，使用clone函数替代