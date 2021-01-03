
#### 1.这个项目是干嘛的
我正在知乎上写一个从零开始做自动驾驶定位的系列博客，这个工程就是博客的配套源代码。  
博客专栏地址：https://zhuanlan.zhihu.com/c_1114864226103037952

#### 2.这些代码怎么用
每篇博客更新后，如果配套有代码更新，我会在关键节点版本上添加tag，同时会在博客里注明当篇博客对应的的版本tag，这样博客的阅读进度和代码的进度就能够对应上

#### 3.调试环境
- ubuntu 16.04
- ros kinectic
- pcl 1.7
- glog

这是我个人的调试环境，也没在别的环境上测试过，所以前期可能会暴露不少问题，如果遇到，各位可以在本工程里提issue，也可以在对应的博客下面的评论区留言。

#### 4.测试数据
开源程序最好使用开放数据集，所以我们选择了KITTI，并且把RawData里的"2011_10_03_drive_0027_sync"做成了bag文件，后面所有程序的测试都是在这个bag基础上做的。    
数据文件我放在了百度网盘里   
地址：https://pan.baidu.com/s/1TyXbifoTHubu3zt4jZ90Wg   
提取码: n9ys

#### 5.Fork from 任乾工程师的repo：https://link.zhihu.com/?target=https%3A//github.com/Little-Potato-1990/localization_in_auto_driving
#### 此项目仅仅是对任乾工程师的repo进行学习，写了一些注释
