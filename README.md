# pseudo-LiDAR-point-cloud-augmentation
3D object detection in road scene by pseudo-LiDAR point cloud augmentation


## 点云补全实验

### KITTI数据集裁剪生成

1、计算包围框顶点坐标、并生成对应的bin点云文件：[generate_bin.py](https://github.com/jinshuai224/pseudo-LiDAR-point-cloud-augmentation/blob/main/generate_bin.py)、[kitti_util.py](https://github.com/jinshuai224/pseudo-LiDAR-point-cloud-augmentation/blob/main/kitti_util.py)

2、bin文件转换为pcd文件：[bin2pcd](https://github.com/jinshuai224/pseudo-LiDAR-point-cloud-augmentation/tree/main/bin2pcd)

3、自制的KITTI点云补全数据集（前100个）：https://drive.google.com/file/d/1ylXaLjJQiMvsm6TSVRnnZkoK7qTkdLk8/view?usp=sharing

4、利用PoinTr算法模型的预测结果可视化图片及点云npy文件：https://drive.google.com/file/d/1yg9545Dacj4II1ZEBmB1U_9dMcCn1NIA/view?usp=sharing





## 伪点云

1、伪点云数据：https://drive.google.com/file/d/1oQ02CkoRDMR9jGGgsfTa9gk4eLWwc904/view?usp=sharing

2、伪点云与原始点云合并代码：[new_bin.py](https://github.com/jinshuai224/pseudo-LiDAR-point-cloud-augmentation/blob/main/new_bin.py)


