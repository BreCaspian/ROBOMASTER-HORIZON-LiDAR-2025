# ByteTracker 多目标跟踪模块

基于 ByteTrack 算法的多目标跟踪模块，专为 ROBOMASTER-2025 华北理工大学 HORIZON 战队 雷达站设计。

## 功能特性

- 高精度多目标跟踪  
- 实时性能优化  
- 稳定的 ID 管理  
- 集成卡尔曼滤波预测  
- 可调参数适配不同场景  

## 模块结构

```text
ByteTracker/
├── include/
│   ├── ByteTrack/           # 核心算法
│   │   ├── BYTETracker.h
│   │   ├── STrack.h
│   │   ├── KalmanFilter.h
│   │   ├── Object.h
│   │   └── lapjv.h
│   └── ByteTrackerWrapper.h # 系统接口封装
├── src/
│   └── ByteTrackerWrapper.cpp
└── CMakeLists.txt
```

## 快速使用

### 初始化

```cpp
ByteTrackerWrapper::Ptr tracker = std::make_shared<ByteTrackerWrapper>();
tracker->init(30, 30, 0.5f, 0.6f, 0.8f);
```

### 执行跟踪

```cpp
vector<bboxAndRect> detections = ...;
vector<bboxAndRect> tracked = tracker->track(detections);
```

### 获取统计信息

```cpp
ByteTrackerStats stats = tracker->getStats();
```

## 配置参数

| 参数            | 默认值 | 说明       |
| --------------- | ------ | ---------- |
| frame_rate      | 30     | 帧率       |
| track_buffer    | 30     | 轨迹保留帧数 |
| track_thresh    | 0.5    | 跟踪阈值   |
| high_thresh     | 0.6    | 高置信度阈值 |
| match_thresh    | 0.8    | IOU 匹配阈值 |

### 高召回场景

```yaml
track_thresh: 0.3
high_thresh: 0.5
match_thresh: 0.6
track_buffer: 40
```

### 高精度场景

```yaml
track_thresh: 0.6
high_thresh: 0.75
match_thresh: 0.8
track_buffer: 20
```

## 算法原理

1. 检测结果按置信度分组  
2. 高置信度检测与轨迹匹配  
3. 卡尔曼滤波预测轨迹  
4. 低置信度检测二次匹配  
5. 管理轨迹生命周期  

## 集成说明

- 输入：检测器输出 → ByteTracker  
- 输出：带 ID 的目标结果  
- 支持多线程与自动内存管理  
- 配置文件示例：  

```yaml
bytetracker:
  enable: true
  frame_rate: 30
  track_buffer: 30
  track_thresh: 0.5
  high_thresh: 0.6
  match_thresh: 0.8
```

## 注意事项

1. 参数需结合场景调优  
2. 长时间运行需注意内存占用  
3. 确保检测与跟踪坐标系一致  
4. 定期检查跟踪统计信息  

## 参考资料

- [ByteTrack 论文](https://arxiv.org/abs/2110.06864)  
- [官方实现](https://github.com/ifzhang/ByteTrack)  

---

> 建议先用默认参数，再根据实际需求调优。
