# Changelog

All notable changes to the LiDAR-Camera Calibration Validator project will be documented in this file.

---

## [3.1.0] - 2025-09-12

### Added

- **高精度投影计算**: 完整的畸变校正支持
- **定量评价指标**: 边缘重叠度和归一化互信息计算
- **并行处理**: 大点云的高性能并行投影处理
- **动态参数调节**: 实时参数调整，无需重启节点
- **边缘检测分析**: Canny边缘检测和重叠分析
- **性能监控**: 实时FPS和处理时间统计
- **可视化增强**: 深度颜色条移至右上角，点大小默认为3
- **完整文档**: 中英双语README和详细的使用指南

### Changed

- **代码重构**: 去除冗余注释，代码结构更加简洁
- **项目统一**: 所有文件名称统一为"LiDAR-Camera Calibration Validator"
- **配置优化**: 默认禁用降采样，最大点数设为100万
- **许可证更新**: 采用GPL-3.0-or-later许可证

### Fixed

- **参数传递**: 修复dynamic_reconfigure参数传递问题
- **线程安全**: 解决dynamic_reconfigure mutex警告
- **深度阈值**: 统一深度阈值为0.01f，提高一致性
- **投影精度**: 改进畸变校正算法，提高投影精度

### Performance

- **处理速度**: 优化点云处理，支持高速实时处理
- **内存优化**: 预分配缓存，减少内存分配开销
- **并行计算**: 大点云自动启用并行处理

### Documentation

- **完整指南**: 详细的安装、配置和使用指南
- **故障排除**: 常见问题和解决方案
- **最佳实践**: 标定质量评估的最佳实践指南
- **多语言**: 中英双语文档支持

## [3.0.0] - 2025-09-08

### Added

- **初始发布**: 基础的LiDAR-Camera标定验证功能
- **基础指标**: 投影比例和基础统计信息
- **可视化**: 点云投影到图像的基础可视化
- **配置系统**: 基础的参数配置支持

### Features

- **点云投影**: 基础的3D到2D投影计算
- **实时显示**: OpenCV窗口实时显示融合结果
- **ROS集成**: 完整的ROS Noetic支持
- **话题订阅**: 图像和点云话题同步订阅

---

## 许可证 | License

本项目采用 GPL-3.0-or-later 许可证 - 详见 [LICENSE](LICENSE) 文件。

This project is licensed under the GPL-3.0-or-later License - see the [LICENSE](LICENSE) file for details.
