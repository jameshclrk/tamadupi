# Tamadupi

[English](README.md)

Tamadupi 是为 Waveshare ESP32-S3-Touch-AMOLED-1.8 制作的传感器驱动虚拟角色。Mochi 会响应运动、附近的 BLE 设备和室外天气。

## 硬件

- ESP32-S3-Touch-AMOLED-1.8 开发板
- USB 数据线

## 构建和烧录

构建前复制本地配置模板：

```bash
cp main/secrets.example.h main/secrets.h
```

在 `main/secrets.h` 中填写固定的 Wi-Fi 凭据和位置。Git 会忽略此文件，其中的值会直接编译进固件。

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

将 `PORT` 替换为开发板的串口。

## 预期结果

AMOLED 屏幕会显示一个动画角色：

- 左右倾斜开发板，Mochi 会跟着倾斜并转动眼睛。
- 左右移动开发板，Mochi 会在屏幕中滑动，撞到边缘时会被挤扁。
- 移动开发板时，Mochi 会弹跳，底部运动指示条也会变化。
- 轻轻摇晃开发板，会触发 Mochi 的惊讶表情。
- 静置时，Mochi 会呼吸和眨眼。

状态标签显示 `IMU live` 表示传感器工作正常。若显示 `no IMU`，请在串口监视器中检查 I2C 或 QMI8658 错误信息。

## 调整

运动阈值、采样率和界面颜色集中定义在 `main/main.c` 顶部，便于快速调整。请勿将真实凭据写入 `secrets.example.h`。
