# 声网 ESP32 视频门铃

*简体中文| [English](README.en.md)*

## 例程简介

本例程演示了如何通过乐鑫 ESP32-S3 Korvo V3 开发板和一个安卓手机，模拟一个典型的视频门铃场景，可以演示按门铃键呼叫手机 APP 端，APP 端接听；或是手机 APP 实时查看门铃端的摄像头画面。

### 文件结构
```
├── CMakeLists.txt
├── components                                  Agora iot sdk component
│   ├── agora_iot_sdk
│   │   ├── CMakeLists.txt
│   │   ├── include                             Agora iot sdk header files
│   │   │   ├── agora_iot_api.h
│   │   │   └── agora_iot_call.h
│   │   └── lib
│   │       ├── esp32
│   │       │   └── PLACEHOLDER
│   │       ├── esp32s2
│   │       │   └── PLACEHOLDER
│   │       └── esp32s3                         Agora iot sdk libraries
│   │           ├── libagora-cjson.a
│   │           ├── libagora-iot-solution.a
│   │           ├── libagora-mbedtls.a
│   │           ├── libagora-webclient.a
│   │           ├── libahpl.a
│   │           ├── libhttp-parser.a
│   │           ├── libiot-license.a
│   │           ├── libiot-utility.a
│   │           ├── libmedia-engine.a
│   │           ├── librtsa.a
│   │           └── PLACEHOLDER
│   └── camera                                  Camera component
├── firmware
│   ├── ag_videodoorbell_esp32.bin
│   ├── bootloader.bin
│   └── partition-table.bin
├── main                                        Video doorbell Demo code
│   ├── app_config.h
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild
│   └── video_doorbell.c
├── partitions.csv                              partition table
├── README.en.md
├── README.md
├── sdkconfig.defaults
└── sdkconfig.defaults.esp32s3
```

## 环境配置

### 硬件要求

本例程目前仅支持`ESP32-S3-Korvo-2`开发板。

## 编译和下载

### 默认 IDF 分支

本例程支持 IDF release/v[4.4] 及以后的分支，例程默认使用 IDF release/v[4.4] 分支。

选择 IDF 分支的方法，如下所示：

```bash
cd $IDF_PATH
git checkout release/v4.4
git pull
git submodule update --init --recursive
```

本例程支持 ADF 最新的 master 分支

### 打上 IDF 补丁

本例程还需给 IDF 合入1个 patch， 合入命令如下：
cd $IDF_PATH
git apply $ADF_PATH/idf_patches/idf_v4.4_freertos.patch


### 编译固件

将本例程（agora-demo-for-esp32）目录拷贝至 ~/esp 目录下。请运行如下命令：
```bash
$ export ADF_PATH=~/esp/esp-adf
$ . $HOME/esp/esp-idf/export.sh
$ cd ~/esp/agora-demo-for-esp32
$ idf.py set-target esp32s3
$ idf.py menuconfig	--> Agora Demo for ESP32 --> (配置 WIFI SSID 和 Password)
$ idf.py build
```

### 下载固件

#### Linux 操作系统

请运行如下命令：
```bash
$ idf.py -p /dev/ttyUSB0 flash monitor
```
注意：遇到 /dev/ttyUSB0 权限问题，请执行 sudo usermod -aG dialout $USER

下载成功后，本例程会自动运行，待初始化完成后，可以看到串口打印："Agora: Press [REC] key to ring the doorbell ..."。此时设备处在 `低功耗保活状态`

#### Windows 操作系统
（TBD）

### 下载预编译的固件

您也可以跳过以上的步骤，直接下载本例程中预编译好的固件。

#### Linux 操作系统

请运行如下命令：

```bash
esptool.py --chip esp32s3 \
--port /dev/ttyUSB0 --baud 921600 \
--before default_reset \
--after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect \
0x0      ./firmware/bootloader.bin \
0x8000   ./firmware/partition-table.bin \
0x10000  ./firmware/ag_videodoorbell_esp32.bin
```
#### Windows 操作系统
（TBD）

## 如何使用例程

### 五分钟快速体验

注意：

1. 请确认开发板上已至少接入了一个扬声器。
2. 在安卓手机上安装好门铃的客户端 APP: [Video Doorbell](https://github.com/AgoraIO-Community/AG-VideoDoorbell-Android/blob/master/release)。

#### Demo 1：门铃呼叫 APP 并进行视频通话

1. 打开安卓手机上的 `Video Doorbell` APP。
2. 选择 `作为移动应用运行` -> 勾选 `双向通话` -> 点击 `确定`，进入注册/登录界面。
3. 在 `请输入账号` 的地方输入 `jack` -> 点击 `登录`。
4. 回到开发板（门铃端），再次确认上述的固件编译、下载步骤已经完成，并看到串口每隔几秒打印出："Agora: Press [REC] key to ring the doorbell ..."
5. 按一下 `[REC]` 按键，对 `jack` 用户进行呼叫。
6. `Video Doorbell` APP 会弹出被呼界面，显示 `有人按门铃`，并等待您的接听。同时您已经可以看到门铃端的实时画面，听到门铃端的声音，但此时无法和门铃端对话。
7. 在 APP 端点击 `接听`，随即就能和门铃端实时对讲了。
8. 在 APP 端点击 `挂断`，结束和门铃端的通话。

说明：在按 [REC] 键呼叫远端 APP 前，门铃处于低功耗保活状态，典型功耗在 800uA 以下。呼叫后门铃自动切换到全功耗模式

#### Demo 2：APP 远程唤醒门铃并进行视频通话

步骤 1 ~ 4 同 Demo 1。

5. 在 `要呼叫的设备账号` 的地方输入 `mydoorbeel` -> 点击 `呼叫`。
6. 随即就能看到门铃端的实时画面了，并且可以和门铃端实时对讲。APP 端出图时间通常在 1 秒左右。

### 开始注册自己的门铃账号

刚才的五分钟快速体验使用的 APP 端的用户账号 (`jack`) 和设备账号 (`mydoorbell`) 都是声网预先注册好的。

接下来我们带您创建属于您自己的用户账号和设备账号。

#### APP 端创建新的用户账号

1. 打开安卓手机上的 `呼叫测试` APP。
2. 选择 `作为移动应用运行` -> 勾选 `双向通话` -> 点击 `确定`，进入注册/登录界面。如果在已登录界面，请点击 `退出登录`。
3. 在 `请输入账号` 的地方输入想要注册的新账户名，比如 `YOUR_NEW_ACCOUNT` （这里需要替换成你自己的账号名）-> 点击 `注册`。正常情况下界面会显示 `注册成功，正在登录中..`。
4. 在已登录界面中，显示的当前用户账号为新注册的 `YOUR_NEW_ACCOUNT`

#### 设备端创建新的设备账号

1. 打开项目工程里的 app_config.h
2. 修改 `CONFIG_DEVICE_ID` 为您门铃设备的真实 Device ID，把原来的 `mydoorbell` 修改为比如 `YOUR_NEW_DEVICE`（这里需要替换成你自己的 Device ID）。
3. 启用 `CONFIG_REGISTER_NEW_DEVICE` 选项 (去掉注释即可)，这一步骤会把您新的设备 Device ID `YOUR_NEW_DEVICE` 注册到声网的后台服务。
4. 修改 `CONFIG_USER_ACCOUNT` 为刚才在 APP 端创建的用户账号 `YOUR_NEW_ACCOUNT`。
5. 重新编译和下载固件。至此，您将使用门铃端的新设备账号 `YOUR_NEW_DEVICE` 和 APP 端的新用户账号 `YOUR_NEW_ACCOUNT` 来进行视频呼叫、接听和通话。

## 关于 License

为了让您可以流畅体验我们的例程，并快速开始集成、测试您自己的项目，我们的设备端例程里内嵌了预激活的 License 证书 (`cert_for_test`)，提供了 `90天` 的免费试用期。免费期到期之后，该证书会失效，设备端例程将无法继续使用 (初始化函数 `agora_iot_init` 会失败)。

所以，在免费期到期之前，请务必联系声网销售(sales@agora.io) 或门铃方案负责人(微信：karstamu)，进一步了解商用 License 的购买和激活流程。

## 关于声网

声网音视频物联网平台方案，依托声网自建的底层实时传输网络 Agora SD-RTN™ (Software Defined Real-time Network)，为所有支持网络功能的 Linux/RTOS 设备提供音视频码流在互联网实时传输的能力。该方案充分利用了声网全球全网节点和智能动态路由算法，与此同时支持了前向纠错、智能重传、带宽预测、码流平滑等多种组合抗弱网的策略，可以在设备所处的各种不确定网络环境下，仍然交付高连通、高实时和高稳定的最佳音视频网络体验。此外，该方案具有极小的包体积和内存占用，适合运行在任何资源受限的 IoT 设备上，包括乐鑫 ESP32 全系列产品。

## 技术支持

请按照下面的链接获取技术支持：

- 如果发现了示例代码的 bug，欢迎提交 [issue](https://github.com/AgoraIO-Community/AG-VideoDoorbell-esp32/issues)
- 如果有其他疑问，也可以直接联系门铃方案负责人（微信：karstamu）

我们会尽快回复。

