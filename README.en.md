# Agora Video Doorbell for ESP32

This page guides you on running the demo project of the ESP32-S3 solution.

> This free 90-day trial demo version is available for testing purposes. Contact sales@agora.io or add the Wechat account karstamu to learn how to purchase and activate a commercial license.

## Run the device-side sample project

### Set up the development environment

Make sure your development environment meets the following requirements.

#### Hardware envrionment

- ESP32-S3-Korvo-2 
- A connected camera and loudspeaker for video/audio communication
- Two A to Micro-B USB cables used for power supply and data connection
- PC running Windows, Linux, or macOS

#### Software envrionment

1. Clone the sample project.

    ```shell
    git clone git@github.com:AgoraIO-Community/AG-VideoDoorbell-esp32.git
    ```

2. Install the following software in your computer:

    - Espressif IoT Development Framework (ESP-IDF). See the official documentation for installation steps. After installation, select the v4.4 branch.
        
        ```shell
        cd $IDF_PATH
        git checkout release/v4.4
        git pull
        git submodule update --init --recursive
        ```
        
   - The FreeRTOS patch of ESP-IDF. Run the following commands to install the patch.
       
       ```shell
       cd $IDF_PATH
       git apply $ADF_PATH/idf_patches/idf_v4.4_freertos.patch
       ```
   - Espressif Systems Audio Development Framework (ESP-ADF). See the official documentation for installation steps.

### Run the sample project

The following steps take macOS or Linux as an example:

1. Copy the AG-VideoDoorbell-esp32 folder to the ~/esp folder. Run the following command to compile the project:

     ```shell
     cd ~/esp/agora-demo-for-esp32
     idf.py set-target esp32s3
     # Configure WiFi SSID 和 WiFi Password in the menuconfig interface
     idf.py menuconfig
     # Build the project
     idf.py build
     ```

2. Flash the firmware to the device:

     ```shell
     $ idf.py -p /dev/ttyUSB0 flash monitor
     ```

> If you encounter the /dev/ttyUSB0 permission issue, run sudo usermod -aG dialout $USER to get privileges.

After flashing the firmware, this demo runs automatically. When initialization is successful, you can see the following information from the device:

```text
Agora: Press [REC] key to ring the doorbell ...
```

The device is in low-power mode.

## Run the client-side sample project

### Set up the development environment

Make sure your development environment meets the following requirements.

#### Hardware envrionment

- Android device or simulator
- PC running Windows, Linux, or macOS
- USB cable compatible with the Android device (not necessary for simulators)

#### Software envrionment

1. Download the latest version of Android Studio.
2. Clone the sample project:
      
      ```shell
      git clone git@github.com:AgoraIO-Community/AG-VideoDoorbell-Android.git
      ```
      
### Run the sample project

1. Open the sample project with Android Studio, which automatically syncs the project with Gradle.
2. After project sync, connect the Android device to the computer. Build and run the project on the Android device.


### Implement device-to-client communication 

Implement device-to-client communication with the following steps.

#### The device calls the client for a video chat

1. Open the **VideoDoorbell** app in the Android device.
2. Grant permission requests for microphone and local storage. Enter the **register/login** page.
3. Enter `jack` as the account, and click **Login** to enter the main interface.
4. On the ESP32-S3-Korvo-2 device, press [REC] to call user jack. The calling interface pops up in the **VideoDoorbell** app. You can receive the real-time video and audio from the device, but you cannot chat with the device.
5. Click **Answer** in the **VideoDoorbell** app to communicate with the device.
6. Click **Hang up** in the **VideoDoorbell** app to stop communicating with the client.

Before pressing [REC] to communicate with the client app, the doorbell is in low-power mode. The typical power consumption is below 800 μA. After calling, the device automatically switches to full-power mode.

#### The client wakes up the device for a video chat

Enter mydoorbell for the device account field. Click Call to see the real-time video from the device. You can also chat with the device.

