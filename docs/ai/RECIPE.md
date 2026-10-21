# ESP32 IoT Device Demo

## Overview

Use ESP32 hardware as the device endpoint for an Agora-powered IoT audio/video communication demo.

Repository: [AgoraIO-Community/ag-iot-device-demo-esp32](https://github.com/AgoraIO-Community/ag-iot-device-demo-esp32)

## Prerequisites and Setup

1. Use the ESP32-S3-Korvo-2 board with camera/speaker and the compatible Agora Link SDK/license. Follow the pinned ESP-IDF v4.4 and ESP-ADF revisions and patch instructions in the repository README.
2. Configure the project, customer/product values and target app user in `main/app_config.h`; configure Wi-Fi with menuconfig.
3. In an activated ESP-IDF environment, run `idf.py set-target esp32s3`, `idf.py menuconfig`, then `idf.py build` from the repository root.
4. Flash with `idf.py -p <serial-port> flash monitor`. Provision and run the compatible mobile companion app described by the repository.

## What To Check

- Confirm the device reaches the REC-key standby prompt.
- Test device-to-app calling, answering and hang-up.
- Test app-to-device video viewing and two-way conversation.

## Integration Notes

The English and Chinese guides contain historical trial/license and sample-path details. Confirm current entitlement and use the actual checkout directory; do not assume old shared trial credentials remain active.

Keep App Certificates, customer secrets and privileged token generation on a trusted backend for production. Use only your own test credentials while configuring historical demos; do not commit them. Preserve the sample's existing dependency versions and lifecycle until a separate migration is tested.

## Source References

- [README.md](../../README.md)
- [README.en.md](../../README.en.md)
- [main/app_config.h](../../main/app_config.h)

## Validation Status

This recipe was checked against the repository's source and setup documents. The platform build and live service/device flow have not been run as part of this documentation change. Complete the checks above in an environment with the required SDKs, hardware and service access before treating it as a validated deployment.

## Implementation Prompt

```text
You are working in AgoraIO-Community/ag-iot-device-demo-esp32. Use docs/ai/RECIPE.md and its linked source files to adapt the ESP32 IoT Device Demo reference to this project.

Start by inspecting the actual sample directory and configuration described above. Resolve its listed missing prerequisites before claiming it is runnable. Preserve the existing architecture and compatible dependency versions. Implement the recipe's feature-specific flow, permissions and resource cleanup. Use a trusted backend for privileged credentials and tokens in production.

Run the repository-specific build or launch flow, then perform each What To Check item. Report the commands run, the observed results and anything blocked by service access, hardware or incomplete sample code. Do not claim live validation from a documentation or catalog build alone.
```
