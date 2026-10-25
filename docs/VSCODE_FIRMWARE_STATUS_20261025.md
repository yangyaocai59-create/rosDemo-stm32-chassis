# VS Code 固件构建与烧录状态同步

工程使用 VS Code 的 **STM32** 配置文件。原截图的分析对象为 `build/Debug/rosDemo.elf`，不是新版本的独立构建。现在当前配置为 `WheelV0205Release`，STM32 Build Analyzer 指向 `build/WheelV0205Release/rosDemo-0.20.5.elf`。

## 内存百分比与 MAP 配对

内存分析页面属于 ST 官方 **STM32 Build Analyzer**；本地 `rosdemo.firmware-status` 只同步构建与烧录报告，并打开官方分析页。

首次切换到版本化 ELF 后，页面提示“未知工具链，未找到映射文件”，区域容量为 0，无法显示百分比。原因是 ST 插件查找与 ELF 同名的 MAP，而链接器仍输出 `rosDemo.map`。此前仅确认了当前预设和分析对象切换，遗漏了 MAP 配对。

已在 CMake 成功链接之后、界面通知之前生成 `rosDemo-<版本>.map` 副本，原 `rosDemo.map` 保留给既有打包流程。同步脚本也能为已有构建补齐同名副本，并检查 MAP 的 OUTPUT 指向对应 ELF。今后构建会自动处理，不修改 ST 插件、不写死内存百分比。

实际验证 Release 重新链接、同名 MAP 与原 MAP 字节一致、ST 插件重新提取成功、界面刷新回执成功。当前链接器报告 RAM 105216 / 131072 字节（80.27%）、FLASH 81632 / 786432 字节（10.38%）、CCMRAM 0 / 65536 字节（0%）。本次只修改构建/编辑器流程，不需要重新烧录或校准，所有已归档固件保持原哈希。

## 使用

- 使用原来的 CMake 编译按钮，或 `Ctrl+Shift+B` 的“rosDemo: 编译当前固件并更新界面”。成功链接并生成 HEX 后自动同步版本、配置、ELF、HEX、文件时间与哈希，并打开对应的构建分析。
- CMake 当前预设与代码补全的 compile_commands 路径随成功构建一起更新。默认旧预设和所有版本目录保留。
- 状态栏分别显示“编译版本”和“烧录版本（用户报告）”。外部烧录器固定全片擦除；当前没有其烧录完成回调，也没有从板上独立读取完整版本的接口。用户确认烧录后，再记录该版本，不能把编译成功当作已烧录。
- 手动刷新可在命令面板运行“rosDemo: 更新当前固件与构建分析”，查看详情用“rosDemo: 查看编译与烧录状态”。

成功编译的实际数据在 `.vscode/firmware-status.json`；VS Code 执行命令后的回执在 `.vscode/firmware-ui-receipt.json`。回执包含实际 active preset、分析 ELF 路径和所处理的请求时间。回执的请求时间应与状态文件一致，且 success=true。界面同步失败时状态栏显示警告，错误记录另存；先前错误不能冒充后续同步结果。

## 实现与验证

`Tools/sync_vscode_firmware.py` 由 CMake 的 POST_BUILD 调用，也在离线候选打包成功后调用。它先验证 ELF、版本文本、HEX 存在和预设，再原子写入状态。构建更新保留已单独记录的烧录报告，不操作串口、校准或烧录，也不覆盖 `build/firmware/<版本>` 和 `releases/<版本>` 的归档。

本地扩展源代码位于 `Tools/vscode-firmware-status`，没有第三方依赖或网络访问，通过 VS Code 命令调用 CMake Tools 与 ST Build Analyzer，不改它们的安装文件或私有状态数据库。扩展只处理唯一含 `rosDemo.ioc` 的工程，并检查 ELF 仍与构建记录哈希一致。

当前安装在 STM32 配置中的扩展为 `rosdemo.firmware-status` 0.1.1。早期试装和一次重载辅助扩展已移除。可重建安装包：

```powershell
python -B Tools/package_vscode_firmware_status.py
code --profile STM32 --install-extension build/vscode/rosdemo-firmware-status-0.1.1.vsix --force
```

初次安装后若 VS Code 要求重载，执行一次“重新加载窗口”。回退可通过扩展界面卸载该本地扩展，原有 CMake/STM32 扩展保持不变。

本轮验证包括：实际重编译 Release、POST_BUILD 写入新状态、VS Code 自动切换/保持 WheelV0205Release、对应分析命令完成以及请求时间匹配的成功回执。Release RAM/FLASH 为 105216 / 81632 字节。此次重编译的 HEX 与已交付的 0.20.5 相比，仅构建时间字符串的 5 个字节不同，其余被编程字节一致；已归档固件未改变，不需要因为界面同步再刷固件。

烧录报告只能说明用户确认刷入了 0.20.5，不能证明新重编译文件的哈希与板上镜像完全相同。两类状态独立保留。

实现依据为 [VS Code 扩展 API](https://code.visualstudio.com/api/references/vscode-api)及 [CMake Tools 预设说明](https://github.com/microsoft/vscode-cmake-tools/blob/main/docs/cmake-presets.md)，实际命令参数与 ST 分析入口另核对了本机安装版本的代码。
