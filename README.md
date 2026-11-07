# Forza Horizon 4 载具自由移动

《极限竞速：地平线4》单人模式下的载具位置控制工具。
每次启动现场定位坐标结构，方向取自游戏自带的 Data Out 偏航角，操作界面用 ImGui 绘制。

> 仅限单人 / 离线使用。

## 功能


- **Data Out 偏航角**：读取游戏自带的数据输出（UDP 324 字节，Yaw 在 offset 56），
  地面为 X-Z 平面、Y 为高度。
- **ImGui 界面（DirectX 11）**：无边框、可折叠、可自由缩放，默认深色面板，可切换透明。
- **操作**：目标地址多选/单选、速度 20–500 m/s、存点/回点、运行日志。
- **按键可配置**：编辑 `fh4_keys.ini`，或使用 `--key-fwd` 等参数。
- **安全闸门**：没有 Data Out 不允许移动；未通过位移验证不能武装写入；
  写前校验有限值与坐标范围。


## 使用流程

1. 进入游戏，坐在车上进入自由漫游；
2. 以管理员身份运行 `fh4_coord.exe`；
3. 按提示按 `ESC` / 刹车让车完全停住（第一遍搜索要求静止）；
4. 程序提示后开动车移动十几米，再移动一小段完成复筛；
5. 校验通过后打开 ImGui 界面，勾选"武装写入"，按住方向键移动。

## 数据输出设置

```
设置 -> 抬头显示与游戏玩法 -> 数据输出 = 开
IP 地址 127.0.0.1, 端口 5300
```

没收到数据包时程序不会允许移动，会提示你开启数据输出后按任意键重试。


## 构建

需要 Visual Studio（MSVC x64 工具链）以及仓库内的 `imgui/` 目录。

双击 `build.bat`，或执行：

```
cl /nologo /utf-8 /EHsc /O2 /std:c++17 /MT /W3 /I"imgui" /I"imgui\backends" ^
   fh4_coord.cpp imgui\imgui.cpp imgui\imgui_draw.cpp imgui\imgui_tables.cpp imgui\imgui_widgets.cpp ^
   imgui\backends\imgui_impl_win32.cpp imgui\backends\imgui_impl_dx11.cpp ^
   /Fe:fh4_coord.exe /Fo:obj\
```

> `build.bat` 必须保持纯 ASCII：cmd.exe 按 ANSI 解析批处理，中文注释会把脚本拆坏。

## 运行要求

- 64 位 Windows 10 / 11；
- 管理员权限（程序会自行请求提权）；
- 游戏正在运行且已进入驾驶状态。

## 免责声明

本项目仅供单人 / 离线模式的游戏研究与学习使用。
在多人线上模式使用可能导致封号、进度清零等后果，风险由使用者自行承担。
