# TruckTurn 5.3.1 · CAD 车辆转弯与扫掠包络

TruckTurn 在 CAD 中交互式绘制货车和集装箱跨运车的行驶轨迹、轮迹及扫掠包络。支持货车整车与铰接车，以及跨运车的前桥转向、后桥转向、反相四轮、蟹行等运动方式。可选择只显示首尾车体，或保留所有点击节点。

此包同时整理 AutoCAD 与浩辰 CAD（GstarCAD）的版本。**各 DLL 只能加载到下表指定的 CAD 宿主，不可交叉加载。**

| CAD 宿主 | 目标运行时 | DLL 位置 | 状态 |
| --- | --- | --- | --- |
| AutoCAD 2018 | .NET Framework 4.6 | `dist/AutoCAD/2018/` | 编译验证；待对应版本实机验证 |
| AutoCAD 2019–2020 | .NET Framework 4.7 | `dist/AutoCAD/2019-2020/` | 编译验证；待对应版本实机验证 |
| AutoCAD 2021–2024 | .NET Framework 4.8 | `dist/AutoCAD/2021-2024/` | 编译验证；待对应版本实机验证 |
| AutoCAD 2025–2026 | .NET 8 目标 | `dist/AutoCAD/2025-2026/` | 编译验证；不同更新补丁待实机验证 |
| AutoCAD 2027 | .NET 10 | `dist/AutoCAD/2027/` | 编译验证；待对应版本实机验证 |
| 浩辰 CAD 2025 | .NET Framework 4.8 | `dist/GstarCAD/2025/` | 沿用 5.3 交付版 |
| 浩辰 CAD 2026 | .NET 8 | `dist/GstarCAD/2026/` | 沿用 5.3 交付版 |

AutoCAD 的分组依据为 [Autodesk 官方 Managed .NET 兼容性表](https://help.autodesk.com/cloudhelp/2027/CHS/AutoCAD-Customization/files/GUID-A6C680F2-DE2E-418A-A182-E4884073338A.htm)。AutoCAD 2025 Update 1.4+ 与 2026 Update 1.2+ 已将宿主更新到 .NET 10；Autodesk 表示现有 .NET 8 应用通常仍能运行，但可能存在兼容性例外（[2025 更新说明](https://help.autodesk.com/cloudhelp/2025/ENU/AutoCAD-WhatsNew/files/GUID-07450FCA-16CA-4D7A-8EA2-9CE842631D75.htm)、[2026 更新说明](https://help.autodesk.com/cloudhelp/2026/ENU/AutoCAD-WhatsNew/files/GUID-FAB1960D-49C1-4A12-B128-5511F7889AB9.htm)）。编译成功并不等于全部版本已完成实机测试，以上补丁尤其要现场确认。

## 安装与使用

1. 从 `dist` 找到与 CAD 年份完全对应的 **TruckTurn DLL**，复制到本机固定文件夹。AutoCAD 2018 还须将同目录的 `System.ValueTuple.dll` 一起复制，其许可证和第三方通知也随包附上。AutoCAD 2025–2027 的 `.deps.json` 请保持与同名 DLL 同目录。
2. 在 CAD 命令行执行 `NETLOAD`，选择该 DLL。若 CAD 的安全加载拦截插件，把该文件夹加入 `TRUSTEDPATHS`；不要为加载插件而长期关闭安全加载。
3. 输入 `TRUCKDRIVE` 开始连续绘制货车轨迹，或输入 `TRUCKTURN90` 绘制一键 90° 转弯；输入 `CARRIERDRIVE`（简写 `CAR`）开始跨运车轨迹。
4. 在选型窗填写与项目一致的设备尺寸、轴距、轮距、转向能力及集装箱类型，再按界面提示确认行驶节点；`X` 完成出图。

更完整的参数解释和操作步骤见 [`docs/TruckTurn_5.3_插件使用说明.docx`](docs/TruckTurn_5.3_插件使用说明.docx)；AutoCAD 的现场检查步骤见 [`docs/AutoCAD_实机验收清单.md`](docs/AutoCAD_实机验收清单.md)。Word 文档沿用浩辰版 5.3 的说明，AutoCAD 的 DLL 选择以本页为准。

## 源码与构建

- `src/AutoCAD/`：AutoCAD 五组目标项目，使用 Autodesk 官方 [`AutoCAD.NET`](https://www.nuget.org/packages/AutoCAD.NET) NuGet API 包作为编译引用。
- `src/GstarCAD/`：浩辰 CAD 2025/2026 工程和离线几何回归验证器。构建浩辰工程时须自行从有权使用的浩辰 SDK/安装中取得 `GcMgd.dll`、`GcDbMgd.dll`、`GcCoreMgd.dll`，分别放入 `src/GstarCAD/lib2025/` 与 `src/GstarCAD/lib/`；这些第三方程序集不包含在仓库中。
- 交付 DLL 位于 `dist/`，不包含 CAD 宿主的 API 程序集。

从仓库根目录执行对应项目的 `dotnet build -c Release`。旧版 AutoCAD 工程还需对应 .NET Framework 引用程序集；AutoCAD 2027 工程需要 .NET 10 SDK。几何回归验证器可通过 `dotnet run -c Release --project src/GstarCAD/TruckTurn.Verify/TruckTurn.Verify.csproj` 运行。CAD 命令交互、NETLOAD 与图面效果需要在实际 CAD 中验收，离线回归不能替代。

## 权利与注意事项

TruckTurn 自身暂不提供开源许可证；公开查看源码**不等于**获得复制、修改或再发布授权。AutoCAD 2018 随包的 `System.ValueTuple.dll` 按其目录中的 MIT 许可和通知单独分发，不代表 TruckTurn 采用 MIT 许可。Autodesk、AutoCAD、浩辰 CAD 及其 API 程序集属于各自权利人。跨运车内置尺寸是通用参数模板，实际工程应以厂商资料和实测数据复核。插件输出是方案模拟，不代替驾驶操作、安全评估或设备厂商的技术确认。
