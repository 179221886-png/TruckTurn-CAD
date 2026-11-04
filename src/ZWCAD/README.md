# TruckTurn 中望 CAD 移植版

纳入5.4.0三平台统一交付，包括货车整车、铰接半挂、集装箱搬运车、消防车、首尾/全部节点车体和轨迹动画；包含2026-11-04尺寸一致性修复。DLL元数据统一为5.4.0.0，实机验收状态见仓库说明。

## 目标

Windows x64，ZWCAD 2022、2023、2024、2025、2026逐年独立构建；目标为.NET Framework 4.7（可运行于4.7及以上兼容系统运行时，如4.8/4.8.1），不是.NET 8。须使用提供.NET接口的专业版或相应行业版，标准版不支持。

五组分别引用22.20.22.20、23.20.23.20、24.0.24.0、25.0.25.0、26.0.26.0的原始ZwManaged/ZwDatabaseMgd程序集，并已完整编译。不要跨年份/跨CAD宿主加载。

官方2026.NET移植指南说明2021–2024属于同一代、2025需要重新编译且2026兼容2025；本项目仍逐年构建以降低引用版本歧义。2027 Beta及未来正式版没有纳入当前构建和验收，不承诺自动兼容。

## 构建

使用.NET 8或以上SDK及.NET Framework 4.7引用程序集。需要还原Microsoft.NETFramework.ReferenceAssemblies.net47 1.0.3；工作区NuGet.Config默认关闭在线源，离线时请预先准备缓存，在线构建请显式使用允许的NuGet配置。

从合法的中望Windows x64安装或SDK取对应年份的 `ZwManaged.dll`、`ZwDatabaseMgd.dll`，将两者放到 `lib/2022/` 等目录。直接构建：

```powershell
dotnet build ./TruckTurn.ZWCAD2022.csproj -c Release
```

也可指定安装/SDK目录：

```powershell
dotnet build ./TruckTurn.ZWCAD2022.csproj -c Release '-p:ZwcadSdkDir=D:/SDK/ZWCAD/2022'
```

批量构建，SdkRoot下面按年份分目录，OutputRoot须为新的输出位置：

```powershell
./Build-ZWCAD.ps1 -SdkRoot D:/SDK/ZWCAD -OutputRoot D:/Build/TruckTurn-ZWCAD-test
```

脚本校验所选年份、程序集身份和主版本，不执行厂商程序集；SDK引用 `Private=false`，不随插件交付。没有对应年份SDK时明确失败，不冒用另一年API生成同名DLL。

## 实施差异及验证边界

- 命名空间改为 `ZwSoft.ZwCAD`，保留宿主模态/非模态窗口接口与Framework JSON后端。
- 原绘图工具中有一个从未被调用的周期样条入口，其构造器在2022–2024缺失；中望副本移除该死代码。当前轨迹、轮迹、包络仍使用既有多段线/圆弧bulge，不改变平滑算法或精度。
- 五年API均包含DrawJig、TransientManager、文档锁及文档切换/销毁事件，动画仍用不入库的临时图形；没有用每帧写数据库降级替代。
- 编译引用从公开NuGet `ZWCAD.NetApi` 20.22.0–20.26.0取得。该包由第三方InspireFunction维护，并非中望官方NuGet账号。仅使用其中原始、Authenticode验证为ZWSOFT有效签名的两份程序集作编译引用；没有使用第三方业务代码或把其包标的MIT许可套用给中望API。
- 编译/元数据/离线源链接回归不等同真实CAD验收；实际加载、SP补丁差异、模型空间/布局交互和动画GPU显示须实测。清单见仓库 `docs/ZWCAD_实机验收清单.md`。

## 来源

- [中望官方开发支持](https://www.zwsoft.cn/support/zwcad-devdoc)：2026.NET移植指南“开发环境及兼容性”、命名空间及NETLOAD章节。
- [官方.NET项目创建示例](https://helpmanager.zwsoft.cn/file/helpres/ZWCADAPI_zh-CN_2025/中望CAD二次开发1218/Development%20Samples/NET/NET_Create_Project.htm)。
- [官方标准版、专业版及行业版功能比较](https://www.zwsoft.com/user-guide/differences-between-zwcad-standard-professional-and-mfg)。
- [实际使用的第三方NuGet引用包](https://www.nuget.org/packages/ZWCAD.NetApi)。

源码和分类DLL随5.4.0同步；厂商SDK不入库、不分发。旧5.3.1可从Git tag恢复，没有修改CAD许可或绕过安全加载。
