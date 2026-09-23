# TruckTurn AutoCAD port (v5.3)

This directory contains five AutoCAD managed plug-in builds. They share the same source files in `src/` and register `TRUCKDRIVE`, `TRUCKTURN90`, `CARRIERDRIVE`, and `CAR`.

| Project | Target AutoCAD releases | Target framework | Autodesk `AutoCAD.NET` package |
| --- | --- | --- | --- |
| `TruckTurn.AutoCAD2018.csproj` | 2018 | `net46` | 22.0.0 |
| `TruckTurn.AutoCAD2019_2020.csproj` | 2019–2020 | `net47` | 23.0.0 |
| `TruckTurn.AutoCAD2021_2024.csproj` | 2021–2024 | `net48` | 24.0.0 |
| `TruckTurn.AutoCAD2025_2026.csproj` | 2025–2026 | `net8.0-windows` | 25.0.1 |
| `TruckTurn.AutoCAD2027.csproj` | 2027 | `net10.0-windows` | 26.0.0 |

Restore packages from NuGet.org, then build a project with a compatible .NET SDK. The AutoCAD API assemblies are compile-time references; do not distribute `AcMgd.dll`, `AcDbMgd.dll`, or `AcCoreMgd.dll` with this plug-in. AutoCAD supplies them at runtime.

The AutoCAD 2018 output includes `System.ValueTuple.dll`, required by the `net46` build. Keep it beside `TruckTurn.AutoCAD2018.dll`. The other four builds do not copy a separate runtime DLL.

Load the DLL that matches your AutoCAD release with `NETLOAD`. AutoCAD 2025 Update 1.4 and AutoCAD 2026 Update 1.2 move their host to .NET 10; Autodesk says .NET 8 applications should normally continue to work, but these update levels still need a host smoke test. This port was compiled against Autodesk's versioned SDK packages; it has not been run inside AutoCAD on this build machine.

References: [Autodesk AutoCAD.NET packages](https://www.nuget.org/packages/AutoCAD.NET), [Autodesk managed compatibility table](https://help.autodesk.com/cloudhelp/2027/ENU/AutoCAD-Customization/files/GUID-A6C680F2-DE2E-418A-A182-E4884073338A.htm), [AutoCAD 2026 .NET update note](https://help.autodesk.com/cloudhelp/2026/ENU/AutoCAD-WhatsNew/files/GUID-FAB1960D-49C1-4A12-B128-5511F7889AB9.htm).
