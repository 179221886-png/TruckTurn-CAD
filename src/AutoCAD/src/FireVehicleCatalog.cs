using System;
using System.Collections.Generic;

namespace TruckTurn
{
    /// <summary>
    /// 消防车参考模板。尺寸始终以米记录，不能把通道圆直径直接当作前轴/车心半径。
    /// 第一版只精确表示两轴、前桥转向的平面刚体；不是厂家认证车型数据库。
    /// </summary>
    public sealed class FireVehicleProfile
    {
        public string Id { get; internal set; }
        public string Name { get; internal set; }
        public bool Supported { get; internal set; }
        public bool IsManufacturerReference { get; internal set; }
        public double LengthM { get; internal set; }
        public double WidthM { get; internal set; }
        public double HeightM { get; internal set; }
        public double WheelbaseM { get; internal set; }
        public double FrontOverhangM { get; internal set; }
        public double RearOverhangM { get; internal set; }
        public double TrackM { get; internal set; }
        public double SteerDeg { get; internal set; }
        public string SourceTitle { get; internal set; }
        public string SourceUrl { get; internal set; }
        public string SourceNotes { get; internal set; }
        public string PublishedTurningData { get; internal set; }
        public override string ToString() { return Name; }
    }

    public static class FireVehicleCatalog
    {
        /// <summary>返回全新的条目，调用者修改条目不会污染下一次选型。</summary>
        public static IList<FireVehicleProfile> CreateProfiles()
        {
            return new List<FireVehicleProfile>
            {
                new FireVehicleProfile
                {
                    Id = "generic-water-2axle", Name = "通用两轴水罐消防车（参考模板）", Supported = true,
                    LengthM = 8.50, WidthM = 2.50, HeightM = 3.30, WheelbaseM = 4.50,
                    FrontOverhangM = 1.40, RearOverhangM = 2.60, TrackM = 1.95, SteerDeg = 38.0,
                    SourceTitle = "插件通用参考模板（非厂家车型、非标准规定值）", SourceUrl = "",
                    SourceNotes = "全部尺寸及转角为可编辑的设计参考值，并非实车测量或法规默认值。请按项目指定车辆核对轴距、前后悬、轮距和转角。",
                    PublishedTurningData = "无厂家转弯数据；仅按当前输入参数计算。"
                },
                new FireVehicleProfile
                {
                    Id = "zoomlion-zlj5060-reference", Name = "中联 ZLJ5060TXFJY68（历史公开参数参考）",
                    Supported = true, IsManufacturerReference = true,
                    LengthM = 7.10, WidthM = 1.93, HeightM = 2.90, WheelbaseM = 3.815,
                    FrontOverhangM = 1.10, RearOverhangM = 2.185, TrackM = 1.40, SteerDeg = 38.0,
                    SourceTitle = "中联重科历史产品页：ZLJ5060TXFJY68（2015 页面）",
                    SourceUrl = "https://sales-cloud.zoomlion.com/equipment/content/201508/821.html",
                    SourceNotes = "公开值：长7.10m、宽1.93m、高2.90m、轴距3.815m。前悬1.10m、后悬2.185m、轮距1.40m、前轮转角38°是插件估算，尚未经厂家核验；轮距估算仅为使本模板轮胎在全转角内不超出所填车宽，不代表厂家轮距或认证转弯性能。不同公告/批次可能不同，勿混用数据。",
                    PublishedTurningData = "厂家页面：最小转弯直径（通道圆）16m。仅供口径说明，不用于反算本模板转角；当前模拟不承诺匹配16m。"
                },
                new FireVehicleProfile
                {
                    Id = "custom-two-axle", Name = "自定义两轴消防车（用户参数）", Supported = true,
                    LengthM = 8.50, WidthM = 2.50, HeightM = 3.30, WheelbaseM = 4.50,
                    FrontOverhangM = 1.40, RearOverhangM = 2.60, TrackM = 1.95, SteerDeg = 38.0,
                    SourceTitle = "用户自定义参数（未核验）", SourceUrl = "",
                    SourceNotes = "本选项不作厂家或型号识别。参数由用户核对；初始值仅继承当前窗口数值，不能据此认定为任何实车型号。",
                    PublishedTurningData = "请分别核实外轮、车体、通道圆等测量口径，不能直接互换。"
                },
                new FireVehicleProfile
                {
                    Id = "xcmg-jp62-unsupported", Name = "徐工 JP62T5（多轴，暂不支持精确模拟）", Supported = false,
                    SourceTitle = "徐工 JP62T5 厂家产品页", SourceUrl = "https://www.xcmg.com/product/pro-detail-120000.htm",
                    SourceNotes = "多轴举高喷射车具有多个轴距段，不能把多个轴距相加后当成普通两轴前桥转向。本版禁止套用两轴模板生成该型号。",
                    PublishedTurningData = "需取得各轴位置、各桥转角/联动关系及收拢外廓后再建模。"
                },
                new FireVehicleProfile
                {
                    Id = "rosenbauer-rt-unsupported", Name = "Rosenbauer RT（后桥辅助转向，暂不支持）", Supported = false,
                    SourceTitle = "Rosenbauer RT 厂家参数表", SourceUrl = "https://emobility.rosenbauer.com/en/download/3320/15318",
                    SourceNotes = "该车型存在后桥辅助转向，不能使用本窗口的两轴前桥模型冒充实车；本版仅列出边界，不生成该型号轨迹。",
                    PublishedTurningData = "需取得前后桥转向关系、模式限制和配置对应尺寸后再支持。"
                }
            };
        }

        public static bool IsFinite(double value)
        {
            return !double.IsNaN(value) && !double.IsInfinity(value);
        }

        /// <summary>几何约束：独立读取真实轴距/前后悬，绝不按整车长比例反解。</summary>
        public static bool ValidateGeometry(double width, double height, double wheelbase,
            double front, double rear, double track, double steer, double scale, out string error)
        {
            error = null;
            double[] values = { width, height, wheelbase, front, rear, track, steer, scale };
            foreach (double value in values)
                if (!IsFinite(value)) { error = "参数必须是有限数字，不能包含 NaN 或无穷大。"; return false; }
            if (scale <= 0 || scale > 1000000) error = "图纸单位比例必须大于0且不大于1000000单位/米。";
            else if (width < 0.8 || width > 4.0) error = "车宽请填写0.8～4.0米。";
            else if (height < 0.5 || height > 6.0) error = "车高请填写0.5～6.0米。";
            else if (wheelbase < 0.5 || wheelbase > 14.0) error = "轴距请填写0.5～14.0米。";
            else if (front < 0 || front > 5.0 || rear < 0 || rear > 8.0) error = "前悬应为0～5.0米，后悬应为0～8.0米。";
            else if (front + wheelbase + rear < 2.0 || front + wheelbase + rear > 18.0) error = "整车长（前悬＋轴距＋后悬）应为2.0～18.0米。";
            else if (track <= 0 || track > width) error = "轮距必须大于0，且不能超过车宽。";
            else if (steer < 1 || steer > 70) error = "最大前轮转角请填写1～70度；应依据实车核对。";
            if (error != null) return false;
            // CAD 当前按 0.50 × 0.25m 轮胎矩形出图。不能只核对轮中心，
            // 应检查从直行到左右满舵全过程的最坏投影；峰值不一定在满舵点。
            double halfLength, halfWidth;
            VehicleParams.WheelExtentsMeters(steer, out halfLength, out halfWidth);
            const double tolerance = 1e-9;
            if (track + 2 * halfWidth > width + tolerance)
                error = "轮胎在转向过程中会超出车宽：当前轮距至少需要车宽 " +
                    MinimumDisplay(track + 2 * halfWidth) + " 米。请减小轮距或增大车宽；参数不会自动修改。";
            else if (front + tolerance < halfLength)
                error = "前悬不足以容纳转向轮胎：至少需要 " + MinimumDisplay(halfLength) + " 米；参数不会自动修改。";
            else if (rear + tolerance < 0.25)
                error = "后悬不足以容纳后轮轮胎：至少需要0.25米；参数不会自动修改。";
            return error == null;
        }

        // 建议的最小输入按毫米向上取整，避免显示值略小于精确下限、照填后仍不通过。
        private static string MinimumDisplay(double metres) { return (Math.Ceiling(metres * 1000) / 1000).ToString("0.###"); }

        public static VehicleParams BuildVehicle(double width, double height, double wheelbase,
            double front, double rear, double track, double steer, double scale, VehicleParams displayOptions)
        {
            string error;
            if (!ValidateGeometry(width, height, wheelbase, front, rear, track, steer, scale, out error))
                throw new ArgumentException(error);
            VehicleParams p = VehicleParams.Defaults();
            p.Articulated = false;
            p.BodyWidthIncludesMirrors = true;
            p.RigidAxles = 2;
            p.TractorWheelbase = wheelbase * scale;
            p.TractorFrontOverhang = front * scale;
            p.RigidRearOverhang = rear * scale;
            p.TractorWidth = width * scale;
            p.WheelTrack = track * scale;
            p.SteerAngleDeg = steer;
            p.UnitScale = scale;
            p.TurnAngleDeg = 90;
            // 隔离旧半挂几何；未使用的刚性参数也保持相同单位，避免误读造成尺度污染。
            p.TractorRearToKingpin = 0;
            p.TrailerKingpinToRearAxle = 0;
            p.TrailerRearOverhang = 0;
            p.TrailerKingpinToFront = 0;
            p.TrailerWidth = width * scale;
            p.KingpinToCabRear = 0;
            p.ChassisWidth = Math.Min(1.0, width) * scale;
            p.ChassisRearOverhang = rear * scale;
            p.CabClearance = 0.1 * scale;
            if (displayOptions != null)
            {
                p.ShowEnvelope = displayOptions.ShowEnvelope;
                p.ShowBody = displayOptions.ShowBody;
                p.ShowWheels = displayOptions.ShowWheels;
                p.ShowGhost = displayOptions.ShowGhost;
                p.ShowFrontTrack = displayOptions.ShowFrontTrack;
                p.ShowRadii = displayOptions.ShowRadii;
                p.ShowAllNodeVehicles = displayOptions.ShowAllNodeVehicles;
            }
            return p;
        }
    }
}
