using System;
using System.Globalization;

namespace TruckTurn
{
    /// <summary>
    /// 货车几何与运动参数，支持两种车型：
    ///   · 铰接半挂车 (Articulated = true)：牵引车 + 挂车，挂车通过鞍座耦合
    ///   · 刚性单车   (Articulated = false)：单一刚体车厢（微卡/轻卡/重卡整车）
    /// 单位与当前图形的单位一致，由 UnitScale 控制：1 米对应多少 CAD 内部单位。
    /// 默认值已按图纸单位为米设定；若图纸单位为毫米，可在选型窗体里把「图纸单位」改为毫米。
    /// </summary>
    public class VehicleParams
    {
        // ---------- 车型标志 ----------
        public bool Articulated = true;   // true=铰接半挂车, false=刚性单车

        // ---------- 牵引车/单车共用车头部分 ----------
        public double TractorWheelbase;       // 前后轴距 L（前轴->后轴）。刚性单车即整车轴距
        public double TractorFrontOverhang;   // 前轴到车头(保险杠)的距离
        public double TractorRearToKingpin;   // 后轴到鞍座(kingpin)的距离，向前为正（刚性模式忽略）
        public double TractorWidth;          // 牵引车宽 / 刚性单车宽

        // ---------- 挂车（仅铰接） ----------
        public double TrailerKingpinToRearAxle; // 鞍座到挂车后轴的距离 L2
        public double TrailerRearOverhang;      // 挂车后轴到车尾的距离
        public double TrailerWidth;             // 挂车宽
        public int TrailerAxles;                // 挂车轴数(1/2/3)

        // ---------- 牵引车-挂车耦合几何（仅铰接） ----------
        /// <summary>
        /// 驾驶室后壁相对鞍座的前移量(m)：驾驶室后壁 = 鞍座 + 本值。
        /// 真车驾驶室后方只有窄车架，挂车前部悬在车架上方，所以「挂车压住车架」在平面图里是正常的；
        /// 真正不能碰的是驾驶室。本值不足时 <see cref="CabRearX"/> 会自动抬升到安全值。
        /// </summary>
        public double KingpinToCabRear = 1.3;
        /// <summary>牵引车底盘纵梁宽度(m)。真车约 0.9~1.1m，远窄于驾驶室。</summary>
        public double ChassisWidth = 1.0;
        /// <summary>后轴到车架尾端的距离(m)，向后为正。</summary>
        public double ChassisRearOverhang = 0.8;
        /// <summary>挂车与驾驶室之间要求的最小净距(m)。</summary>
        public double CabClearance = 0.1;
        /// <summary>鞍座到挂车真实前端的距离（向前为正）。默认 0.0m，挂车前端在 kingpin 处。</summary>
        public double TrailerKingpinToFront = 0.0;
        /// <summary>
        /// 最大铰接角（度）。超过此角挂车被机械限位。默认 45°。
        /// 取值依据：最小半径转弯的稳态铰接角约 35°（R=L/tanδ，L2=11m 时），45° 已留足余量；
        /// 同时它决定驾驶室后壁要退到多远（见 <see cref="CabRearX"/>），取 90° 会让驾驶室与挂车之间空出一米多。
        /// </summary>
        public double MaxArticulationAngleDeg = 45.0;

        /// <summary>
        /// 驾驶室后壁的纵向坐标（后轴为原点，向前为正）。
        ///
        /// 关键几何：铰接角为 δ 时，挂车前端最靠前的角点在牵引车坐标系中的纵向坐标为
        ///     x(δ) = 鞍座 + tf·cosδ + (挂车宽/2)·sinδ。
        /// 令 A = √(tf² + (W/2)²)，φ = atan2(tf, W/2)，则 x(δ) = ak + A·sin(δ + φ)。
        /// 若允许的最大铰接角为 δmax，则最大前伸量为：
        ///   - δ* = π/2 − φ 落在 [0, δmax] 内时取 A（完全展开）；
        ///   - 否则取 A·sin(δmax + φ)。
        /// 驾驶室后壁 ≥ 该值 + CabClearance，即可在**允许范围内的任意铰接角**下保证挂车不碰驾驶室。
        /// 构造性保证，不依赖仿真采样，也不会被参数组合绕过。
        /// </summary>
        public double CabRearX()
        {
            double ak = TractorRearToKingpin;
            double tf = TrailerKingpinToFront;
            double half = TrailerWidth * 0.5;
            double dmax = MaxArticulationAngleDeg * Math.PI / 180.0;
            double A = Math.Sqrt(tf * tf + half * half);
            double phi = Math.Atan2(tf, half);
            double deltaStar = Math.PI / 2.0 - phi;
            double swing = (deltaStar <= dmax) ? A : A * Math.Sin(dmax + phi);
            double safe = ak + swing + CabClearance;
            return Math.Max(safe, ak + KingpinToCabRear);
        }

        // ---------- 刚性单车专属补充（仅 Articulated=false 时有效） ----------
        public double RigidRearOverhang;        // 后轴到车尾的距离
        public int RigidAxles;                  // 整车轴数(2/3/4)

        // ---------- 通用 ----------
        public double WheelTrack;     // 轮距（用于绘制左右轮路径偏移）
        public double SteerAngleDeg;  // 前轮最大转角(度)，决定最小转弯半径
        public double TurnAngleDeg;   // 默认转弯角度(度)：车头方向偏转量
        // v4.6：原「安全余量 SafetyMargin」已移除（TRUCKCHECK 命令不再存在，该字段无任何消费方）。
        // ToCsv 的索引 15 保留占位 0，以免打乱后续字段索引、让已存图纸读错参数。
        public double UnitScale = 1000; // 1 米对应多少 CAD 内部单位；默认 1000(毫米)。弹窗可按图纸单位改

        // ---------- 图层显示选项（v4.4 起可在选型窗体勾选） ----------
        // v4.9.24：默认只开「车体 / 包络 / 前轴轨迹」三项——全开会多画轮径、中间姿态幽灵、半径标注，
        // 图元数量成倍增长，大图上明显卡顿。需要的用户可在选型窗自行勾选（选择会被记住）。
        public bool ShowEnvelope = true;
        public bool ShowBody = true;
        public bool ShowWheels = false;
        public bool ShowGhost = false;
        public bool ShowFrontTrack = true;
        public bool ShowRadii = false;
        /// <summary>
        /// 连续驾驶时是否保留每个确认节点的车辆姿态。
        /// false=仅保留起点与终点，true=保留全部确认节点。
        /// 包络、轮迹和实时预览不受此开关影响。
        /// </summary>
        public bool ShowAllNodeVehicles = false;

        /// <summary>一组常见半挂车默认值（单位与 UnitScale 一致；v3.1 起默认图纸单位为米）</summary>
        public static VehicleParams Defaults()
        {
            return new VehicleParams
            {
                Articulated = true,
                TractorWheelbase = 6.0,
                TractorFrontOverhang = 1.5,
                TractorRearToKingpin = 0.5,
                TractorWidth = 2.5,
                TrailerKingpinToRearAxle = 11.0,
                TrailerRearOverhang = 1.3,
                TrailerWidth = 2.5,
                TrailerAxles = 3,
                RigidRearOverhang = 0,
                RigidAxles = 2,
                WheelTrack = 1.85,
                SteerAngleDeg = 18,
                TurnAngleDeg = 90,
                UnitScale = 1.0,
                // v4.4 新增几何/显示默认值
                KingpinToCabRear = 1.3,
                ChassisWidth = 1.0,
                ChassisRearOverhang = 0.8,
                CabClearance = 0.1,
                TrailerKingpinToFront = 0.0,
                MaxArticulationAngleDeg = 45.0,
                ShowEnvelope = true,
                ShowBody = true,
                ShowWheels = false,
                ShowGhost = false,
                ShowFrontTrack = true,
                ShowRadii = false,
                ShowAllNodeVehicles = false
            };
        }

        // ---------- 供选型窗体读取/展示的便捷属性（单位 m） ----------

        /// <summary>主车长(m)：刚性 = 整车长；铰接 = 挂车长（与牵引车同宽等长假设无关）</summary>
        public double LengthMeters()
        {
            if (UnitScale <= 0) UnitScale = 1000;
            if (Articulated)
            {
                double tractor = TractorFrontOverhang + TractorWheelbase + TractorRearToKingpin;
                double trailer = TrailerKingpinToRearAxle + TrailerRearOverhang;
                return (tractor + trailer) / UnitScale;
            }
            return (TractorFrontOverhang + TractorWheelbase + RigidRearOverhang) / UnitScale;
        }

        /// <summary>
        /// v4.6.1：挂车自身长度(m) = 挂车前端到鞍座 + 鞍座到挂车后轴 + 挂车后悬。
        /// 刚性车无挂车，返回整车长。
        ///
        /// 必须和 <see cref="TruckSizeForm.BuildResult"/> 的反解保持一致：那边是
        ///   挂车总长 = TrailerKingpinToFront + TrailerKingpinToRearAxle + TrailerRearOverhang
        /// 此前选型窗回填「主车长」时误用了 <see cref="LengthMeters"/>（铰接时返回
        /// 牵引车+挂车的**列车总长**），于是每打开一次窗体挂车就被多加一次牵引车长，
        /// 参数在反复打开中不断膨胀。
        /// </summary>
        public double TrailerLengthMeters()
        {
            if (UnitScale <= 0) UnitScale = 1000;
            if (!Articulated) return LengthMeters();
            return (TrailerKingpinToFront + TrailerKingpinToRearAxle + TrailerRearOverhang) / UnitScale;
        }

        /// <summary>牵引车长(m) = 前悬 + 轴距 + 后轴到鞍座。刚性车返回 0（该字段在刚性模式下隐藏）。</summary>
        public double TractorLengthMeters()
        {
            if (UnitScale <= 0) UnitScale = 1000;
            if (!Articulated) return 0;
            return (TractorFrontOverhang + TractorWheelbase + TractorRearToKingpin) / UnitScale;
        }

        /// <summary>主车宽(m)：刚性 = 整车宽；铰接 = 挂车宽（与牵引车同宽假设）</summary>
        public double WidthMeters()
        {
            if (UnitScale <= 0) UnitScale = 1000;
            return (Articulated ? TrailerWidth : TractorWidth) / UnitScale;
        }

        /// <summary>主车轴数：刚性 = 整车轴数；铰接 = 挂车轴数（不含车头）</summary>
        public int AxleCount()
        {
            return Articulated ? TrailerAxles : RigidAxles;
        }

        /// <summary>序列化为逗号分隔字符串，便于存入图形字典。v3 起含车型/轴数字段；v4.4 起含 kingpin 间隙/最大铰接角/图层选项。</summary>
        public string ToCsv()
        {
            var vals = new object[]
            {
                TractorWheelbase, TractorFrontOverhang, TractorRearToKingpin, TractorWidth,
                TrailerKingpinToRearAxle, TrailerRearOverhang, TrailerWidth,
                WheelTrack, SteerAngleDeg, TurnAngleDeg,
                Articulated ? 1 : 0, TrailerAxles, RigidRearOverhang, RigidAxles,
                UnitScale,
                0,                  // 索引15：原「安全余量」，v4.6 已移除，占位保持后续索引不变
                // v4.4
                TrailerKingpinToFront, MaxArticulationAngleDeg,
                KingpinToCabRear, ChassisWidth, ChassisRearOverhang, CabClearance,
                ShowEnvelope ? 1 : 0, ShowBody ? 1 : 0, ShowWheels ? 1 : 0,
                ShowGhost ? 1 : 0, ShowFrontTrack ? 1 : 0, ShowRadii ? 1 : 0,
                ShowAllNodeVehicles ? 1 : 0
            };
            return string.Join(",", Array.ConvertAll(vals, v => Convert.ToString(v, CultureInfo.InvariantCulture)));
        }

        /// <summary>从逗号分隔字符串解析；兼容旧版 10 字段格式（无车型/轴数）。</summary>
        public static VehicleParams FromCsv(string s)
        {
            var p = Defaults();
            if (string.IsNullOrWhiteSpace(s)) return p;
            var a = s.Split(',');
            if (a.Length < 10) return p;

            double d(int i) => double.TryParse(a[i], NumberStyles.Any, CultureInfo.InvariantCulture, out var v) ? v : 0;
            int n(int i) => int.TryParse(a[i], NumberStyles.Any, CultureInfo.InvariantCulture, out var v) ? v : 0;

            p.TractorWheelbase = d(0);
            p.TractorFrontOverhang = d(1);
            p.TractorRearToKingpin = d(2);
            p.TractorWidth = d(3);
            p.TrailerKingpinToRearAxle = d(4);
            p.TrailerRearOverhang = d(5);
            p.TrailerWidth = d(6);
            p.WheelTrack = d(7);
            p.SteerAngleDeg = d(8);
            p.TurnAngleDeg = d(9);

            if (a.Length >= 11) p.Articulated = n(10) != 0;
            if (a.Length >= 12) p.TrailerAxles = n(11);
            if (a.Length >= 13) p.RigidRearOverhang = d(12);
            if (a.Length >= 14) p.RigidAxles = n(13);
            if (a.Length >= 15) p.UnitScale = d(14);
            else p.UnitScale = 1000; // v1/v2/v3 旧格式未存单位，默认按毫米解释
            // 索引15 原为「安全余量」，v4.6 已移除；读到的旧值直接忽略（占位防止后续字段错位）。
            // v4.4 新增字段（缺失时保持默认值）
            if (a.Length >= 17) p.TrailerKingpinToFront = d(16);
            if (a.Length >= 18) p.MaxArticulationAngleDeg = d(17);
            if (a.Length >= 19) p.KingpinToCabRear = d(18);
            if (a.Length >= 20) p.ChassisWidth = d(19);
            if (a.Length >= 21) p.ChassisRearOverhang = d(20);
            if (a.Length >= 22) p.CabClearance = d(21);
            if (a.Length >= 23) p.ShowEnvelope = n(22) != 0;
            if (a.Length >= 24) p.ShowBody = n(23) != 0;
            if (a.Length >= 25) p.ShowWheels = n(24) != 0;
            if (a.Length >= 26) p.ShowGhost = n(25) != 0;
            if (a.Length >= 27) p.ShowFrontTrack = n(26) != 0;
            if (a.Length >= 28) p.ShowRadii = n(27) != 0;
            if (a.Length >= 29) p.ShowAllNodeVehicles = n(28) != 0;
            if (p.UnitScale <= 0) p.UnitScale = 1000;
            return p;
        }
    }
}
