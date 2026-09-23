using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.Geometry;

namespace TruckTurn
{
    /// <summary>一帧车辆姿态（所有坐标在 World XY 平面，z=0）</summary>
    public class Frame
    {
        public Point2d O1; // 牵引车后轴中心
        public Vector2d U1; // 牵引车前进方向(单位向量)
        public Point2d K;  // 鞍座(kingpin)位置
        public Vector2d U2; // 挂车前进方向(单位向量)
        public Point2d O2; // 挂车后轴中心
    }

    /// <summary>
    /// 铰接半挂车运动学：牵引车用后轴参考的自行车模型，挂车通过鞍座"无侧滑"约束耦合。
    ///
    /// 路径模型（v4.1 起）：
    ///   · 默认交互命令(TRUCKPATH/TRUCKDRIVE)优先使用【自适应单圆弧】：由起点前轴/航向/目标前轴
    ///     反解出恰好通过目标点的后轴中心圆，半径随目标自适应（目标越远越缓），整段均为圆弧、无切线直线。
    ///   · 当目标过近导致所需半径 &lt; 最小转弯半径时，自动回退到经典的【圆弧段 + 切线直线段】。该组合
    ///     中前轮保持恒定转角 δ，后轴中心绕转弯中心 C 做精确圆周运动（半径 R=L/tanδ，无欧拉累积误差）。
    ///   · 直线段：前轮回正(δ=0)，牵引车直行，挂车航向角逐渐趋向牵引车（自然拉直）。
    /// 模拟函数接受可选的 turnRadiusOverride 以支持自适应圆弧（覆盖默认的 R=L/tanδ）。
    /// </summary>
    public static class TruckKinematics
    {
        /// <summary>
        /// 实时预览专用关键帧抽取。首末帧始终保留；确认后的正式出图不调用本方法，
        /// 仍使用完整运动帧。均匀按时间序列抽取可同时覆盖圆弧、拉直与铰接收敛段。
        /// </summary>
        public static List<Frame> PreviewFrames(IList<Frame> frames, int maximumFrames)
        {
            var result = new List<Frame>();
            if (frames == null || frames.Count == 0) return result;
            if (maximumFrames < 2 || frames.Count <= maximumFrames)
            {
                for (int i = 0; i < frames.Count; i++) result.Add(frames[i]);
                return result;
            }

            int previous = -1;
            for (int i = 0; i < maximumFrames; i++)
            {
                int index = (int)Math.Round(
                    i * (frames.Count - 1.0) / (maximumFrames - 1.0));
                if (index == previous) continue;
                result.Add(frames[index]);
                previous = index;
            }
            return result;
        }

        /// <summary>最终出图/确认后的精细角度采样步长（度）。越小越光滑，但点越多。</summary>
        public const double FineStepDeg = 0.25;

        /// <summary>
        /// v4.9.10：直行段包络台阶齿的深度预算（米）。
        /// 铰接车带铰接角 θ 直行时 θ 指数衰减、挂车横向摆动，离散帧并集的包络外边界呈
        /// 「挂车前外角点 → 前短边 → 长边」的折线台阶，齿深 ≈ ds·|sinθ|
        /// （旧版 ds=0.5m、θ=40° 时齿深 ≈0.26m，即用户截图里 S 弯后直行段的大幅锯齿）。
        /// 直行步长改按 StraightStepLimit 钳到 本预算/|sinθ|：齿深 ≤2cm，
        /// 1:50 出图仅 0.4mm 不可见，且低于稀化容差 5cm 会被 RDP 平滑掉。
        /// </summary>
        public const double StraightToothBudgetM = 0.02;

        /// <summary>
        /// 预览（Jig）专用齿深预算（v4.9.14）：预览每次鼠标移动都要重算包络
        /// （EnvelopeTracks ~10ms/91帧），用 0.02m 预算时残留铰接角段帧数 ~16×、
        /// 拖动明显变卡。预览放宽到 0.10m（帧数 ~5×↓，齿 ≤10cm 仅存在于拖动中的
        /// 临时预览），确认出图仍用 StraightToothBudgetM = 0.02m 精采样。
        /// </summary>
        public const double PreviewToothBudgetM = 0.10;

        /// <summary>
        /// 直行段自适应步长上限（v4.9.10）：baseDs（原 0.5m 钳位）与
        /// budget/|sinθ| 取小；θ→0 时退回 baseDs。
        /// 下限 1mm 防极端参数下 while 循环过细。
        /// </summary>
        public static double StraightStepLimit(double thetaRad, double baseDs,
                                               double budget = StraightToothBudgetM)
        {
            double s = Math.Abs(Math.Sin(thetaRad));
            double lim = s < 1e-6 ? baseDs : Math.Min(baseDs, budget / s);
            return lim < 1e-3 ? 1e-3 : lim;
        }

        /// <summary>兼容旧签名：仅转弯，无直行段</summary>
        public static List<Frame> Simulate(VehicleParams p, Point2d start, Vector2d heading, int steerSign, double turnAngleDeg)
        {
            return Simulate(p, start, heading, steerSign, turnAngleDeg, 0.0, 1.0);
        }

        /// <summary>
        /// 模拟一次「转弯 + 直行」运动。
        /// </summary>
        /// <param name="p">车辆参数</param>
        /// <param name="start">起始点 = 牵引车后轴中心</param>
        /// <param name="heading">初始车头方向(任意长度，内部归一化)</param>
        /// <param name="steerSign">+1=左转, -1=右转</param>
        /// <param name="turnAngleDeg">圆弧段：牵引车车头方向总偏转角(度)，可为 0</param>
        /// <param name="straightLen">直线段：转弯结束后沿车头方向直行的长度，可为 0</param>
        /// <param name="degPerStep">圆弧段采样步长(度)。1=精细(出图用)，5=粗略(动态预览用)</param>
        public static List<Frame> Simulate(VehicleParams p, Point2d start, Vector2d heading, int steerSign,
                                           double turnAngleDeg, double straightLen, double degPerStep,
                                           double? turnRadiusOverride = null)
        {
            // 刚性单车走独立运动学（整体刚体绕后轴旋转），铰接半挂走耦合模型
            if (!p.Articulated)
                return SimulateRigid(p, start, heading, steerSign, turnAngleDeg, straightLen, degPerStep, turnRadiusOverride);

            var frames = new List<Frame>();

            double L = p.TractorWheelbase;
            double ak = p.TractorRearToKingpin;
            double L2 = p.TrailerKingpinToRearAxle;
            double maxArt = p.MaxArticulationAngleDeg;
            if (L <= 1e-9 || L2 <= 1e-9) return frames;

            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u10 = heading.GetNormal();
            double phi10 = Math.Atan2(u10.Y, u10.X);
            Vector2d leftNormal = new Vector2d(-u10.Y, u10.X);
            Point2d C = start + (steerSign * R) * leftNormal;   // 转弯中心

            double tanDelta = L / R;
            double phi1dot = steerSign * tanDelta / L;          // dφ1/ds（弧长参数化，单位速度）

            double dTheta = degPerStep * Math.PI / 180.0;
            double ds = R * dTheta;                             // 每步弧长

            double phi2 = phi10;                                // 挂车初始与牵引车共线
            Point2d O1 = start;
            Vector2d u1 = u10;

            // ---------------- 圆弧段 ----------------
            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            int nTurn = (int)Math.Floor(totalTurn / dTheta + 1e-9);

            for (int i = 0; i <= nTurn; i++)
            {
                double ang = steerSign * i * dTheta;
                PoseOnArc(start, C, phi10, ang, out O1, out u1);
                AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);

                if (i == nTurn) break;
                phi2 += TrailerYawRate(u1, phi2, ak, phi1dot, L2) * ds;
            }

            // 补齐不足一步的残余角度，保证终点角度精确
            double rem = totalTurn - nTurn * dTheta;
            if (rem > 1e-9)
            {
                phi2 += TrailerYawRate(u1, phi2, ak, phi1dot, L2) * (R * rem);
                PoseOnArc(start, C, phi10, steerSign * totalTurn, out O1, out u1);
                AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
            }

            // ---------------- 直线段（前轮回正 φ1_dot = 0） ----------------
            if (straightLen > 1e-9)
            {
                int nS = Math.Max(1, (int)Math.Ceiling(straightLen / Math.Max(ds, 1e-6)));
                double step = straightLen / nS;
                for (int i = 0; i < nS; i++)
                {
                    phi2 += TrailerYawRate(u1, phi2, ak, 0.0, L2) * step;
                    O1 = O1 + step * u1;
                    AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                }
            }

            // 至少保证有一帧（turnAngle=0 且 straightLen=0 时）
            if (frames.Count == 0) AddFrame(frames, start, u10, phi10, ak, L2, maxArt);

            return frames;
        }

        /// <summary>
        /// 刚性单车运动学：整车作为一个刚体，后轴中心绕转弯中心 C 做精确圆周运动（半径 R=L/tanδ），
        /// 与直线段组合。无需挂车耦合，比铰接简单，但同样采用精确旋转避免累积误差。
        /// </summary>
        private static List<Frame> SimulateRigid(VehicleParams p, Point2d start, Vector2d heading, int steerSign,
                                                 double turnAngleDeg, double straightLen, double degPerStep,
                                                 double? turnRadiusOverride = null)
        {
            var frames = new List<Frame>();
            double L = p.TractorWheelbase;
            if (L <= 1e-9) return frames;

            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u10 = heading.GetNormal();
            double phi10 = Math.Atan2(u10.Y, u10.X);
            Vector2d leftNormal = new Vector2d(-u10.Y, u10.X);
            Point2d C = start + (steerSign * R) * leftNormal;   // 转弯中心

            double tanDelta = L / R;
            double phi1dot = steerSign * tanDelta / L;          // dφ1/ds（与铰接牵引车一致）
            double dTheta = degPerStep * Math.PI / 180.0;
            double ds = R * dTheta;

            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            int nTurn = (int)Math.Floor(totalTurn / dTheta + 1e-9);

            Point2d O1 = start;
            Vector2d u1 = u10;

            // ---------------- 圆弧段 ----------------
            for (int i = 0; i <= nTurn; i++)
            {
                PoseOnArc(start, C, phi10, steerSign * i * dTheta, out O1, out u1);
                AddRigidFrame(frames, O1, u1, p);
                if (i == nTurn) break;
            }
            double rem = totalTurn - nTurn * dTheta;
            if (rem > 1e-9)
            {
                PoseOnArc(start, C, phi10, steerSign * totalTurn, out O1, out u1);
                AddRigidFrame(frames, O1, u1, p);
            }

            // ---------------- 直线段（刚体平移，朝向不变） ----------------
            if (straightLen > 1e-9)
            {
                int nS = Math.Max(1, (int)Math.Ceiling(straightLen / Math.Max(ds, 1e-6)));
                double step = straightLen / nS;
                for (int i = 0; i < nS; i++)
                {
                    O1 = O1 + step * u1;
                    AddRigidFrame(frames, O1, u1, p);
                }
            }

            if (frames.Count == 0) AddRigidFrame(frames, start, u10, p);
            return frames;
        }

        /// <summary>
        /// 路径跟随（对应 AutoTURN 的 Adaptation 模式）：给定「后轴中心参考线」的密集采样点序列（按行进顺序排列），
        /// 牵引车后轴精确沿该线行进（朝向 = 该点切线），挂车按无侧滑约束滞后（off-tracking），自然产生外摆。
        /// 用于让车辆沿已有道路中线/边线自适应生成扫掠包络。
        /// 刚性单车直接按点放置（整体刚体）；铰接车据此传播挂车航向角。
        /// </summary>
        public static List<Frame> SimulateAlongPath(VehicleParams p, List<Point2d> rearPath)
        {
            var frames = new List<Frame>();
            double maxArt = p.MaxArticulationAngleDeg;
            int n = rearPath.Count;
            if (n == 0) return frames;
            if (n == 1)
            {
                Vector2d u0 = new Vector2d(1, 0);
                if (!p.Articulated) AddRigidFrame(frames, rearPath[0], u0, p);
                else AddFrame(frames, rearPath[0], u0, 0.0, p.TractorRearToKingpin, p.TrailerKingpinToRearAxle, maxArt);
                return frames;
            }

            // 逐点切线（局部邻居法向）
            var u = new Vector2d[n];
            for (int i = 0; i < n; i++)
            {
                Point2d a = rearPath[Math.Max(0, i - 1)];
                Point2d b = rearPath[Math.Min(n - 1, i + 1)];
                Vector2d t = b - a;
                if (t.Length < 1e-9) t = (i > 0) ? u[i - 1] : new Vector2d(1, 0);
                u[i] = t.GetNormal();
            }

            if (!p.Articulated)
            {
                for (int i = 0; i < n; i++) AddRigidFrame(frames, rearPath[i], u[i], p);
                return frames;
            }

            double ak = p.TractorRearToKingpin, L2 = p.TrailerKingpinToRearAxle;
            double phi2 = Math.Atan2(u[0].Y, u[0].X); // 初始挂车与牵引车共线（已拉直）
            double maxYawStep = 15.0 * Math.PI / 180.0; // 防尖锐折角导致数值爆裂

            for (int i = 0; i < n; i++)
            {
                Point2d O1 = rearPath[i];
                Vector2d u1 = u[i];
                AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                if (i < n - 1)
                {
                    double ang = Math.Atan2(u[i + 1].Y, u[i + 1].X) - Math.Atan2(u[i].Y, u[i].X);
                    ang = NormalizeAngle(ang);
                    double segLen = (rearPath[i + 1] - rearPath[i]).Length;
                    double kappa = segLen > 1e-9 ? ang / segLen : 0.0;
                    double dPhi = TrailerYawRate(u1, phi2, ak, kappa, L2) * segLen;
                    if (dPhi > maxYawStep) dPhi = maxYawStep;
                    else if (dPhi < -maxYawStep) dPhi = -maxYawStep;
                    phi2 += dPhi;
                }
            }
            return frames;
        }

        /// <summary>
        /// 从任意初始状态（后轴位置/车头朝向/挂车航向）开始模拟一段「转弯 + 直行」。
        /// 用于多段机动（前进 + 倒车串联）。dir=+1 前进，dir=-1 后退：
        /// 后退时后轴沿 -u1 移动、航向变化方向取反、挂车滞后方向相反（整条运动学镜像）。
        /// 退化的纯直行（turnAngleDeg=0）只做平移 + 挂车拉直。
        /// </summary>
        public static List<Frame> SimulateFromState(VehicleParams p, Point2d startO1, Vector2d startU1, double startPhi2,
                                                    int steerSign, double turnAngleDeg, double straightLen, int dir, double degPerStep,
                                                    double? turnRadiusOverride = null)
        {
            if (!p.Articulated)
                return SimulateRigidFromState(p, startO1, startU1, steerSign, turnAngleDeg, straightLen, dir, degPerStep, turnRadiusOverride);

            var frames = new List<Frame>();
            double L = p.TractorWheelbase, ak = p.TractorRearToKingpin, L2 = p.TrailerKingpinToRearAxle;
            double maxArt = p.MaxArticulationAngleDeg;
            if (L <= 1e-9 || L2 <= 1e-9) return frames;
            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u10 = startU1.GetNormal();
            double phi10 = Math.Atan2(u10.Y, u10.X);
            Vector2d leftNormal = new Vector2d(-u10.Y, u10.X);
            Point2d C = startO1 + (steerSign * R) * leftNormal;

            double tanDelta = L / R;
            double dTheta = degPerStep * Math.PI / 180.0;
            // v4.5.2 修复：同上 SimulateFromFrontAxle：自适应 R→+∞ 时 ds=R·dTheta 也→+∞，
            // phi2 单步 Euler 严重过冲（delta=-2.4°、straightLen=60m、nS=1 时单步 +13°）。
            // 把 ds 钳到 ≤0.5m，既保证 Euler 稳定（step≪2·L2=22m）又保证精度（误差 ≤0.7°）。
            double ds = Math.Min(R * dTheta, 0.5);

            double phi2 = startPhi2;
            Point2d O1 = startO1; Vector2d u1 = u10;

            // 圆弧段（绕 C 的绕行方向随 dir 取反，即后退沿相反方向绕行同一圆）
            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            int nTurn = (int)Math.Floor(totalTurn / dTheta + 1e-9);
            for (int i = 0; i <= nTurn; i++)
            {
                double theta = dir * steerSign * i * dTheta;
                PoseOnArc(startO1, C, phi10, theta, out O1, out u1);
                AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                if (i == nTurn) break;
                double phi1dotSigned = dir * steerSign * tanDelta / L;
                double dsSigned = dir * ds;
                phi2 += TrailerYawRate(u1, phi2, ak, phi1dotSigned, L2) * dsSigned;
            }
            // 残余角度
            double rem = totalTurn - nTurn * dTheta;
            if (rem > 1e-9)
            {
                double phi1dotSigned = dir * steerSign * tanDelta / L;
                double dsSigned = dir * (R * rem);
                phi2 += TrailerYawRate(u1, phi2, ak, phi1dotSigned, L2) * dsSigned;
                PoseOnArc(startO1, C, phi10, dir * steerSign * totalTurn, out O1, out u1);
                AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
            }
            // 直行段（前轮回正，φ1dot=0；后退沿 -u1 平移）
            if (straightLen > 1e-9)
            {
                int nS = Math.Max(1, (int)Math.Ceiling(straightLen / Math.Max(ds, 1e-6)));
                double step = straightLen / nS;
                for (int i = 0; i < nS; i++)
                {
                    double dsSigned = dir * step;
                    phi2 += TrailerYawRate(u1, phi2, ak, 0.0, L2) * dsSigned;
                    O1 = O1 + dir * step * u1;
                    AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                }
            }
            if (frames.Count == 0) AddFrame(frames, startO1, u10, phi2, ak, L2, maxArt);
            return frames;
        }

        /// <summary>
        /// 由当前状态（后轴中心 O1、航向 U1）到目标点 target 的「圆弧 + 切线直线」反解。
        /// 转向方向由 steerSign 决定（+1 左转，-1 右转）。返回 turnAngleDeg 与 straightLen（CAD 单位）。
        /// 若目标在最小转弯圆内，退化为「转向目标方向后停在最近可达点」（straightLen=0）。
        /// </summary>
        public static bool SolveFromStateToTarget(VehicleParams p, Point2d startO1, Vector2d startU1, int steerSign,
                                                  Point2d target, out double turnAngleDeg, out double straightLen)
        {
            turnAngleDeg = 0.0;
            straightLen = 0.0;
            double L = p.TractorWheelbase;
            double delta = p.SteerAngleDeg * Math.PI / 180.0;
            if (L <= 1e-9 || Math.Abs(delta) < 1e-9) return false;

            double R = L / Math.Tan(delta);
            Vector2d u10 = startU1.GetNormal();
            double phi10 = Math.Atan2(u10.Y, u10.X);
            Vector2d leftNormal = new Vector2d(-u10.Y, u10.X);
            Vector2d nInward = (steerSign >= 0 ? 1.0 : -1.0) * leftNormal;
            Point2d C = startO1 + R * nInward;
            Vector2d W = target - C;
            double a = W.X * u10.X + W.Y * u10.Y;                 // W·U1
            double b = -(W.X * nInward.X + W.Y * nInward.Y);    // -W·nInward
            double mag = Math.Sqrt(a * a + b * b);

            const double MaxTurn = 270.0 * Math.PI / 180.0;
            if (mag >= R - 1e-9)
            {
                double phi = Math.Atan2(b, a);
                double ratio = Math.Min(1.0, R / mag);
                double alpha1 = Math.Asin(ratio) - phi;
                double alpha2 = Math.PI - Math.Asin(ratio) - phi;
                double bestAlpha = double.NaN;
                double bestTotal = double.MaxValue;

                foreach (double alpha in new[] { alpha1, alpha2 })
                {
                    if (alpha < -1e-9 || alpha > MaxTurn) continue;
                    Point2d P;
                    Vector2d Ua;
                    PoseOnArc(startO1, C, phi10, steerSign * alpha, out P, out Ua);
                    Vector2d toTarget = target - P;
                    double s = toTarget.Length;
                    if (s < 1e-9) { bestAlpha = alpha; bestTotal = alpha * R; break; }
                    // 直行必须沿 Ua 指向前方
                    double cross = Ua.X * toTarget.Y - Ua.Y * toTarget.X;
                    if (Math.Abs(cross) > 1e-3 * s) continue;
                    if (Ua.X * toTarget.X + Ua.Y * toTarget.Y < -1e-6) continue;
                    double total = alpha * R + s;
                    if (total < bestTotal) { bestTotal = total; bestAlpha = alpha; }
                }

                if (!double.IsNaN(bestAlpha))
                {
                    turnAngleDeg = bestAlpha * 180.0 / Math.PI;
                    Point2d P;
                    Vector2d Ua;
                    PoseOnArc(startO1, C, phi10, steerSign * bestAlpha, out P, out Ua);
                    straightLen = (target - P).Length;
                    return true;
                }
            }

            // 目标在转弯圆内：退化为朝目标方向转到尽可能接近，然后直行距离 0
            Vector2d toT = target - startO1;
            double dist = toT.Length;
            if (dist < 1e-9) return true;
            Vector2d targetDir = toT.GetNormal();
            double cosA = targetDir.X * u10.X + targetDir.Y * u10.Y;
            double sinA = targetDir.X * nInward.X + targetDir.Y * nInward.Y;
            double alphaFb = Math.Atan2(sinA, cosA);
            if (alphaFb < 0) alphaFb += 2.0 * Math.PI;
            if (alphaFb > MaxTurn) alphaFb = MaxTurn;
            turnAngleDeg = alphaFb * 180.0 / Math.PI;
            straightLen = 0.0;
            return true;
        }

        /// <summary>刚性单车版：从任意初始状态开始一段「转弯 + 直行」，支持前进/后退（dir）。</summary>
        private static List<Frame> SimulateRigidFromState(VehicleParams p, Point2d startO1, Vector2d startU1,
                                                         int steerSign, double turnAngleDeg, double straightLen, int dir, double degPerStep,
                                                         double? turnRadiusOverride = null)
        {
            var frames = new List<Frame>();
            double L = p.TractorWheelbase;
            if (L <= 1e-9) return frames;
            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;
            Vector2d u10 = startU1.GetNormal();
            double phi10 = Math.Atan2(u10.Y, u10.X);
            Vector2d leftNormal = new Vector2d(-u10.Y, u10.X);
            Point2d C = startO1 + (steerSign * R) * leftNormal;
            double dTheta = degPerStep * Math.PI / 180.0;
            double ds = R * dTheta;
            Point2d O1 = startO1; Vector2d u1 = u10;

            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            int nTurn = (int)Math.Floor(totalTurn / dTheta + 1e-9);
            for (int i = 0; i <= nTurn; i++)
            {
                double theta = dir * steerSign * i * dTheta;
                PoseOnArc(startO1, C, phi10, theta, out O1, out u1);
                AddRigidFrame(frames, O1, u1, p);
                if (i == nTurn) break;
            }
            double rem = totalTurn - nTurn * dTheta;
            if (rem > 1e-9)
            {
                PoseOnArc(startO1, C, phi10, dir * steerSign * totalTurn, out O1, out u1);
                AddRigidFrame(frames, O1, u1, p);
            }
            if (straightLen > 1e-9)
            {
                int nS = Math.Max(1, (int)Math.Ceiling(straightLen / Math.Max(ds, 1e-6)));
                double step = straightLen / nS;
                for (int i = 0; i < nS; i++)
                {
                    O1 = O1 + dir * step * u1;
                    AddRigidFrame(frames, O1, u1, p);
                }
            }
            if (frames.Count == 0) AddRigidFrame(frames, startO1, u10, p);
            return frames;
        }

        /// <summary>牵引车后轴中心的转弯半径 R = L / tanδ。可传入 overrideRadius 覆盖（用于自适应圆弧）。</summary>
        public static double TurnRadius(VehicleParams p, double? overrideRadius = null)
        {
            if (overrideRadius.HasValue)
            {
                double r = overrideRadius.Value;
                if (r < 1e-6) r = 1e-6;
                return r;
            }
            double delta = Math.Abs(p.SteerAngleDeg) * Math.PI / 180.0;
            // 夹住极端角度，避免 tan 溢出或半径为 0
            if (delta < 0.05 * Math.PI / 180.0) delta = 0.05 * Math.PI / 180.0;
            if (delta > 75.0 * Math.PI / 180.0) delta = 75.0 * Math.PI / 180.0;
            return p.TractorWheelbase / Math.Tan(delta);
        }

        /// <summary>转弯中心</summary>
        public static Point2d TurnCenter(VehicleParams p, Point2d start, Vector2d heading, int steerSign)
        {
            Vector2d u10 = heading.GetNormal();
            Vector2d leftNormal = new Vector2d(-u10.Y, u10.X);
            return start + (steerSign * TurnRadius(p)) * leftNormal;
        }

        /// <summary>
        /// 由目标点反解路径参数：路径 = 半径 R 的圆弧 + 其切线直线，
        /// 因此从转弯中心 C 到目标点 T 的距离 D 满足 D² = R² + straightLen²（切线长），
        /// 圆弧转角 = ∠(起点→目标点) − atan(straightLen / R)。
        /// 当 D ≤ R（目标点落在转弯圆内）时退化为纯圆弧转弯。
        /// </summary>
        public static void SolveFromTarget(VehicleParams p, Point2d start, Vector2d heading, int steerSign,
                                           Point2d target, out double turnAngleDeg, out double straightLen)
        {
            double R = TurnRadius(p);
            Vector2d u10 = heading.GetNormal();
            Point2d C = TurnCenter(p, start, heading, steerSign);

            Vector2d vs = start - C;
            Vector2d vt = target - C;
            double D = vt.Length;

            // 沿转向方向累计的角度差，归一到 [0, 2π)
            double a0 = Math.Atan2(vs.Y, vs.X);
            double at = Math.Atan2(vt.Y, vt.X);
            double dphi = steerSign * (at - a0);
            dphi = Norm2Pi(dphi);

            if (D <= R + 1e-9)
            {
                // 目标点在转弯圆内：纯圆弧
                straightLen = 0.0;
                turnAngleDeg = dphi * 180.0 / Math.PI;
            }
            else
            {
                double tangentLen = Math.Sqrt(Math.Max(0.0, D * D - R * R));
                double alpha = Math.Atan2(tangentLen, R);   // 切点相对目标点的圆心角偏移
                double theta = dphi - alpha;

                if (theta < 0.0)
                {
                    // 目标点还没进入弯道（位于车头前方/侧后方）：视为纯直行
                    turnAngleDeg = 0.0;
                    double proj = (target - start).X * u10.X + (target - start).Y * u10.Y;
                    straightLen = Math.Max(0.0, proj);
                }
                else
                {
                    turnAngleDeg = theta * 180.0 / Math.PI;
                    straightLen = tangentLen;
                }
            }

            if (turnAngleDeg > 359.0) turnAngleDeg = 359.0;   // 不允许超过一整圈
            if (turnAngleDeg < 0.0) turnAngleDeg = 0.0;
            if (straightLen < 0.0) straightLen = 0.0;
        }

        #region 内部
        /// <summary>圆弧上某转角处的位姿（绕 C 精确旋转，无累积误差）</summary>
        private static void PoseOnArc(Point2d start, Point2d C, double phi10, double signedAng,
                                      out Point2d O1, out Vector2d u1)
        {
            double phi1 = phi10 + signedAng;
            u1 = new Vector2d(Math.Cos(phi1), Math.Sin(phi1));

            Vector2d rel = start - C;
            double ca = Math.Cos(signedAng), sa = Math.Sin(signedAng);
            O1 = C + new Vector2d(rel.X * ca - rel.Y * sa, rel.X * sa + rel.Y * ca);
        }

        /// <summary>
        /// 挂车航向角变化率（对弧长求导），来自鞍座处的无侧滑约束：
        /// dφ2/ds = [ (u1·n2) + a·(dφ1/ds)·(n1·n2) ] / L2
        /// </summary>
        private static double TrailerYawRate(Vector2d u1, double phi2, double ak, double phi1dot, double L2)
        {
            Vector2d u2 = new Vector2d(Math.Cos(phi2), Math.Sin(phi2));
            Vector2d n1 = new Vector2d(-u1.Y, u1.X);
            Vector2d n2 = new Vector2d(-u2.Y, u2.X);
            double u1n2 = u1.X * n2.X + u1.Y * n2.Y;
            double n1n2 = n1.X * n2.X + n1.Y * n2.Y;
            return (u1n2 + ak * phi1dot * n1n2) / L2;
        }

        private static void AddFrame(List<Frame> frames, Point2d O1, Vector2d u1, double phi2, double ak, double L2,
                                     double maxArticulationDeg = double.MaxValue)
        {
            // v4.4：限制最大铰接角，防止挂车折叠侵入驾驶室
            if (maxArticulationDeg < 180.0 && maxArticulationDeg > 0.0)
            {
                double phi1 = Math.Atan2(u1.Y, u1.X);
                double delta = NormalizeAngle(phi2 - phi1);
                double maxRad = maxArticulationDeg * Math.PI / 180.0;
                if (delta > maxRad) phi2 = phi1 + maxRad;
                else if (delta < -maxRad) phi2 = phi1 - maxRad;
            }

            Vector2d u2 = new Vector2d(Math.Cos(phi2), Math.Sin(phi2));
            Point2d K = O1 + ak * u1;
            Point2d O2 = K - L2 * u2;
            frames.Add(new Frame { O1 = O1, U1 = u1, K = K, U2 = u2, O2 = O2 });
        }

        /// <summary>刚性单车：整体刚体，U2=U1，O2=前轴中心（用于轮径层绘制）</summary>
        private static void AddRigidFrame(List<Frame> frames, Point2d O1, Vector2d u1, VehicleParams p)
        {
            Point2d O2 = O1 + p.TractorWheelbase * u1;   // 前轴中心
            frames.Add(new Frame { O1 = O1, U1 = u1, K = O2, U2 = u1, O2 = O2 });
        }

        private static double Norm2Pi(double a)
        {
            double twoPi = 2.0 * Math.PI;
            a = a % twoPi;
            if (a < 0) a += twoPi;
            return a;
        }

        /// <summary>把角度归一到 (-π, π]</summary>
        private static double NormalizeAngle(double a)
        {
            while (a > Math.PI) a -= 2.0 * Math.PI;
            while (a < -Math.PI) a += 2.0 * Math.PI;
            return a;
        }
        #endregion

        /// <summary>四种转弯半径（用于出图标注/报告，对应 AutoTURN 的 turning template）</summary>
        public struct TurnRadiusSet
        {
            public double Centerline; // R1 = L/tanδ，后轴中心
            public double CurbToCurb; // 前轮外缘 = L/sinδ
            public double WallToWall; // 车身外角（前悬外角）到转弯中心
            public double InnerTire;  // 后内轮 = R1 − 轮距/2
        }

        /// <summary>计算四种转弯半径（解析公式，无需整条路径）</summary>
        public static TurnRadiusSet TurnRadii(VehicleParams p)
        {
            double R1 = TurnRadius(p);
            double delta = Math.Abs(p.SteerAngleDeg) * Math.PI / 180.0;
            if (delta < 0.05 * Math.PI / 180.0) delta = 0.05 * Math.PI / 180.0;
            double L = p.TractorWheelbase;
            double Rcurb = L / Math.Sin(delta);
            double fh = p.TractorFrontOverhang;
            double w = p.TractorWidth;
            double Rwall = Math.Sqrt((L + fh) * (L + fh) + (R1 + w / 2.0) * (R1 + w / 2.0));
            double Rinner = R1 - p.WheelTrack / 2.0;
            return new TurnRadiusSet { Centerline = R1, CurbToCurb = Rcurb, WallToWall = Rwall, InnerTire = Rinner };
        }

        /// <summary>返回某帧的关键检查点（外摆点）：命名角点，沿路径追踪其最小净距。
        /// 刚性单车取整车四角；铰接车另加挂车尾两角（off-tracking 使其外摆最大）。</summary>
        public static List<(string name, Point2d pt)> CheckpointPoints(Frame f, VehicleParams p)
        {
            var res = new List<(string, Point2d)>();
            Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);
            if (!p.Articulated)
            {
                var rp = Corners(f, p);
                res.Add(("前左", rp[0]));
                res.Add(("前右", rp[1]));
                res.Add(("后左", rp[2]));
                res.Add(("后右", rp[3]));
            }
            else
            {
                double wt = p.TractorWidth, wb = p.TrailerWidth;
                double fx = p.TractorWheelbase + p.TractorFrontOverhang;
                double rx = p.TractorRearToKingpin;
                double bx = -(p.TrailerKingpinToRearAxle + p.TrailerRearOverhang);
                Vector2d n2 = new Vector2d(-f.U2.Y, f.U2.X);
                res.Add(("主车前左", f.O1 + fx * f.U1 + (wt / 2) * n1));
                res.Add(("主车前右", f.O1 + fx * f.U1 - (wt / 2) * n1));
                res.Add(("主车后左", f.O1 + rx * f.U1 + (wt / 2) * n1));
                res.Add(("主车后右", f.O1 + rx * f.U1 - (wt / 2) * n1));
                res.Add(("挂车尾左", f.K + bx * f.U2 + (wb / 2) * n2));
                res.Add(("挂车尾右", f.K + bx * f.U2 - (wb / 2) * n2));
            }
            return res;
        }

        /// <summary>返回某帧全部 8 个车体角点：0-3 牵引车(FL,FR,RL,RR)，4-7 挂车(FLt,FRt,RLt,RRt)。
        /// 刚性单车模式下两块矩形重合（整车即一个刚体），凸包与绘制均兼容。</summary>
        public static Point2d[] Corners(Frame f, VehicleParams p)
        {
            Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);
            Vector2d n2 = new Vector2d(-f.U2.Y, f.U2.X);

            if (!p.Articulated)
            {
                // 刚性单车：单一矩形（后轴为参考点）
                double bfx = p.TractorWheelbase + p.TractorFrontOverhang; // 后轴到车头
                double brx = -p.RigidRearOverhang;                         // 后轴到车尾
                double w = p.TractorWidth;
                var rp = new Point2d[8];
                rp[0] = f.O1 + bfx * f.U1 + (w / 2) * n1; // 前左
                rp[1] = f.O1 + bfx * f.U1 - (w / 2) * n1; // 前右
                rp[2] = f.O1 + brx * f.U1 + (w / 2) * n1; // 后左
                rp[3] = f.O1 + brx * f.U1 - (w / 2) * n1; // 后右
                rp[4] = rp[0]; rp[5] = rp[1]; rp[6] = rp[3]; rp[7] = rp[2]; // 复制供 TrailerRect 复用
                return rp;
            }

            double wt = p.TractorWidth, wb = p.TrailerWidth;
            double fx = p.TractorWheelbase + p.TractorFrontOverhang; // 后轴到车头
            // v4.4：牵引车矩形 = 驾驶室（宽体部分）。驾驶室后壁由挂车前角最大摆幅反推，
            //       保证任意铰接角下挂车都不可能碰到驾驶室。驾驶室后方的窄车架见 ChassisRect。
            double rx = p.CabRearX();
            double tf = p.TrailerKingpinToFront;                         // 鞍座到挂车前端（向前）
            double bx = -(p.TrailerKingpinToRearAxle + p.TrailerRearOverhang); // 鞍座到车尾(挂车，从 kingpin 量)

            var pts = new Point2d[8];
            // 牵引车（驾驶室）
            pts[0] = f.O1 + fx * f.U1 + (wt / 2) * n1; // 前左
            pts[1] = f.O1 + fx * f.U1 - (wt / 2) * n1; // 前右
            pts[2] = f.O1 + rx * f.U1 + (wt / 2) * n1; // 后左
            pts[3] = f.O1 + rx * f.U1 - (wt / 2) * n1; // 后右
            // 挂车（前端从 kingpin 向前 tf，后端从 kingpin 向后 |bx|）
            pts[4] = f.K + tf * f.U2 + (wb / 2) * n2;              // 前左
            pts[5] = f.K + tf * f.U2 - (wb / 2) * n2;              // 前右
            pts[6] = f.K + (tf + bx) * f.U2 + (wb / 2) * n2;       // 后左(车尾)
            pts[7] = f.K + (tf + bx) * f.U2 - (wb / 2) * n2;       // 后右
            return pts;
        }

        /// <summary>牵引车矩形(4 角，顺序 FL,FR,RR,RL) 供绘制</summary>
        public static Point2d[] TractorRect(Frame f, VehicleParams p)
        {
            var c = Corners(f, p);
            return new[] { c[0], c[1], c[3], c[2] };
        }

        /// <summary>挂车矩形(4 角，顺序 FLt,FRt,RRt,RLt) 供绘制</summary>
        public static Point2d[] TrailerRect(Frame f, VehicleParams p)
        {
            var c = Corners(f, p);
            return new[] { c[4], c[5], c[7], c[6] };
        }

        /// <summary>
        /// 牵引车底盘纵梁矩形（仅铰接车有，刚性车返回 null）。
        ///
        /// 真车驾驶室后方只有窄车架，挂车前部悬在车架上方，因此挂车与车架在平面图里重叠是正常且正确的；
        /// 真正不能接触的是驾驶室（由 <see cref="VehicleParams.CabRearX"/> 从构造上保证）。
        /// 画出这条窄车架，驾驶室与挂车之间才不会出现一眼假的空洞。
        /// 顺序：前左、前右、后右、后左。
        /// </summary>
        public static Point2d[] ChassisRect(Frame f, VehicleParams p)
        {
            if (!p.Articulated) return null;
            double xFront = p.CabRearX();
            double xRear = -p.ChassisRearOverhang;
            if (xFront <= xRear) return null;
            double w = p.ChassisWidth;
            if (w > p.TractorWidth) w = p.TractorWidth;
            if (w <= 1e-6) return null;
            Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);
            return new[]
            {
                f.O1 + xFront * f.U1 + (w / 2) * n1,
                f.O1 + xFront * f.U1 - (w / 2) * n1,
                f.O1 + xRear  * f.U1 - (w / 2) * n1,
                f.O1 + xRear  * f.U1 + (w / 2) * n1
            };
        }

        /// <summary>当前帧牵引车前轴中心</summary>
        public static Point2d FrontAxle(Frame f, VehicleParams p)
        {
            return f.O1 + p.TractorWheelbase * f.U1;
        }

        /// <summary>
        /// v4.9.17：刚性单车的精致车体轮廓（**仅用于绘制**，包络/轮迹仍走 <see cref="Corners"/> 矩形，不受影响）。
        /// 原来刚性车预览/出图 = 一个矩形画两遍（TractorRect 与 TrailerRect 重合）+ 车轮，
        /// 看不出车头车厢。本函数按真车平面特征拆成：
        ///   ① 车头（驾驶室）：前部带挡风玻璃斜面的六边形（鼻端宽 86%，斜面段占车头长 35%）；
        ///   ② 挡风玻璃线：斜面根部的横向线（发动机盖与驾驶室的分界）；
        ///   ③ 后视镜：斜面根部向外的两小段（0.22m）；
        ///   ④ 车厢：驾驶室后壁留 0.12m 缝隙起的矩形货厢。
        /// 车头长 ≈ 25% 车长，钳制在 1.2~2.6m（微卡不至于太长、重卡不至于太短）。
        /// 返回若干段折线，闭合形状已把首点追加到末尾，统一按「开折线」画即可。
        /// 局部坐标：x 向前（后轴为原点），y 向左；单位与图纸一致（内部用 UnitScale 换算米常量）。
        /// </summary>
        public static List<Point2d[]> RigidBodyOutlines(Frame f, VehicleParams p)
        {
            var res = new List<Point2d[]>();
            if (p.Articulated) return res;   // 铰接车沿用 驾驶室+车架+挂车 的既有画法

            double us = p.UnitScale > 1e-9 ? p.UnitScale : 1.0;
            double L = p.TractorWheelbase, fh = p.TractorFrontOverhang, ro = p.RigidRearOverhang;
            double total = L + fh + ro;
            if (total < 1e-6) return res;

            double xF = L + fh;      // 前保险杠
            double xR = -ro;         // 后保险杠
            double wH = p.TractorWidth / 2.0;

            // 车头长：25% 车长，钳 1.2~2.6m，且不超过车身的 45%
            double cabLen = 0.25 * total;
            double cabMin = 1.2 * us, cabMax = 2.6 * us;
            if (cabLen < cabMin) cabLen = cabMin;
            if (cabLen > cabMax) cabLen = cabMax;
            if (cabLen > 0.45 * total) cabLen = 0.45 * total;
            double xCab = xF - cabLen;                    // 驾驶室后壁
            double slope = 0.35 * cabLen;                 // 挡风玻璃斜面段长
            double wNose = wH * 0.86;                     // 鼻端半宽
            double gap = 0.12 * us;                       // 驾驶室与车厢缝隙
            double mirror = 0.22 * us;                    // 后视镜外伸

            Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);
            Point2d P(double x, double y) => f.O1 + x * f.U1 + y * n1;

            // ① 车头六边形（闭合）：鼻端窄、斜面后全宽
            res.Add(new[]
            {
                P(xF, +wNose), P(xF - slope, +wH), P(xCab, +wH),
                P(xCab, -wH), P(xF - slope, -wH), P(xF, -wNose),
                P(xF, +wNose)
            });
            // ② 挡风玻璃线（斜面根部横向）
            res.Add(new[] { P(xF - slope, +wH), P(xF - slope, -wH) });
            // ③ 后视镜
            res.Add(new[] { P(xF - slope, +wH), P(xF - slope, +wH + mirror) });
            res.Add(new[] { P(xF - slope, -wH), P(xF - slope, -wH - mirror) });
            // ④ 车厢（闭合，驾驶室后壁留缝）
            double xBox = xCab - gap;
            if (xBox > xR + 1e-6)
                res.Add(new[] { P(xBox, +wH), P(xBox, -wH), P(xR, -wH), P(xR, +wH), P(xBox, +wH) });
            return res;
        }

        /// <summary>将一帧的牵引车前轴中心强制调整到指定位置（保持朝向/挂车航向不变）</summary>
        public static Frame WithFrontAt(this Frame f, VehicleParams p, Point2d front)
        {
            Vector2d u = f.U1;
            double L = p.TractorWheelbase;
            Point2d O1 = front - L * u;
            Point2d K = O1 + p.TractorRearToKingpin * u;
            Point2d O2 = K - p.TrailerKingpinToRearAxle * f.U2;
            return new Frame { O1 = O1, U1 = u, K = K, U2 = f.U2, O2 = O2 };
        }

        /// <summary>
        /// 由目标点（前轴中心）反解路径参数。实现：把前轴目标按当前/最终车头方向反算为后轴目标，
        /// 复用成熟的后轴反解迭代校正（通常 2~3 次收敛），保证前轴终点精确落在目标上。
        /// </summary>
        public static void SolveFromTargetFront(VehicleParams p, Point2d startFront, Vector2d heading, int steerSign,
                                                 Point2d targetFront, out double turnAngleDeg, out double straightLen)
        {
            turnAngleDeg = 0.0;
            straightLen = 0.0;
            double L = p.TractorWheelbase;
            double delta = Math.Abs(p.SteerAngleDeg) * Math.PI / 180.0;
            if (L <= 1e-9 || delta < 1e-9) return;

            Vector2d u0 = heading.GetNormal();
            Point2d O0 = startFront - L * u0;
            Vector2d dir = (targetFront - startFront).GetNormal();
            if (dir.Length < 1e-9) dir = u0;

            for (int iter = 0; iter < 8; iter++)
            {
                Point2d O_target = targetFront - L * dir;
                SolveFromTarget(p, O0, u0, steerSign, O_target, out turnAngleDeg, out straightLen);
                var frames = Simulate(p, O0, u0, steerSign, turnAngleDeg, straightLen, 1.0);
                if (frames.Count == 0) break;
                Vector2d uEnd = frames[frames.Count - 1].U1;
                if ((uEnd - dir).Length < 1e-9) break;
                dir = uEnd;
            }
        }

        /// <summary>
        /// 由当前状态（前轴中心/车头朝向）到目标前轴中心的反解。同样采用后轴反解迭代校正。
        /// </summary>
        public static bool SolveFromStateToTargetFront(VehicleParams p, Point2d startFront, Vector2d startU1, int steerSign,
                                                        Point2d targetFront, out double turnAngleDeg, out double straightLen)
        {
            turnAngleDeg = 0.0;
            straightLen = 0.0;
            double L = p.TractorWheelbase;
            double delta = Math.Abs(p.SteerAngleDeg) * Math.PI / 180.0;
            if (L <= 1e-9 || delta < 1e-9) return false;

            Point2d O1 = startFront - L * startU1.GetNormal();
            Vector2d dir = (targetFront - startFront).GetNormal();
            if (dir.Length < 1e-9) dir = startU1.GetNormal();
            double phi2 = Math.Atan2(startU1.Y, startU1.X);

            for (int iter = 0; iter < 8; iter++)
            {
                Point2d O_target = targetFront - L * dir;
                bool ok = SolveFromStateToTarget(p, O1, startU1, steerSign, O_target, out turnAngleDeg, out straightLen);
                if (!ok) return false;
                var frames = SimulateFromState(p, O1, startU1, phi2, steerSign, turnAngleDeg, straightLen, 1, 1.0);
                if (frames.Count == 0) break;
                Vector2d uEnd = frames[frames.Count - 1].U1;
                if ((uEnd - dir).Length < 1e-9) break;
                dir = uEnd;
            }
            return true;
        }

        /// <summary>
        /// 自适应半径单圆弧反解（AutoTURN 式"转弯到目标"）。
        /// 给定起点前轴 S、起点航向 u、终点前轴 T，找一条后轴中心圆弧（起点与 u 相切），
        /// 使得刚体上的前轴恰好经过 T。半径随目标自适应：目标越远越缓、越近越急。
        /// 若所需后轴半径 ≥ 最小转弯半径，则返回纯圆弧（straightLen=0）与对应的后轴半径；
        /// 否则返回 false，调用方回退到经典的 arc+straight。
        /// </summary>
        /// <summary>
        /// v4.9：从起点前轴「过目标前轴」的自适应单圆弧反解，支持前进(dir=+1)/倒退(dir=-1)。
        ///
        /// 几何（前进/倒退通用）：瞬心 C 恒在 steerSign 侧 —— C = O0 + steerSign·R·n，
        /// 因为前轮往哪边打，瞬心就在哪边，与车往哪个方向走无关。
        /// 差别只在绕 C 的行进方向：前进沿 +steerSign 绕，倒退沿 −steerSign 绕。
        ///   R      = (D² + 2·L·a) / (2·d·steerSign)      ← 与 dir 无关
        ///   可行性 : R &gt; 0（由分子分母同号自动筛掉不可行的那一侧）
        ///            R ≥ Rmin
        ///            dir·steerSign·θ ≥ 0（θ = 圆心处 起点→目标 的有符号夹角）
        ///
        /// 倒车的两个反直觉之处（已实测验算，勿"修正"）：
        ///  · 前进时目标必须在转向同侧（d·steerSign&gt;0），倒退时常态是目标在转向<b>对侧</b>。
        ///    例：L=5、Rmin=8.66，倒车左打转 30°，前轴从 (0,0) 走到 (−5, −1.34)，
        ///    即"左打方向盘、前轴向车身右后方走"—— 因为倒车时前轮沿其滚动方向的反向走。
        ///  · 因此 steerSign==0（自动判向）时两侧都要试，取转角小的一侧。
        ///    前进时另一侧会被 R&lt;0 自动排除，结果与旧的单侧逻辑完全一致。
        /// </summary>
        public static bool TrySolveFromTargetFrontAdaptive(VehicleParams p, Point2d startFront, Vector2d heading, int steerSign,
                                                           Point2d targetFront, out double turnAngleDeg, out double straightLen,
                                                           out double rearRadius, int dir = 1)
        {
            turnAngleDeg = 0.0;
            straightLen = 0.0;
            rearRadius = 0.0;
            double L = p.TractorWheelbase;
            if (L <= 1e-9) return false;

            Vector2d u = heading.GetNormal();
            Vector2d n = new Vector2d(-u.Y, u.X);                // 左法向
            Vector2d diff = targetFront - startFront;
            double a = u.X * diff.X + u.Y * diff.Y;              // 沿航向距离（倒车时为负）
            double d = n.X * diff.X + n.Y * diff.Y;              // 带符号横向偏移（左正右负）

            if (Math.Abs(d) < 1e-9)                             // 正前方（前进）/正后方（倒退）：走直线
            {
                turnAngleDeg = 0.0;
                // v4.9：dir 进到 a 里 —— 倒退时目标须在车后方（a&lt;0）才有正的行驶距离
                straightLen = Math.Max(0.0, dir * a);
                rearRadius = double.PositiveInfinity;
                return true;
            }

            double D2 = a * a + d * d;
            double Rmin = TurnRadius(p);                         // 当前最大转角决定的最小后轴半径

            // steerSign==0 → 两侧都试（自动判向）；否则只试指定侧
            int first = steerSign >= 0 ? 1 : -1;
            int[] candidates = (steerSign == 0) ? new[] { 1, -1 } : new[] { first };

            int bestSign = 0;
            double bestDeg = double.MaxValue, bestR = 0.0;

            foreach (int m in candidates)
            {
                double denom = 2.0 * d * m;
                if (Math.Abs(denom) < 1e-12) continue;

                double Rrear = (D2 + 2.0 * L * a) / denom;
                if (double.IsNaN(Rrear) || double.IsInfinity(Rrear)) continue;
                if (Rrear < Rmin - 1e-9) continue;               // 比最小半径还急，不可行

                // 后轴圆心
                Point2d O0 = startFront - L * u;
                Point2d C = O0 + (m * Rrear) * n;
                // 前轴相对圆心的起止向量夹角 = 后轴圆弧转角
                Vector2d v1 = startFront - C;
                Vector2d v2 = targetFront - C;
                double cross = v1.X * v2.Y - v1.Y * v2.X;
                double dotv = v1.X * v2.X + v1.Y * v2.Y;
                double theta = Math.Atan2(cross, dotv);          // 有符号圆心角
                // v4.9：dir 进到方向判据 —— 倒退时绕 C 的方向与前进相反
                if (dir * m * theta < -1e-9) continue;           // 几何方向与行进方向不符

                double deg = Math.Abs(theta) * 180.0 / Math.PI;
                if (deg < bestDeg) { bestDeg = deg; bestR = Rrear; bestSign = m; }
            }

            if (bestSign == 0) return false;
            turnAngleDeg = bestDeg;                              // 非负角度，方向由 steerSign 决定
            straightLen = 0.0;
            rearRadius = bestR;
            return true;
        }

        /// <summary>从任意状态开始的自适应单圆弧反解（用于 TRUCKDRIVE 连续段）。dir=+1 前进 / −1 倒退。</summary>
        public static bool TrySolveFromStateToTargetFrontAdaptive(VehicleParams p, Point2d startFront, Vector2d startU1, int steerSign,
                                                                  Point2d targetFront, out double turnAngleDeg, out double straightLen,
                                                                  out double rearRadius, int dir = 1)
        {
            return TrySolveFromTargetFrontAdaptive(p, startFront, startU1, steerSign, targetFront, out turnAngleDeg, out straightLen, out rearRadius, dir);
        }

        /// <summary>
        /// v4.9：自动判向的自适应反解（TRUCKDRIVE 连续段用）。
        ///
        /// 前进(dir=+1)：光标在车身左侧就左转，与 v4.1~v4.8 的行为完全一致
        /// （另一侧根本不可行，会被 R&lt;0 自动排除）。
        ///
        /// 倒退(dir=−1)：两侧<b>都</b>可能可行 —— 一侧是短弧，另一侧是绕过 180° 的长弧。
        /// 例：L=5、Rmin=8.66，目标在右后方 (−5,−1.34)，左打转 30° 即可到达；
        ///     同一个点用右打也能到，但要绕一大圈。所以先试「光标所在侧」，不行再试另一侧。
        /// </summary>
        public static bool TrySolveAutoSide(VehicleParams p, Point2d startFront, Vector2d u1, int dir,
                                            Point2d targetFront, out int steerSign,
                                            out double turnAngleDeg, out double straightLen, out double rearRadius)
        {
            Vector2d diff = targetFront - startFront;
            double cross = u1.X * diff.Y - u1.Y * diff.X;        // >0：目标在车身左侧
            // 前进：光标在左 → 左转。
            // 倒退：前轴绕瞬心的方向反过来，于是"左打方向盘、前轴往车身右后方走"
            // （实测 L=5、Rmin=8.66：左打倒退 30°，前轴从 (0,0) 到 (−5,−1.34)，即后方+右侧）。
            // 所以倒退时猜测方向要反过来：目标在右后方(cross<0) 才先试左打。
            // 猜错了也没关系 —— 不可行的那一侧会被 R<0 / dir·steerSign·θ<0 挡掉，接着试另一侧。
            int guess = (dir >= 0) ? (cross > 0 ? 1 : -1) : (cross > 0 ? -1 : 1);
            // 前进只试 guess 一侧（与 v4.1~v4.8 完全一致，另一侧数学上不可行）；
            // 倒退两侧都可能可行（短弧 / 绕过 180° 的长弧），guess 落空再试另一侧。
            int[] order = (dir >= 0) ? new[] { guess } : new[] { guess, -guess };

            foreach (int m in order)
            {
                if (TrySolveFromTargetFrontAdaptive(p, startFront, u1, m, targetFront,
                                                    out turnAngleDeg, out straightLen, out rearRadius, dir))
                {
                    steerSign = m;
                    return true;
                }
            }
            steerSign = guess;
            turnAngleDeg = 0.0;
            straightLen = 0.0;
            rearRadius = 0.0;
            return false;
        }

        /// <summary>
        /// v4.7：自适应单圆弧不可行（所需半径 &lt; 最小转弯半径）时的<b>连续</b>回退解 —— 沿最小半径圆尽量转向目标。
        ///
        /// 旧做法是「按最小半径硬转 90°」：只要目标近到走不出单圆弧，不管光标在哪个方向，
        /// 预览一律跳到同一个 90° 姿态。用户把光标扫到车头侧面（大致 90° 方向）时最常踩中这条分支，
        /// 表现为「指针划到大概 90° 转弯的时候会自动吸附过去」，且因为解与光标无关，
        /// 光标继续移动预览却一动不动 —— 手感很差。
        ///
        /// 这里改为：仍走最小半径圆，但转角取「圆心处 起点前轴 → 目标 的夹角」，
        /// 也就是沿最小半径圆把车头尽量转到朝向光标。
        /// 关键性质：当目标恰好落在最小半径圆上时（即自适应可行的临界位置），
        /// 圆心 C 与本函数在自适应解里的 C 完全相同，算出的角度也完全相同，
        /// 因此光标跨过可行性边界时预览是<b>连续</b>的，不再有跳变；
        /// 目标再往里挪，转角只是平滑地停在「最接近目标」的那个角度上，不会乱跳。
        ///
        /// v4.9：新增 dir 参数，倒退(dir=−1)时同样成立 —— 瞬心 C 仍在 steerSign 侧，
        /// 只是绕 C 的行进方向反过来，故方向判据由 steerSign·θ≥0 变为 dir·steerSign·θ≥0。
        ///
        /// 返回需要转动的角度（度，非负）。目标已在正前方 / 转不动时返回 0（调用方据此判定该段无效）。
        /// 上限 180°：目标绕到正后方时不做整圈，避免预览原地打转。
        /// </summary>
        public static double TurnTowardTargetAtMinRadius(VehicleParams p, Point2d startFront, Vector2d startU1,
                                                         int steerSign, Point2d targetFront, int dir = 1)
        {
            double L = p.TractorWheelbase;
            if (L <= 1e-9) return 0.0;

            double Rmin = TurnRadius(p);
            if (Rmin <= 1e-9 || double.IsNaN(Rmin) || double.IsInfinity(Rmin)) return 0.0;

            Vector2d u = startU1.GetNormal();
            Vector2d n = new Vector2d(-u.Y, u.X);                 // 左法向
            int sign = steerSign >= 0 ? 1 : -1;

            Point2d O0 = startFront - L * u;                      // 起点后轴中心
            Point2d C = O0 + (sign * Rmin) * n;                   // 最小半径圆的圆心
            Vector2d v1 = startFront - C;
            Vector2d v2 = targetFront - C;
            if (v1.Length < 1e-9 || v2.Length < 1e-9) return 0.0;

            double cross = v1.X * v2.Y - v1.Y * v2.X;
            double dotv = v1.X * v2.X + v1.Y * v2.Y;
            double theta = Math.Atan2(cross, dotv);               // 有符号圆心角

            // v4.9：倒退时绕 C 的方向与前进相反，判据须带上 dir
            if (dir * sign * theta < 0.0) return 0.0;            // 目标在行进方向的反侧 → 不转
            double deg = Math.Abs(theta) * 180.0 / Math.PI;
            const double MaxDeg = 180.0;
            if (deg > MaxDeg) deg = MaxDeg;
            return deg;
        }

        #region v4.9.9 倒退：车尾控制 + 司机修正模型

        /// <summary>
        /// v4.9.9：倒退时「车尾控制点」的最小转弯半径。
        ///
        /// 倒退的运动学改以车尾为控制点（铰接车 = 挂车后轴 O2，刚性车 = 后轴 O1）：
        /// 司机倒车时眼睛盯的是车尾，光标应该直接指挥车尾，而不是前轴。
        ///
        /// 几何约束（纯几何，与行进方向无关）：同心圆稳态下
        ///   R1² + ak² = R2² + L2²     （R1=牵引车后轴半径，R2=挂车后轴半径）
        /// 牵引车转向极限 R1 ≥ R1min ⇒ R2 ≥ sqrt(R1min² + ak² − L2²)。
        /// 半挂典型参数（R1min≈8.7、L2≈10.8）下右端为负 ⇒ 挂车几乎可原地 pivot，
        /// 返回 0（调用方需自行防退化）。
        /// </summary>
        public static double ReverseMinControlRadius(VehicleParams p)
        {
            double R1min = TurnRadius(p);
            if (!p.Articulated) return R1min;
            double ak = p.TractorRearToKingpin, L2 = p.TrailerKingpinToRearAxle;
            double v = R1min * R1min + ak * ak - L2 * L2;
            return v > 0.0 ? Math.Sqrt(v) : 0.0;
        }

        /// <summary>
        /// v4.9.9：倒退反解 —— 车尾控制点精确过光标的自适应单圆弧。
        ///
        /// 与前进的前轴反解完全同构，只是控制点换成车尾（铰接 O2 / 刚性 O1）：
        /// 过控制点、与车尾航向相切、且经过目标点的圆唯一确定（圆心在目标同侧），
        /// 不存在「左打还是右打」的猜侧问题 —— 车尾往哪边走由光标位置唯一决定，
        /// 这正好修复「鼠标指向左边、车却往右倒」的手感错位。
        ///
        /// 可行性：所需车尾半径 ≥ ReverseMinControlRadius；圆心角 ≤ 180°。
        /// 目标几乎在正后方（|d|≈0）时退化为直线倒退。
        /// </summary>
        /// <param name="side">车尾甩向：+1=左, −1=右（= 目标所在侧；方向盘实际打向相反侧 −side）</param>
        /// <param name="radius">车尾控制点半径 R2（铰接=挂车后轴，刚性=后轴）；直线时为 +∞</param>
        public static bool TrySolveReverseArc(VehicleParams p, Frame startFrame, Point2d target,
                                              out int side, out double turnAngleDeg,
                                              out double radius, out double straightLen)
        {
            side = 0; turnAngleDeg = 0.0; radius = 0.0; straightLen = 0.0;

            Point2d P0; Vector2d u;
            if (p.Articulated) { P0 = startFrame.O2; u = startFrame.U2; }
            else { P0 = startFrame.O1; u = startFrame.U1; }
            u = u.GetNormal();
            Vector2d n = new Vector2d(-u.Y, u.X);
            Vector2d diff = target - P0;
            double a = u.X * diff.X + u.Y * diff.Y;      // >0：目标在车尾前方
            double d = n.X * diff.X + n.Y * diff.Y;      // >0：目标在车身左侧

            if (Math.Abs(d) < 1e-9)
            {
                // 正后方：直线倒退。目标在车尾前方（a>0）时倒退够不到，拒绝。
                if (a >= -1e-9) return false;
                straightLen = -a;
                radius = double.PositiveInfinity;
                side = 1;
                return straightLen > 1e-9;
            }

            int s = d > 0 ? 1 : -1;
            double D2 = a * a + d * d;
            double R2 = D2 / (2.0 * Math.Abs(d));        // 过 P0 切 u 且过 T 的圆半径（唯一）
            double R2min = ReverseMinControlRadius(p);
            if (R2 < R2min - 1e-9) return false;         // 比转向极限还急 → 调用方走最小半径回退

            Point2d C = P0 + (s * R2) * n;
            Vector2d v1 = P0 - C, v2 = target - C;
            double raw = Math.Atan2(v1.X * v2.Y - v1.Y * v2.X, v1.X * v2.X + v1.Y * v2.Y);
            // 倒退绕 C 的旋转方向 = −s（车尾甩向 s 侧时车头航向向 −s 侧偏：物理即"倒左甩右"）
            double signed = Norm2Pi(-s * raw);
            // v4.9.21：倒退单段甩尾上限 180° → 90°。>90° 的倒退单弧在车身侧后方陡变区
            // 极易被无意点出来（用户多次报「确认后多绕一圈」），且倒车超过 90° 本来
            // 就该分两段走（与 L/R 一键 90° 的最大刻意甩尾一致）。
            if (signed > Math.PI / 2.0) return false;    // 超过 90°：不可行，回退

            side = s;
            turnAngleDeg = signed * 180.0 / Math.PI;
            radius = R2;
            return turnAngleDeg > 1e-6;
        }

        /// <summary>
        /// v4.9.9：倒退的最小半径回退解 —— 所需半径 &lt; 极限时，沿最小半径圆把车尾尽量甩向光标。
        /// 与前进的 TurnTowardTargetAtMinRadius 同构，边界连续；转角上限 180°。
        /// </summary>
        public static bool ReverseTurnTowardAtMinRadius(VehicleParams p, Frame startFrame, Point2d target,
                                                        out int side, out double turnAngleDeg, out double radius)
        {
            side = 0; turnAngleDeg = 0.0; radius = 0.0;

            Point2d P0; Vector2d u;
            if (p.Articulated) { P0 = startFrame.O2; u = startFrame.U2; }
            else { P0 = startFrame.O1; u = startFrame.U1; }
            u = u.GetNormal();
            Vector2d n = new Vector2d(-u.Y, u.X);
            Vector2d diff = target - P0;
            double a = u.X * diff.X + u.Y * diff.Y;      // >0：目标在控制点前方
            double d = n.X * diff.X + n.Y * diff.Y;
            if (Math.Abs(d) < 1e-9) return false;        // 正前/正后方由 TrySolveReverseArc 处理

            // v4.9.16 修复：目标在控制点「前方」（a>0）时纯倒退弧必须绕 >180° 才够得着
            // （TrySolveReverseArc 的 signed>π 规则已正确拒绝这类目标），
            // 但本回退函数不看 a —— deg 被钳到 180°，画出一个几十米半径的半圆怪物。
            // 更糟的是 a=0 是间断线：光标悬停在线后（预览小转角正常），
            // 左键落点越过线 → 确认段突变 180° 半圆（「预览正确、确认多转一圈」）。
            // 与精确解保持同一边界：a>0 判不可行，由调用方把该段标为无效（不画）。
            if (a > 1e-9) return false;

            double R2min = ReverseMinControlRadius(p);
            if (R2min < 0.3) R2min = 0.3;                // 挂车可 pivot 时防退化

            int s = d > 0 ? 1 : -1;
            Point2d C = P0 + (s * R2min) * n;
            Vector2d v1 = P0 - C, v2 = target - C;
            if (v1.Length < 1e-9 || v2.Length < 1e-9) return false;
            double raw = Math.Atan2(v1.X * v2.Y - v1.Y * v2.X, v1.X * v2.X + v1.Y * v2.Y);
            double signed = Norm2Pi(-s * raw);
            double deg = signed * 180.0 / Math.PI;
            // v4.9.21：回退甩尾同样封顶 90°（原钳 180°）。目标在最小半径圆远侧时
            // deg 可达 ~180°，画出来就是「多绕一圈」的大弧，且车尾根本到不了光标点
            // （回退只负责"尽量甩向"）——判不可行，提示用户分两段倒。
            if (deg > 90.0) return false;

            side = s; turnAngleDeg = deg; radius = R2min;
            return deg > 1e-6;
        }

        /// <summary>
        /// v4.9.9：倒退模拟 —— 「挂车牵引」模型 + 司机修正。
        ///
        /// 为什么重写（v4.9 ~ v4.9.8 的倒退一直被报"卡死一个角度不动"）：
        ///   旧实现把前进运动学直接反向积分（dir=−1），铰接角 θ 对行驶弧长是
        ///   θ(s)=θ0·e^(+s/L2) —— <b>指数发散</b>（真实 jackknife），几步就撞上
        ///   MaxArticulationAngleDeg 限位然后被钳死，画面上就是"车头车厢锁死一个角度"。
        ///   真实司机倒车不会这样：他会反打方向盘把铰接角控制在稳态附近。
        ///
        /// 新模型分两层：
        ///   ① 挂车层（精确运动学）：挂车后轴 O2 沿「过目标点的圆弧」走，
        ///      挂车航向随圆弧精确旋转（倒退时 φ2 的变化方向 = −side）。
        ///      这层无积分误差，车尾精确到达光标。
        ///   ② 牵引车层（司机修正模型）：稳态铰接角 θ_eq 由同心圆几何解析给出
        ///      θ_eq = −atan2(side·(R1·L2 − ak·R2), R1·R2 + ak·L2)，R1 = sqrt(R2²+L2²−ak²)
        ///      （已与前进恒转角模拟的 ODE 稳态条件 φ1dot·(L2−ak·cosθ) = −side·sinθ 核对一致）；
        ///      实际铰接角从当前值指数收敛过去：θ(s) = θ_eq + (θ0−θ_eq)·e^(−s/λ)，
        ///      λ = max(L2, 3m) —— 等价于司机边倒边修正，θ 有界、永不 jackknife。
        ///      牵引车姿态由鞍座几何直接构造：φ1 = φ2 − θ，O1 = K − ak·u1。
        ///   直行倒退是 R2→∞ 的特例：θ_eq = 0，挂车直线后移，铰接角指数回正。
        ///
        /// 副作用（可接受）：收敛过渡段内牵引车后轴有厘米级侧滑（司机修正的代价），
        /// 视觉上就是"老司机倒车"的平滑修正动作。
        /// </summary>
        /// <param name="side">车尾甩向 +1=左 / −1=右（来自 TrySolveReverseArc）</param>
        /// <param name="radius">车尾控制点半径 R2；+∞ 或 ≤0 表示纯直行倒退</param>
        public static List<Frame> SimulateReverseFromState(VehicleParams p, Frame startFrame,
                                                           int side, double turnAngleDeg,
                                                           double straightLen, double radius,
                                                           double degPerStep,
                                                           double toothBudget = StraightToothBudgetM)
        {
            if (!p.Articulated)
                return SimulateRigidReverseFromState(p, startFrame, side, turnAngleDeg, straightLen, radius, degPerStep);

            var frames = new List<Frame>();
            double L = p.TractorWheelbase, ak = p.TractorRearToKingpin, L2 = p.TrailerKingpinToRearAxle;
            if (L <= 1e-9 || L2 <= 1e-9) return frames;
            double maxArt = p.MaxArticulationAngleDeg * Math.PI / 180.0;

            int s = side >= 0 ? 1 : -1;
            Point2d O2 = startFrame.O2;
            double phi2 = Math.Atan2(startFrame.U2.Y, startFrame.U2.X);
            double phi1 = Math.Atan2(startFrame.U1.Y, startFrame.U1.X);
            double theta = NormalizeAngle(phi2 - phi1);         // 当前铰接角
            double lambda = Math.Max(L2, 3.0);                  // 司机修正收敛长度

            bool hasArc = turnAngleDeg > 1e-9 && !double.IsInfinity(radius) && !double.IsNaN(radius) && radius > 1e-9;
            double arcLenDone = 0.0;

            if (hasArc)
            {
                double R2 = radius;
                Vector2d u20 = new Vector2d(Math.Cos(phi2), Math.Sin(phi2));
                Vector2d n20 = new Vector2d(-u20.Y, u20.X);
                Point2d C = O2 + (s * R2) * n20;
                double R1 = Math.Sqrt(Math.Max(1e-6, R2 * R2 + L2 * L2 - ak * ak));
                // 稳态铰接角（同心圆几何解析解，推导见函数头注释）
                double thetaEq = -Math.Atan2(s * (R1 * L2 - ak * R2), R1 * R2 + ak * L2);
                if (thetaEq > maxArt) thetaEq = maxArt;
                else if (thetaEq < -maxArt) thetaEq = -maxArt;

                double totalTurn = turnAngleDeg * Math.PI / 180.0;
                double dStep = Math.Min((degPerStep > 0 ? degPerStep : 1.0) * Math.PI / 180.0, 0.5 / R2);
                int n = Math.Max(1, (int)Math.Ceiling(totalTurn / dStep));
                double dTh = totalTurn / n;
                Vector2d rel0 = O2 - C;

                for (int i = 0; i <= n; i++)
                {
                    double Th = i * dTh;                        // 无符号圆心角
                    double ang = -s * Th;                       // 倒退旋转方向 = −side
                    double phi2i = phi2 + ang;
                    Vector2d u2i = new Vector2d(Math.Cos(phi2i), Math.Sin(phi2i));
                    double ca = Math.Cos(ang), sa = Math.Sin(ang);
                    Point2d O2i = C + new Vector2d(rel0.X * ca - rel0.Y * sa,
                                                   rel0.X * sa + rel0.Y * ca);
                    double arcLen = R2 * Th;
                    double thetaI = thetaEq + (theta - thetaEq) * Math.Exp(-arcLen / lambda);
                    if (thetaI > maxArt) thetaI = maxArt;
                    else if (thetaI < -maxArt) thetaI = -maxArt;
                    double phi1i = phi2i - thetaI;
                    Vector2d u1i = new Vector2d(Math.Cos(phi1i), Math.Sin(phi1i));
                    Point2d Ki = O2i + L2 * u2i;
                    Point2d O1i = Ki - ak * u1i;
                    frames.Add(new Frame { O1 = O1i, U1 = u1i, K = Ki, U2 = u2i, O2 = O2i });
                }

                var lastF = frames[frames.Count - 1];
                O2 = lastF.O2;
                phi2 = Math.Atan2(lastF.U2.Y, lastF.U2.X);
                theta = NormalizeAngle(phi2 - Math.Atan2(lastF.U1.Y, lastF.U1.X));
                arcLenDone = R2 * totalTurn;
            }
            else
            {
                frames.Add(new Frame { O1 = startFrame.O1, U1 = startFrame.U1, K = startFrame.K,
                                       U2 = startFrame.U2, O2 = startFrame.O2 });
            }

            // 直行倒退段：挂车沿 −u2 直线后移，铰接角指数回正（θ_eq = 0）
            // v4.9.10：步长按 StraightStepLimit 自适应（带铰接角倒退直行时挂车同样横向
            // 摆动，固定 0.5m 步长会在包络外边界留 ds·|sinθ| 级台阶齿）。
            if (straightLen > 1e-9)
            {
                Vector2d u2 = new Vector2d(Math.Cos(phi2), Math.Sin(phi2));
                double done = 0.0;
                while (done < straightLen - 1e-12)
                {
                    double step = Math.Min(StraightStepLimit(theta, 0.5, toothBudget), straightLen - done);
                    arcLenDone += step;
                    O2 = O2 - step * u2;
                    double thetaI = theta * Math.Exp(-step / lambda);
                    if (thetaI > maxArt) thetaI = maxArt;
                    else if (thetaI < -maxArt) thetaI = -maxArt;
                    theta = thetaI;
                    double phi1i = phi2 - thetaI;
                    Vector2d u1i = new Vector2d(Math.Cos(phi1i), Math.Sin(phi1i));
                    Point2d Ki = O2 + L2 * u2;
                    Point2d O1i = Ki - ak * u1i;
                    frames.Add(new Frame { O1 = O1i, U1 = u1i, K = Ki, U2 = u2, O2 = O2 });
                    done += step;
                }
            }
            return frames;
        }

        /// <summary>v4.9.9：刚性单车倒退 —— 后轴沿圆弧（与挂车层同一套几何，无铰接角）。</summary>
        private static List<Frame> SimulateRigidReverseFromState(VehicleParams p, Frame startFrame,
                                                                 int side, double turnAngleDeg,
                                                                 double straightLen, double radius,
                                                                 double degPerStep)
        {
            var frames = new List<Frame>();
            int s = side >= 0 ? 1 : -1;
            Point2d O1 = startFrame.O1;
            double phi1 = Math.Atan2(startFrame.U1.Y, startFrame.U1.X);

            bool hasArc = turnAngleDeg > 1e-9 && !double.IsInfinity(radius) && !double.IsNaN(radius) && radius > 1e-9;
            if (hasArc)
            {
                Vector2d u10 = new Vector2d(Math.Cos(phi1), Math.Sin(phi1));
                Vector2d n10 = new Vector2d(-u10.Y, u10.X);
                Point2d C = O1 + (s * radius) * n10;
                double totalTurn = turnAngleDeg * Math.PI / 180.0;
                double dStep = Math.Min((degPerStep > 0 ? degPerStep : 1.0) * Math.PI / 180.0, 0.5 / radius);
                int n = Math.Max(1, (int)Math.Ceiling(totalTurn / dStep));
                double dTh = totalTurn / n;
                Vector2d rel0 = O1 - C;
                for (int i = 0; i <= n; i++)
                {
                    double ang = -s * i * dTh;
                    double phi1i = phi1 + ang;
                    Vector2d u1i = new Vector2d(Math.Cos(phi1i), Math.Sin(phi1i));
                    double ca = Math.Cos(ang), sa = Math.Sin(ang);
                    Point2d O1i = C + new Vector2d(rel0.X * ca - rel0.Y * sa,
                                                   rel0.X * sa + rel0.Y * ca);
                    AddRigidFrame(frames, O1i, u1i, p);
                }
                var lastF = frames[frames.Count - 1];
                O1 = lastF.O1;
                phi1 = Math.Atan2(lastF.U1.Y, lastF.U1.X);
            }
            else
            {
                AddRigidFrame(frames, O1, new Vector2d(Math.Cos(phi1), Math.Sin(phi1)), p);
            }

            if (straightLen > 1e-9)
            {
                int nS = Math.Max(1, (int)Math.Ceiling(straightLen / 0.5));
                double step = straightLen / nS;
                Vector2d u1 = new Vector2d(Math.Cos(phi1), Math.Sin(phi1));
                for (int i = 1; i <= nS; i++)
                {
                    O1 = O1 - step * u1;                        // 后轴沿 −u1 直线后移
                    AddRigidFrame(frames, O1, u1, p);
                }
            }
            return frames;
        }

        #endregion

        /// <summary>从前轴中心开始模拟一段「转弯+直行」（初始放置用）</summary>
        public static List<Frame> SimulateFromFrontAxle(VehicleParams p, Point2d startFront, Vector2d heading, int steerSign,
                                                         double turnAngleDeg, double straightLen, double degPerStep,
                                                         double? turnRadiusOverride = null,
                                                         double toothBudget = StraightToothBudgetM)
        {
            if (!p.Articulated)
                return SimulateRigidFromFrontAxle(p, startFront, heading, steerSign, turnAngleDeg, straightLen, degPerStep, turnRadiusOverride);

            var frames = new List<Frame>();
            double L = p.TractorWheelbase;
            double ak = p.TractorRearToKingpin;
            double L2 = p.TrailerKingpinToRearAxle;
            double maxArt = p.MaxArticulationAngleDeg;
            if (L <= 1e-9 || L2 <= 1e-9) return frames;

            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u0 = heading.GetNormal();
            double phi0 = Math.Atan2(u0.Y, u0.X);
            Vector2d n0 = new Vector2d(-u0.Y, u0.X);
            Point2d O0 = startFront - L * u0;
            Point2d C = O0 + (steerSign * R) * n0;

            double tanDelta = L / R;
            double phi1dot = steerSign * tanDelta / L;
            double dTheta = degPerStep * Math.PI / 180.0;
            // v4.5.2 修复：自适应反解会把 R 取得很大（目标几乎正前方时 R→+∞），
            // ds=R·dTheta 也→∞，phi2 Euler 积分「dphi2/ds = sin(phi2-phi1)/L2 + phi1dot·cos/...」
            // 单步幅度可能远超挂车收敛常数 L2=11m，step≫22m 时显式 Euler 失稳。
            // 实测：delta=-2.4°、R=+∞、straightLen=60m 时 nS=1、单步 deltaUpdate=+13°，
            //   与用户报告「直线行驶时车尾乱飘」一致。
            // 修法：把 ds 钳到 ≤0.5m（远小于 2·L2=22m 稳定界，且 Euler 误差 ≤0.7°）。
            double ds = Math.Min(R * dTheta, 0.5);

            double phi2 = phi0;
            Point2d F = startFront;
            Vector2d u1 = u0;

            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            // v4.9.13：转弯分支改弧长自适应步进（原按固定角 dTheta 一帧，帧距=R·dTheta
            // 不随铰接角加密）。缓弯（大 R）+ 段首残留铰接角时，挂车扇形扫掠的
            // 离散并集产生 ds·|sinθ| 级台阶齿（与 v4.9.10 直行段同机理，
            // 实测 θ0=30°、R=80m 时齿深 ~17cm）。按 StraightStepLimit 钳弧长步
            // ≤ 0.02/|sinθ| 后齿深 ≤2cm，被 RDP 平滑且出图不可见；
            // θ→0 退回 ds（≤0.5m）不增帧。Euler 稳定界不变（step ≤ ds ≤ 0.5m ≪ 2·L2）。
            // R ≥ 1e9（自适应反解 R→+∞）时圆弧退化为直线，跳过转弯帧由直行段覆盖。
            if (totalTurn > 1e-12 && R < 1e9)
            {
                double done = 0.0; // 已转过的前轮弧角（弧度，无符号）
                while (done < totalTurn - 1e-12)
                {
                    PoseOnArcFront(startFront, C, phi0, steerSign * done, L, out F, out u1);
                    Point2d O1 = F - L * u1;
                    AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                    double th = NormalizeAngle(phi2 - Math.Atan2(u1.Y, u1.X));
                    double step = Math.Min(StraightStepLimit(th, ds, toothBudget), R * (totalTurn - done));
                    phi2 += TrailerYawRate(u1, phi2, ak, phi1dot, L2) * step;
                    done += step / R;
                }
                // 终点帧（精确落在 totalTurn）
                PoseOnArcFront(startFront, C, phi0, steerSign * totalTurn, L, out F, out u1);
                Point2d O1f = F - L * u1;
                AddFrame(frames, O1f, u1, phi2, ak, L2, maxArt);
            }

            if (straightLen > 1e-9)
            {
                // v4.9.10：自适应步长（StraightStepLimit），挂车带铰接角直行时加密，
                // 消除包络外边界 ds·|sinθ| 级台阶齿（详见常量注释）。
                double done = 0.0;
                while (done < straightLen - 1e-12)
                {
                    double th = NormalizeAngle(phi2 - Math.Atan2(u1.Y, u1.X));
                    double step = Math.Min(StraightStepLimit(th, ds, toothBudget), straightLen - done);
                    phi2 += TrailerYawRate(u1, phi2, ak, 0.0, L2) * step;
                    F = F + step * u1;
                    Point2d O1 = F - L * u1;
                    AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                    done += step;
                }
            }

            if (frames.Count == 0) AddFrame(frames, O0, u0, phi0, ak, L2, maxArt);
            return frames;
        }

        /// <summary>从前轴中心任意状态开始模拟一段「转弯+直行」，支持前进(dir=+1)/后退(dir=-1)</summary>
        public static List<Frame> SimulateFromStateFront(VehicleParams p, Point2d startFront, Vector2d startU1, double startPhi2,
                                                          int steerSign, double turnAngleDeg, double straightLen, int dir, double degPerStep,
                                                          double? turnRadiusOverride = null,
                                                          double toothBudget = StraightToothBudgetM)
        {
            if (!p.Articulated)
                return SimulateRigidFromStateFront(p, startFront, startU1, steerSign, turnAngleDeg, straightLen, dir, degPerStep, turnRadiusOverride);

            var frames = new List<Frame>();
            double L = p.TractorWheelbase, ak = p.TractorRearToKingpin, L2 = p.TrailerKingpinToRearAxle;
            double maxArt = p.MaxArticulationAngleDeg;
            if (L <= 1e-9 || L2 <= 1e-9) return frames;
            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u0 = startU1.GetNormal();
            double phi0 = Math.Atan2(u0.Y, u0.X);
            Vector2d n0 = new Vector2d(-u0.Y, u0.X);
            Point2d O0 = startFront - L * u0;
            Point2d C = O0 + (steerSign * R) * n0;

            double tanDelta = L / R;
            double dTheta = degPerStep * Math.PI / 180.0;
            // v4.5.2 修复：同上 SimulateFromFrontAxle：自适应 R→+∞ 时 ds=R·dTheta 也→+∞，
            // phi2 单步 Euler 严重过冲（delta=-2.4°、straightLen=60m、nS=1 时单步 +13°）。
            // 把 ds 钳到 ≤0.5m，既保证 Euler 稳定（step≪2·L2=22m）又保证精度（误差 ≤0.7°）。
            double ds = Math.Min(R * dTheta, 0.5);

            double phi2 = startPhi2;
            Point2d F = startFront;
            Vector2d u1 = u0;

            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            // v4.9.13：转弯分支改弧长自适应步进（同 SimulateFromFrontAxle 的说明）——
            // 缓弯 + 残留铰接角时挂车扇形扫掠产生 ds·|sinθ| 级台阶齿，
            // 按 StraightStepLimit 钳弧长步后齿深 ≤2cm。
            if (totalTurn > 1e-12 && R < 1e9)
            {
                double phi1dotSigned = dir * steerSign * tanDelta / L;
                double done = 0.0;
                while (done < totalTurn - 1e-12)
                {
                    PoseOnArcFront(startFront, C, phi0, dir * steerSign * done, L, out F, out u1);
                    Point2d O1 = F - L * u1;
                    AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                    double th = NormalizeAngle(phi2 - Math.Atan2(u1.Y, u1.X));
                    double step = Math.Min(StraightStepLimit(th, ds, toothBudget), R * (totalTurn - done));
                    phi2 += TrailerYawRate(u1, phi2, ak, phi1dotSigned, L2) * (dir * step);
                    done += step / R;
                }
                PoseOnArcFront(startFront, C, phi0, dir * steerSign * totalTurn, L, out F, out u1);
                Point2d O1f = F - L * u1;
                AddFrame(frames, O1f, u1, phi2, ak, L2, maxArt);
            }

            if (straightLen > 1e-9)
            {
                // v4.9.10：自适应步长（StraightStepLimit），挂车带铰接角直行时加密，
                // 消除包络外边界 ds·|sinθ| 级台阶齿（详见常量注释）。
                double done = 0.0;
                while (done < straightLen - 1e-12)
                {
                    double th = NormalizeAngle(phi2 - Math.Atan2(u1.Y, u1.X));
                    double step = Math.Min(StraightStepLimit(th, ds, toothBudget), straightLen - done);
                    double dsSigned = dir * step;
                    phi2 += TrailerYawRate(u1, phi2, ak, 0.0, L2) * dsSigned;
                    F = F + dir * step * u1;
                    Point2d O1 = F - L * u1;
                    AddFrame(frames, O1, u1, phi2, ak, L2, maxArt);
                    done += step;
                }
            }
            if (frames.Count == 0) AddFrame(frames, O0, u0, phi2, ak, L2, maxArt);
            return frames;
        }

        /// <summary>刚性单车：从前轴中心开始模拟</summary>
        private static List<Frame> SimulateRigidFromFrontAxle(VehicleParams p, Point2d startFront, Vector2d heading, int steerSign,
                                                               double turnAngleDeg, double straightLen, double degPerStep,
                                                               double? turnRadiusOverride = null)
        {
            var frames = new List<Frame>();
            double L = p.TractorWheelbase;
            if (L <= 1e-9) return frames;
            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u0 = heading.GetNormal();
            double phi0 = Math.Atan2(u0.Y, u0.X);
            Vector2d n0 = new Vector2d(-u0.Y, u0.X);
            Point2d O0 = startFront - L * u0;
            Point2d C = O0 + (steerSign * R) * n0;

            double dTheta = degPerStep * Math.PI / 180.0;
            // v4.9.18 修复：刚性变体漏了铰接变体 v4.5.2 的 ds 钳制 ——
            // 自适应反解在目标近正前方时 R→+∞（或几十万米），ds=R·dTheta→∞，
            // 旧代码 nS=Ceiling(straightLen/∞)=1 → 20m 直行只出 2 帧（首+尾），
            // 两个互不重叠的车体矩形被包络引擎串成「两端鼓起、中间细缝」的环，
            // 远看就是「直行时黄色包络消失」（刚性车高发/必发，铰接变体早有钳制不受影响）。
            // 同时转弯循环改弧长自适应步进（同铰接 v4.9.13），R≥1e9 时跳过转弯帧
            // （C=O0+steer·∞·n0=±∞，PoseOnArcFront 会算 NaN 毒化全部帧）。
            double ds = Math.Min(R * dTheta, 0.5);
            Point2d F = startFront;
            Vector2d u1 = u0;

            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            if (totalTurn > 1e-12 && R < 1e9)
            {
                double done = 0.0;
                while (done < totalTurn - 1e-12)
                {
                    PoseOnArcFront(startFront, C, phi0, steerSign * done, L, out F, out u1);
                    Point2d O1 = F - L * u1;
                    AddRigidFrame(frames, O1, u1, p);
                    double step = Math.Min(ds, R * (totalTurn - done));
                    done += step / R;
                }
                PoseOnArcFront(startFront, C, phi0, steerSign * totalTurn, L, out F, out u1);
                Point2d O1f = F - L * u1;
                AddRigidFrame(frames, O1f, u1, p);
            }
            if (straightLen > 1e-9)
            {
                double done = 0.0;
                while (done < straightLen - 1e-12)
                {
                    double step = Math.Min(ds, straightLen - done);
                    F = F + step * u1;
                    Point2d O1 = F - L * u1;
                    AddRigidFrame(frames, O1, u1, p);
                    done += step;
                }
            }
            if (frames.Count == 0) AddRigidFrame(frames, O0, u0, p);
            return frames;
        }

        /// <summary>刚性单车：从前轴中心任意状态开始模拟，支持前进/后退</summary>
        private static List<Frame> SimulateRigidFromStateFront(VehicleParams p, Point2d startFront, Vector2d startU1,
                                                                int steerSign, double turnAngleDeg, double straightLen, int dir, double degPerStep,
                                                                double? turnRadiusOverride = null)
        {
            var frames = new List<Frame>();
            double L = p.TractorWheelbase;
            if (L <= 1e-9) return frames;
            double R = TurnRadius(p, turnRadiusOverride);
            if (degPerStep <= 0) degPerStep = 1.0;

            Vector2d u0 = startU1.GetNormal();
            double phi0 = Math.Atan2(u0.Y, u0.X);
            Vector2d n0 = new Vector2d(-u0.Y, u0.X);
            Point2d O0 = startFront - L * u0;
            Point2d C = O0 + (steerSign * R) * n0;
            double dTheta = degPerStep * Math.PI / 180.0;
            // v4.9.18 修复：同 SimulateRigidFromFrontAxle —— ds 钳 ≤0.5m（R→∞ 时
            // 旧代码 nS=1 只出 2 帧，包络塌缩成细缝）；转弯循环改弧长自适应步进；
            // R≥1e9 跳过转弯帧避免 C=±∞ 产生 NaN。
            double ds = Math.Min(R * dTheta, 0.5);
            Point2d F = startFront;
            Vector2d u1 = u0;

            double totalTurn = Math.Max(0.0, turnAngleDeg) * Math.PI / 180.0;
            if (totalTurn > 1e-12 && R < 1e9)
            {
                double done = 0.0;
                while (done < totalTurn - 1e-12)
                {
                    PoseOnArcFront(startFront, C, phi0, dir * steerSign * done, L, out F, out u1);
                    Point2d O1 = F - L * u1;
                    AddRigidFrame(frames, O1, u1, p);
                    double step = Math.Min(ds, R * (totalTurn - done));
                    done += step / R;
                }
                PoseOnArcFront(startFront, C, phi0, dir * steerSign * totalTurn, L, out F, out u1);
                Point2d O1f = F - L * u1;
                AddRigidFrame(frames, O1f, u1, p);
            }
            if (straightLen > 1e-9)
            {
                double done = 0.0;
                while (done < straightLen - 1e-12)
                {
                    double step = Math.Min(ds, straightLen - done);
                    F = F + dir * step * u1;
                    Point2d O1 = F - L * u1;
                    AddRigidFrame(frames, O1, u1, p);
                    done += step;
                }
            }
            if (frames.Count == 0) AddRigidFrame(frames, O0, u0, p);
            return frames;
        }

        /// <summary>前轴中心绕 C 的精确旋转</summary>
        private static void PoseOnArcFront(Point2d startFront, Point2d C, double phi10, double signedAng, double L,
                                           out Point2d front, out Vector2d u1)
        {
            double phi1 = phi10 + signedAng;
            u1 = new Vector2d(Math.Cos(phi1), Math.Sin(phi1));
            Vector2d rel = startFront - C;
            double ca = Math.Cos(signedAng), sa = Math.Sin(signedAng);
            front = C + new Vector2d(rel.X * ca - rel.Y * sa, rel.X * sa + rel.Y * ca);
        }

        /// <summary>
        /// 一键 90° 转弯并拉直挂车（TRUCKTURN90 用）。
        /// 对铰接车：先以最小半径转 90°（车头回正），再继续直行最短距离，
        /// 使挂车航向收敛到与车头一致（铰接角 ≈ 0）。刚性车直接转 90°。
        /// </summary>
        /// <summary>
        /// TRUCKTURN90 结束时的挂车摆正容差（度）。写死，不做成可调参数（用户 v4.5 决定）。
        /// 它同时决定了拉直段的长度，见 SimulateTurn90AndStraighten 里的推导注释。
        ///
        /// v4.9.3：2.0° → 5.0°。原因：2° 容差需要走 ~30m 才能收敛，挂车末段「沿车头方向
        /// 拉一条长直线」在视觉上像"挂车不动只平移"，实测把 5° 误判为 bug 的用户不少。
        /// 5° 容差只需 ~19m，残角 5° 在总图上肉眼难辨（与车厢栏板厚度一个量级）。
        /// </summary>
        public const double StraightenToleranceDeg = 5.0;

        /// v4.9：新增 dir 参数（+1 前进 / −1 倒退）。
        ///
        /// ★ 倒退时<b>不做拉直段</b>，这是物理约束不是偷懒：
        /// 铰接角对行驶弧长是指数收敛 θ(s)=θ0·e^(−s/L2)，前进时稳定、倒退时该式变成
        /// e^(+s/L2) —— 铰接角<b>发散</b>，也就是真实世界的"折叠/jackknife"。
        /// 所以「倒着把挂车拉直」在运动学上根本不存在，倒退 90° 只能停在挂车折叠的姿态上
        /// （折叠角由 AddFrame 里的 MaxArticulationAngleDeg 限位，与实车的机械限位对应）。
        /// </summary>
        public static List<Frame> SimulateTurn90AndStraighten(VehicleParams p, Point2d startFront, Vector2d heading,
                                                              int steerSign, double degPerStep, int dir = 1)
        {
            // v4.9.9：倒退 90° 改走「挂车牵引」模型 —— 旧分支直接反向积分前进运动学，
            // 铰接角指数发散撞限位（jackknife），画面上车头车厢锁死一个角度。
            // steerSign 语义变为「车尾甩向」（+1=车尾向左甩 / −1=向右甩）。
            if (dir < 0)
            {
                Vector2d u1r = heading.GetNormal();
                double Lr = p.TractorWheelbase;
                Point2d O1r = startFront - Lr * u1r;
                Frame f0;
                if (p.Articulated)
                {
                    Point2d Kr = O1r + p.TractorRearToKingpin * u1r;
                    Point2d O2r = Kr - p.TrailerKingpinToRearAxle * u1r;   // 与前进分支一致：起始共线
                    f0 = new Frame { O1 = O1r, U1 = u1r, K = Kr, U2 = u1r, O2 = O2r };
                }
                else
                {
                    f0 = new Frame { O1 = O1r, U1 = u1r, K = O1r + Lr * u1r, U2 = u1r, O2 = O1r + Lr * u1r };
                }
                // 半径选取：稳态铰接角 ≤ 60% 机械限位（R2→∞ 时 tanθ_eq ≈ L2/R2），
                // 同时不小于转向极限允许的最小车尾半径。半挂典型值：L2/tan(0.6·45°) ≈ 21m。
                double R2 = ReverseMinControlRadius(p);
                if (p.Articulated)
                {
                    double thetaStar = p.MaxArticulationAngleDeg * 0.6 * Math.PI / 180.0;
                    double R2gentle = p.TrailerKingpinToRearAxle / Math.Tan(thetaStar);
                    if (R2gentle > R2) R2 = R2gentle;
                }
                if (R2 < 0.3) R2 = 0.3;
                return SimulateReverseFromState(p, f0, steerSign, 90.0, 0.0, R2, degPerStep);
            }

            var frames = SimulateFromFrontAxle(p, startFront, heading, steerSign, 90.0, 0.0, degPerStep);
            if (!p.Articulated || frames == null || frames.Count == 0) return frames;

            var last = frames[frames.Count - 1];
            double phi1 = Math.Atan2(last.U1.Y, last.U1.X);
            double phi2 = Math.Atan2(last.U2.Y, last.U2.X);
            double theta0 = NormalizeAngle(phi2 - phi1);
            // 挂车铰接角是指数收敛的，精确到达 0 需要无穷远，必须取一个容差。
            // 容差的物理代价是挂车尾部相对「完全摆正」的横向偏差 ≈ (L2+后悬)·sin(eps)；
            // 直行距离由精确解反解：s = L2·ln(tan(theta0/2)/tan(eps/2))，
            // 长度常数就是挂车轴距 L2，所以 eps 每缩小一半就要多走 L2·ln2 ≈ 7.6m。
            //   eps=3.0° → 26m，车尾偏 0.64m（明显还能看出歪）
            //   eps=2.0° → 31m，车尾偏 0.43m（用户 v4.5 选定：走廊长度与观感的平衡点）
            //   eps=1.0° → 38m，车尾偏 0.21m
            //   eps=0.5° → 46m，车尾偏 0.11m（v4.4 的行为，用户反馈"走了很长的距离"）
            // 12.3m = 挂车轴距 11m + 后悬 1.3m。
            const double epsDeg = StraightenToleranceDeg;
            double eps = epsDeg * Math.PI / 180.0;
            if (Math.Abs(theta0) <= eps) return frames;

            double L2 = p.TrailerKingpinToRearAxle;
            // 直行拉直挂车：精确解 theta(s)=2*atan(tan(theta0/2)*exp(-s/L2))，
            // 其中 theta=phi2-phi1。令 theta(s)=eps，反解 s。
            double straightLen = 0.0;
            if (Math.Abs(theta0) > eps)
            {
                double t0 = Math.Tan(Math.Abs(theta0) / 2.0);
                double te = Math.Tan(eps / 2.0);
                if (t0 > te && te > 1e-12)
                    straightLen = L2 * Math.Log(t0 / te);
            }
            // 纯保护性上限：正常参数下精确解约 31m，远够不着这里。
            // 只在用户把挂车轴距填得极长、或前轮转角大到 theta0 接近 90° 时才可能触发，
            // 避免算出一个几百米的走廊（那时 tan(theta0/2) 会爆掉）。
            const double MaxStraightM = 60.0;
            straightLen = Math.Min(straightLen, MaxStraightM);
            // 保证至少走一小段，避免数值 residual
            straightLen = Math.Max(straightLen, L2 * 0.05);

            Point2d lastFront = FrontAxle(last, p);
            double startPhi2 = Math.Atan2(last.U2.Y, last.U2.X);
            var straightFrames = SimulateFromStateFront(p, lastFront, last.U1, startPhi2,
                                                        1, 0.0, straightLen, 1, degPerStep);

            // 合并，去掉 straightFrames 第一帧（与 last 重复）
            if (straightFrames != null && straightFrames.Count > 1)
            {
                for (int i = 1; i < straightFrames.Count; i++)
                    frames.Add(straightFrames[i]);
            }
            return frames;
        }

        /// <summary>
        /// 扫掠包络：所有车体 footprint 的并集外边界（AutoTURN 式 swept-area outline）。
        /// 对每一帧的每个矩形边做精确区间裁剪，去掉被其它车体矩形覆盖的部分；
        /// 剩下的子段按端点链式拼接成闭合环，取面积最大的环作为黄色包络。
        /// 这样交叉处自然取并集最外延，铰接车内侧 off-tracking 凹口也被保留，
        /// 不再是时间序角点折线，也不会被凸包错误填平。
        /// </summary>
        /// <summary>
        /// 上一次 <see cref="EnvelopeTracks"/> 是否被迫走了保底分支：并集边界成环失败时
        /// 退回「所有车体角点的凸包」，宁可胖一点也不让黄线消失。
        ///
        /// 正常情况恒为 false。校验工程（TruckTurn.Verify）断言它为 false：
        /// 一旦变 true 说明并集边界引擎又退化了，必须去修引擎，
        /// 不能拿「凸包也画得出来」当胜利 —— 凸包会填平铰接车 off-tracking 的真实凹口。
        /// 留着保底只是为了让用户永远看得到包络，不是让引擎可以躺平。
        /// </summary>
        public static bool EnvelopeFallbackUsed { get; private set; } = false;

        public static List<Point2d> EnvelopeTracks(List<Frame> frames, VehicleParams p)
        {
            EnvelopeFallbackUsed = false;
            if (frames == null || frames.Count < 2) return new List<Point2d>();
            return EnvelopeTracksFromFootprints(BuildEnvelopeFootprints(frames, p));
        }

        /// <summary>
        /// 用确认每段时缓存的车体凸多边形生成最终包络。缓存内容与 EnvelopeTracks
        /// 临时构造的 footprint 完全相同，因此只省去重复展开，不改变最终几何结果。
        /// </summary>
        internal static List<Point2d> EnvelopeTracksFromFootprints(
            List<Point2d[]> footprints)
        {
            EnvelopeFallbackUsed = false;
            if (footprints == null || footprints.Count == 0)
                return new List<Point2d>();
            return UnionEnvelopeFromConvexPolygons(footprints, out bool fallback);
        }

        /// <summary>
        /// 计算一组 CCW/CW 凸多边形的并集最外环。输入与输出均使用米制坐标。
        /// 供跨运车与货车共同复用同一套真实扫掠边界引擎，避免用凸包把弯道
        /// 内侧的真实曲边替换成长弦线。
        /// </summary>
        internal static List<Point2d> UnionEnvelopeFromConvexPolygons(
            List<Point2d[]> polygons, out bool fallback,
            bool? forceSpatialIndex = null)
        {
            EnvelopeFallbackUsed = false;
            fallback = false;
            if (polygons == null || polygons.Count == 0) return new List<Point2d>();

            var rects = new List<Point2d[]>(polygons.Count);
            for (int i = 0; i < polygons.Count; i++)
            {
                Point2d[] polygon = polygons[i];
                if (polygon == null || polygon.Length < 3) continue;
                rects.Add(MakeCCW(polygon));
            }
            if (rects.Count == 0) return new List<Point2d>();

            var segments = ExposedBoundarySegments(rects, forceSpatialIndex);
            var loops = ChainBoundaryLoops(segments);
            if (loops.Count == 0)
            {
                // 保底：并集边界拼不成环时，至少给出一条包住所有车体的闭合线。
                // 只在引擎真正失败时触发（EnvelopeFallbackUsed=true 会被校验判为不通过）。
                EnvelopeFallbackUsed = true;
                fallback = true;
                var all = new List<Point2d>(rects.Count * 4);
                foreach (var r in rects) all.AddRange(r);
                return GeometryUtil.ConvexHull(all);
            }
            loops.Sort((a, b) => PolygonArea(b).CompareTo(PolygonArea(a)));
            return loops[0];
        }

        #region 并集边界引擎

        /// <summary>把每帧车体转换为一个或多个 CCW 凸四边形（刚性 1 个，铰接 3 个）。</summary>
        ///
        /// 注意：不要试图在这里"补相邻帧的扫掠凸包"来消除外廓锯齿。
        /// 试过（v4.5 开发过程）：hull(pose_i, pose_{i+1}) 会完整包住 rect_i 与 rect_{i+1}，
        /// 于是中间所有原始矩形都不再贡献任何暴露边，边界全由凸包边组成，
        /// 首尾端盖处的顶点失去配对伙伴，ChainBoundaryLoops 直接成不了环（实测 loops=0）。
        /// 正确的消锯齿手段在 GeometryUtil.FitArcsAboutCenters —— 用"圆心角一致性"校验
        /// 把每帧一个的台阶整段吞进一条 bulge 圆弧（见那里的注释）。
        internal static List<Point2d[]> BuildEnvelopeFootprints(
            IList<Frame> frames, VehicleParams p)
        {
            if (frames == null) return new List<Point2d[]>();
            var rects = new List<Point2d[]>(frames.Count * (p.Articulated ? 3 : 1));
            foreach (var f in frames)
            {
                var c = Corners(f, p);
                // 牵引车（驾驶室）：前左、后左、后右、前右（CCW）
                rects.Add(MakeCCW(new[] { c[0], c[2], c[3], c[1] }));
                if (p.Articulated)
                {
                    // v4.4：驾驶室后方的窄车架也是真实车体，必须计入扫掠面积，
                    //       否则起步姿态下车架外露段会落在包络之外。
                    var chassis = ChassisRect(f, p);
                    if (chassis != null)
                        rects.Add(MakeCCW(new[] { chassis[0], chassis[3], chassis[2], chassis[1] }));
                }
                if (p.Articulated)
                {
                    // 挂车：前左、后左、后右、前右（CCW）
                    rects.Add(MakeCCW(new[] { c[4], c[6], c[7], c[5] }));
                }
            }
            return rects;
        }

        private static Point2d[] MakeCCW(Point2d[] poly)
        {
            double s = 0.0;
            int n = poly.Length;
            for (int i = 0; i < n; i++)
            {
                var a = poly[i];
                var b = poly[(i + 1) % n];
                s += a.X * b.Y - b.X * a.Y;
            }
            if (s >= 0) return poly;
            var rev = new Point2d[n];
            for (int i = 0; i < n; i++) rev[i] = poly[n - 1 - i];
            return rev;
        }

        private struct HalfPlane
        {
            public Vector2d Normal;  // 内法向
            public double Offset;    // dot(Normal, x) >= Offset
        }

        private static HalfPlane[] BuildHalfPlanes(Point2d[] rect)
        {
            var hs = new HalfPlane[rect.Length];
            int n = rect.Length;
            for (int i = 0; i < n; i++)
            {
                var a = rect[i];
                var b = rect[(i + 1) % n];
                var e = b - a;
                var m = new Vector2d(-e.Y, e.X);  // 左法向 = 内法向（CCW）
                hs[i] = new HalfPlane { Normal = m, Offset = m.X * a.X + m.Y * a.Y };
            }
            return hs;
        }

        /// <summary>
        /// Cyrus-Beck：求线段 AB 落在凸 rect 内的参数子区间 [t0,t1]。
        ///
        /// v4.5.1 关键修正：`A` 是沿外法向外推 delta 后的起点（见 ExposedBoundarySegments），
        /// 所以直接算出来的 t 是**外推后那条平行线**上的交点参数。本函数把它修正回
        /// **原始边所在直线**上的参数：两条线平行且方向同为 d，故 den 相同，
        /// num 只差 -delta·dot(n, nOut)，于是
        ///     t_原始 = t_外推 + delta·dot(n, nOut) / den
        ///
        /// 不修正的后果（用户 v4.5 报「连续转弯有概率包络消失」的真实根因）：
        /// 端点误差 = delta / |tan(theta)|，theta 是两相交边的夹角。
        /// v4.5 只估算了「90°转弯+25m 拉直」这条路径，最坏 theta≈4.2e-4 rad → 误差 2.4e-6 m，
        /// 于是把成环聚类容差定成 2e-5 就够。但**长直行**时挂车按
        /// theta(s)=2·atan(tan(theta0/2)·e^(-s/L2)) 指数收敛，theta 会一路小到 1e-5 rad，
        /// 误差随之涨到 1e-4 m，比容差大一个数量级 —— 走廊侧壁被切成几百段
        /// 端点互不相连的亚毫米碎片（实测「铰接 30°+直行60m」deg1=600、loops=0，
        /// 整条黄色包络消失）。修正后交点是精确的，碎片首尾严丝合缝。
        ///
        /// 副作用（正面的）：近平行时 den 很小、修正量很大，t 会被推到 [0,1] 之外，
        /// 于是「外推线判为覆盖、原始线其实根本没穿过这个面」的误判被自动纠正。
        /// </summary>
        private static bool? ClipInterval(Point2d A, Point2d B, Vector2d nOut, double delta,
                                          HalfPlane[] hs, out double t0, out double t1)
        {
            t0 = 0.0; t1 = 1.0;
            var d = B - A;
            foreach (var h in hs)
            {
                double num = h.Offset - (h.Normal.X * A.X + h.Normal.Y * A.Y);
                double den = h.Normal.X * d.X + h.Normal.Y * d.Y;
                if (Math.Abs(den) < 1e-12)
                {
                    if (num > 0.0) return false;  // 完全在外侧
                    continue;
                }
                double t = num / den;
                t += delta * (h.Normal.X * nOut.X + h.Normal.Y * nOut.Y) / den;
                if (den > 0.0)
                {
                    if (t > t0) t0 = t;
                }
                else
                {
                    if (t < t1) t1 = t;
                }
                if (t0 > t1) return false;
            }
            return true;
        }

        /// <summary>从 [0,1] 中减去若干覆盖区间，返回暴露区间。</summary>
        private static List<(double a, double b)> SubtractIntervals(List<(double a, double b)> covering)
        {
            var clipped = new List<(double a, double b)>();
            foreach (var iv in covering)
            {
                double a = Math.Max(0.0, iv.a);
                double b = Math.Min(1.0, iv.b);
                if (b > a) clipped.Add((a, b));
            }
            if (clipped.Count == 0) return new List<(double, double)> { (0.0, 1.0) };
            clipped.Sort((x, y) => x.a.CompareTo(y.a));
            var res = new List<(double a, double b)>();
            double cur = 0.0;
            foreach (var iv in clipped)
            {
                if (iv.a > cur)
                    res.Add((cur, iv.a));
                cur = Math.Max(cur, iv.b);
                if (cur >= 1.0) break;
            }
            if (cur < 1.0) res.Add((cur, 1.0));
            return res;
        }

        /// <summary>计算每条矩形边的暴露子段；这些子段拼成并集边界。</summary>
        private static List<(Point2d P, Point2d Q)> ExposedBoundarySegments(
            List<Point2d[]> rects, bool? forceSpatialIndex = null)
        {
            var segs = new List<(Point2d, Point2d)>();
            if (rects == null || rects.Count == 0) return segs;

            // =====================================================================
            // v4.5 关键修正：待判定的线段先沿该边的「外法向」外推 delta，再拿去裁剪。
            //
            // 旧实现直接判「线段是否落在另一个矩形的闭区域内」，这在共线等宽时会
            // 互相覆盖：拉直段里相邻帧的驾驶室是精确平移（U1 恒为 (0,1)），左右
            // 纵边严格共线、法向宽度完全相同 —— A 判定被 B 覆盖，B 判定被 A 覆盖，
            // 这条本该是并集边界的公共边一条都不暴露。实测铰接车 90°+25m 拉直：
            // 1503 条暴露段一条环都串不出来（loops=0），TRUCKTURN90 完全不画包络。
            // 纯圆弧（刚性 90°）不会触发，因为相邻帧始终有 0.25° 相对转动，
            // 边不共线 —— 这也解释了为什么只有铰接车躺枪。
            //
            // 外推后判的是「沿外法向走 delta 还在不在并集里」：
            //   在  → 该点外侧还有材料，是内部接缝（驾驶室与车架的对接缝），判覆盖；
            //   不在 → 外侧是空的，该点确实位于并集边界上，判暴露。
            // 这正是并集边界的定义，与「开区间还是闭区间」无关，共线互覆盖自然消失。
            //
            // delta = 1e-9 m（1 纳米）：比双精度在 ~50m 坐标下的噪声（~1e-12）高
            // 三个数量级，又远小于最细的真实特征（拉直段挂车每帧外凸约 2.4mm）。
            //
            // v4.5.1：外推把交点参数带到了「外推后那条平行线」上，端点误差
            // delta/|tan(theta)|（theta = 两相交边夹角）。v4.5 只估了 90°+25m 拉直这条路径，
            // 得最坏 theta≈4.2e-4 rad → 误差 2.4e-6 m，于是成环容差取 2e-5 就够。
            // 但**长直行**时挂车按 theta(s)=2·atan(tan(theta0/2)·e^(−s/L2)) 指数收敛，
            // theta 一路小到 1e-5 rad，误差涨到 1e-4 m，比容差大一个数量级 ——
            // 走廊侧壁被切成几百段端点互不相连的亚毫米碎片（实测「铰接 30°+直行60m」
            // deg1=600、loops=0，整条黄色包络消失）。
            // 兜不住：theta→0 时误差无界，无论把 delta 调小还是把容差调大都只是挪位置。
            // 正解是 ClipInterval 里把参数精确修正回原始边所在直线 —— 见那里的注释。
            // =====================================================================
            const double delta = 1e-9;

            var hss = new HalfPlane[rects.Count][];
            var aabbs = new double[rects.Count][];   // {minX,minY,maxX,maxY}
            var spans = new List<double>(rects.Count);
            for (int i = 0; i < rects.Count; i++)
            {
                hss[i] = BuildHalfPlanes(rects[i]);
                double minX = double.MaxValue, minY = double.MaxValue;
                double maxX = double.MinValue, maxY = double.MinValue;
                foreach (var q in rects[i])
                {
                    if (q.X < minX) minX = q.X;
                    if (q.Y < minY) minY = q.Y;
                    if (q.X > maxX) maxX = q.X;
                    if (q.Y > maxY) maxY = q.Y;
                }
                aabbs[i] = new[] { minX, minY, maxX, maxY };
                double span = Math.Max(maxX - minX, maxY - minY);
                if (span > 1e-9 && !double.IsNaN(span) && !double.IsInfinity(span))
                    spans.Add(span);
            }

            // 长路径尤其是铰接车会产生数千个 footprint。旧代码虽然先做 AABB
            // 判定，但仍对每条边扫描全部矩形，最终 X 出图为 O(N²)。均匀空间桶
            // 只返回与边包围盒处于相同网格的候选矩形；随后仍执行原来的精确
            // AABB + Cyrus-Beck 裁剪，因此这是纯索引加速，不改变并集边界。
            spans.Sort();
            double typicalSpan = spans.Count == 0 ? 4.0 : spans[spans.Count / 2];
            double cellSize = Math.Max(1.0, typicalSpan * 0.5);
            // 中短路径直接顺序扫描更快；空间桶只在最终长路径进入二次复杂度区间时启用。
            bool useSpatialIndex = forceSpatialIndex ?? (rects.Count >= 3000);
            var spatial = new Dictionary<(long x, long y), List<int>>();
            var broadRects = new List<int>();
            if (useSpatialIndex)
            for (int i = 0; i < aabbs.Length; i++)
            {
                double[] bb = aabbs[i];
                long x0 = (long)Math.Floor(bb[0] / cellSize);
                long x1 = (long)Math.Floor(bb[2] / cellSize);
                long y0 = (long)Math.Floor(bb[1] / cellSize);
                long y1 = (long)Math.Floor(bb[3] / cellSize);
                long cells = (x1 - x0 + 1) * (y1 - y0 + 1);
                if (cells > 256)
                {
                    broadRects.Add(i);
                    continue;
                }
                for (long gx = x0; gx <= x1; gx++)
                for (long gy = y0; gy <= y1; gy++)
                {
                    var key = (gx, gy);
                    if (!spatial.TryGetValue(key, out List<int> bucket))
                    {
                        bucket = new List<int>();
                        spatial.Add(key, bucket);
                    }
                    bucket.Add(i);
                }
            }

            var candidateMarks = new int[rects.Count];
            int candidateStamp = 0;

            const double eps = 1e-9;
            for (int ri = 0; ri < rects.Count; ri++)
            {
                var rect = rects[ri];
                int n = rect.Length;
                for (int ei = 0; ei < n; ei++)
                {
                    var A = rect[ei];
                    var B = rect[(ei + 1) % n];
                    var e0 = B - A;
                    double len0 = Math.Sqrt(e0.X * e0.X + e0.Y * e0.Y);
                    if (len0 < eps) continue;

                    // CCW 矩形：左法向 (−e.y, e.x) 是内法向，故外法向为 (e.y, −e.x)
                    var nOut = new Vector2d(e0.Y / len0, -e0.X / len0);
                    var A2 = new Point2d(A.X + delta * nOut.X, A.Y + delta * nOut.Y);
                    var B2 = new Point2d(B.X + delta * nOut.X, B.Y + delta * nOut.Y);

                    // 边的包围盒：与它不相交的矩形不可能覆盖这条边的任何部分，直接跳过。
                    // 纯加速手段，不改变结果（v4.4 加入底盘矩形后矩形数 +50%，这层预筛是必须的）。
                    double sMinX = Math.Min(A2.X, B2.X), sMaxX = Math.Max(A2.X, B2.X);
                    double sMinY = Math.Min(A2.Y, B2.Y), sMaxY = Math.Max(A2.Y, B2.Y);

                    var covered = new List<(double, double)>();
                    if (!useSpatialIndex)
                    {
                        for (int rj = 0; rj < rects.Count; rj++)
                        {
                            if (rj == ri) continue;
                            var bb = aabbs[rj];
                            if (sMaxX < bb[0] - eps || sMinX > bb[2] + eps ||
                                sMaxY < bb[1] - eps || sMinY > bb[3] + eps)
                                continue;
                            if (ClipInterval(A2, B2, nOut, delta, hss[rj],
                                    out double t0, out double t1) == true && t1 > t0)
                                covered.Add((t0, t1));
                        }
                    }
                    else
                    {
                    candidateStamp++;
                    if (candidateStamp == int.MaxValue)
                    {
                        Array.Clear(candidateMarks, 0, candidateMarks.Length);
                        candidateStamp = 1;
                    }
                    var candidates = new List<int>();
                    long sx0 = (long)Math.Floor((sMinX - eps) / cellSize);
                    long sx1 = (long)Math.Floor((sMaxX + eps) / cellSize);
                    long sy0 = (long)Math.Floor((sMinY - eps) / cellSize);
                    long sy1 = (long)Math.Floor((sMaxY + eps) / cellSize);
                    for (long gx = sx0; gx <= sx1; gx++)
                    for (long gy = sy0; gy <= sy1; gy++)
                    {
                        if (!spatial.TryGetValue((gx, gy), out List<int> bucket)) continue;
                        for (int bi = 0; bi < bucket.Count; bi++)
                        {
                            int rj = bucket[bi];
                            if (candidateMarks[rj] == candidateStamp) continue;
                            candidateMarks[rj] = candidateStamp;
                            candidates.Add(rj);
                        }
                    }
                    for (int bi = 0; bi < broadRects.Count; bi++)
                    {
                        int rj = broadRects[bi];
                        if (candidateMarks[rj] == candidateStamp) continue;
                        candidateMarks[rj] = candidateStamp;
                        candidates.Add(rj);
                    }

                    for (int ci = 0; ci < candidates.Count; ci++)
                    {
                        int rj = candidates[ci];
                        if (rj == ri) continue;
                        var bb = aabbs[rj];
                        if (sMaxX < bb[0] - eps || sMinX > bb[2] + eps ||
                            sMaxY < bb[1] - eps || sMinY > bb[3] + eps)
                            continue;
                        if (ClipInterval(A2, B2, nOut, delta, hss[rj], out double t0, out double t1) == true && t1 > t0)
                            covered.Add((t0, t1));
                    }
                    }

                    var exposed = SubtractIntervals(covered);
                    foreach (var iv in exposed)
                    {
                        if (iv.b - iv.a < eps) continue;
                        // 参数 t 对外推前后完全一致，端点仍取原始（未外推）边上的点
                        var P = A + (B - A) * iv.a;
                        var Q = A + (B - A) * iv.b;
                        segs.Add((P, Q));
                    }
                }
            }

            return MergeCollinear(segs);
        }

        /// <summary>
        /// 把共线且重叠的暴露子段合并成一条。
        ///
        /// 外推判别修好「共线互覆盖」后，拉直段里 ~230 个精确平移的驾驶室矩形会把
        /// 同一条走廊侧墙整条暴露出来 —— 230 条互相重叠的共线线段会给成环算法留下
        /// 460 个度为 1 的悬挂端点，照样拼不出环。并集边界上共线段的贡献就是它们在
        /// 该直线上的区间并集，所以按「直线身份 = (规范方向, 到原点有符号距离)」分组，
        /// 组内做一维区间并集即可。
        ///
        /// 只有真共线（方向差 <1e-9 rad、直线偏移差 <1e-4 m）才会被合并：
        /// 圆弧段的相邻台阶夹角是 0.25°（4.4e-3 rad），差着 7 个数量级，不受影响；
        /// 同一条边被不同矩形切成多段时，只要区间不重叠也保持独立。
        ///
        /// v4.5.1 修正分组方式（此前「刚性车 + 长直行」包络消失的直接原因）：
        /// 旧实现把 (ang, c) 一起排序，再用「与组内第一条比较」的双重判据扫描。
        /// 问题在于方向是**离散档位**：同一条直线的方向差只有 ~1e-14 rad，而相邻
        /// 步进的方向差是 4.4e-3 rad（0.25°），中间是 11 个数量级的空档。于是同一
        /// 档里所有平行线（走廊左壁、右壁 ……）的 ang 几乎相等，排序按 ang 排会把
        /// 不同 c 的线段**交错**串在一起：
        ///     (ang_a, c=-0.49) (ang_b, c=+2.01) (ang_c, c=-0.49) …
        /// 分组以第一条为锚，第二条 |Δc|=2.5 > cTol 就断组，第三条又是一个新组 ——
        /// 结果 44 条同线同向、互相重叠几十米的走廊壁被切成 5 个组、24 条被切成 4 个组，
        /// 区间并集根本来不及做。实测「刚性 10°+直行60m」：512→470→456→452 一路收敛，
        /// MergeCollinear 连自己的输出都合并不干净（不幂等），串环自然失败。
        /// 正解是两级聚类：先按方向聚（组内方向必然属于同一档），再在组内按 c 聚。
        /// 修完 470 条一步就并成 178 条，且幂等。
        /// </summary>
        private static List<(Point2d P, Point2d Q)> MergeCollinear(List<(Point2d P, Point2d Q)> segs)
        {
            const double angTol = 1e-9;
            const double cTol = 1e-4;
            const double touchTol = 1e-12;

            var items = new List<(double ang, double c, double s0, double s1, double dx, double dy, int src)>();
            for (int si = 0; si < segs.Count; si++)
            {
                var s = segs[si];
                double ex = s.Q.X - s.P.X, ey = s.Q.Y - s.P.Y;
                double len = Math.Sqrt(ex * ex + ey * ey);
                if (len < 1e-12) continue;
                double dx = ex / len, dy = ey / len;
                // 规范方向：保证同一条直线上的任意两段得到完全相同的 (dx,dy)
                if (dx < 0.0 || (dx == 0.0 && dy < 0.0)) { dx = -dx; dy = -dy; }
                double c = dx * s.P.Y - dy * s.P.X;      // 直线身份（原点到直线的有符号距离）
                double s0 = dx * s.P.X + dy * s.P.Y;
                double s1 = dx * s.Q.X + dy * s.Q.Y;
                if (s0 > s1) { double tt = s0; s0 = s1; s1 = tt; }
                items.Add((Math.Atan2(dy, dx), c, s0, s1, dx, dy, si));
            }
            if (items.Count < 2) return segs;

            // 第一级：按方向聚类。方向是离散档位，用「与锚点比较」绝对判据即可，
            //        不存在上面说的交错问题（档内 ang 差 ≤1e-14，档间差 ≥4.4e-3）。
            items.Sort((a, b) => a.ang.CompareTo(b.ang));

            var res = new List<(Point2d, Point2d)>();
            int i = 0;
            while (i < items.Count)
            {
                double ang = items[i].ang;
                int j = i;
                while (j < items.Count && Math.Abs(items[j].ang - ang) <= angTol) j++;

                // 第二级：档内按 c 聚类（此时只剩「哪几条线」这一个自由度，排序有意义）
                var sub = items.GetRange(i, j - i);
                sub.Sort((a, b) => a.c.CompareTo(b.c));
                int k = 0;
                while (k < sub.Count)
                {
                    double c = sub[k].c;
                    double dx = sub[k].dx, dy = sub[k].dy;
                    var iv = new List<(double a, double b, int src)>();
                    int k2 = k;
                    while (k2 < sub.Count && Math.Abs(sub[k2].c - c) <= cTol)
                    {
                        iv.Add((sub[k2].s0, sub[k2].s1, sub[k2].src));
                        k2++;
                    }

                    iv.Sort((a, b) => a.a.CompareTo(b.a));
                    var merged = new List<(double a, double b, int src, int cnt)>();
                    foreach (var t in iv)
                    {
                        if (merged.Count > 0 && t.a <= merged[merged.Count - 1].b + touchTol)
                        {
                            var last = merged[merged.Count - 1];
                            merged[merged.Count - 1] = (last.a, Math.Max(last.b, t.b), last.src, last.cnt + 1);
                        }
                        else merged.Add((t.a, t.b, t.src, 1));
                    }

                    // 直线上距离原点最近的点 foot = (−dy·c, dx·c)，满足 cross((dx,dy),foot)=c
                    var foot = new Point2d(-dy * c, dx * c);
                    foreach (var t in merged)
                    {
                        // v4.5.1：只有真正把多条段并成一条时才做「投影 → 重建」。
                        // 重建会把线段挪到组内第一条所在的直线上，横向漂移最大 cTol=1e-4 m ——
                        // 是成环聚类容差 2e-5 的 5 倍，足以让本该首尾相接的两个顶点接不上。
                        // 单独一段原样输出，零漂移。
                        if (t.cnt == 1)
                        {
                            res.Add(segs[t.src]);
                            continue;
                        }
                        res.Add((new Point2d(foot.X + dx * t.a, foot.Y + dy * t.a),
                                 new Point2d(foot.X + dx * t.b, foot.Y + dy * t.b)));
                    }
                    k = k2;
                }
                i = j;
            }
            return res;
        }



        /// <summary>
        /// 按共享端点把暴露子段串成闭合环，返回所有环。
        ///
        /// v4.5 重写。旧实现是「每个端点贪心找 tol 内最近邻、互为伙伴」：一旦有第三个点
        /// 更近，就会配错（把 A 段终点配到 B 段终点，而不是它真正该接的 C 段起点），
        /// 链在那个顶点断掉。铰接车 90° 转弯 + 25m 拉直这条路径上，拉直段是指数收敛的
        /// （theta(s)=2·atan(tan(theta0/2)·exp(-s/L2))），末尾相邻帧车体差异 < 1e-6，
        /// 大量端点挤在 1e-6 里互相抢伙伴 —— 实测 1503 条暴露段一条环都串不出来
        /// （loops=0），于是 TRUCKTURN90 对铰接车完全不显示包络。
        ///
        /// 新实现不用「最近邻」这种局部判据，分两步：
        ///   1) 端点聚类：空间网格 + 并查集，tol 内的端点全部并成一个规范顶点。
        ///      几何上重合的点必然同簇，不存在"选错伙伴"；簇心取算术平均以消除抖动。
        ///   2) 建图。
        ///   3) 剥悬挂边：反复删掉度为 1 的边（它不可能属于任何环）。
        ///   4) 图上游走：每段变成无向边 (u,v)，在每个顶点选「相对入射方向最顺时针」
        ///      的未用边。这条规则等价于让并集材料始终留在行进方向左侧，
        ///      走出来的就是 CCW 外边界；度数 > 2 的粘合点（两矩形角对角相接）
        ///      也能正确剥离开，不会像旧实现那样把两条环绞断。
        ///      走不通时回滚已用边（详见代码内注释）。
        ///
        /// v4.5.1 补两条鲁棒性规则（缺一不可，实测 84 个连续段组合里 28 个整条包络消失）：
        ///   3) 剥悬挂边 —— 见步骤 3 处注释。
        ///   4) 失败回滚 —— 见代码内注释。
        /// </summary>
        private static List<List<Point2d>> ChainBoundaryLoops(List<(Point2d P, Point2d Q)> segs)
        {
            var loops = new List<List<Point2d>>();
            int m = segs.Count;
            if (m == 0) return loops;

            // 聚类容差 2e-5 m（0.02mm）。必须覆盖 ExposedBoundarySegments 里外推
            // delta=1e-9 带来的端点偏移 delta/|tan(theta)|：最坏 theta=4.2e-4 rad
            // （拉直段末段挂车每帧转角）→ 2.4e-6 m，这里留了 8 倍余量。
            // 真实相邻顶点间距 ≥ 2.4mm（同一量级台阶的径向幅值），不会被误合并。
            const double tol = 2e-5;
            double cell = tol;
            double tol2 = tol * tol;
            int n = m * 2;

            int Key(double v) => (int)Math.Floor(v / cell);

            // ---- 1) 端点聚类（并查集）----
            var px = new double[n];
            var py = new double[n];
            for (int i = 0; i < m; i++)
            {
                px[2 * i] = segs[i].P.X; py[2 * i] = segs[i].P.Y;
                px[2 * i + 1] = segs[i].Q.X; py[2 * i + 1] = segs[i].Q.Y;
            }

            var grid = new Dictionary<(int, int), List<int>>();
            for (int i = 0; i < n; i++)
            {
                var k = (Key(px[i]), Key(py[i]));
                if (!grid.TryGetValue(k, out var lst)) { lst = new List<int>(); grid[k] = lst; }
                lst.Add(i);
            }

            var parent = new int[n];
            for (int i = 0; i < n; i++) parent[i] = i;
            int Find(int a) { while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; }
            void Union(int a, int b) { a = Find(a); b = Find(b); if (a != b) parent[b] = a; }

            // 只查 3x3 邻域：cell == tol，落在 tol 内的点必然在邻域内。
            // 复杂度 O(n · 邻域内点数)，实测 3000 端点毫秒级，与旧实现的 Register 同量级。
            for (int i = 0; i < n; i++)
            {
                int gx = Key(px[i]), gy = Key(py[i]);
                for (int dx = -1; dx <= 1; dx++)
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        if (!grid.TryGetValue((gx + dx, gy + dy), out var lst)) continue;
                        foreach (int j in lst)
                        {
                            if (j <= i) continue;
                            double ddx = px[j] - px[i], ddy = py[j] - py[i];
                            if (ddx * ddx + ddy * ddy <= tol2) Union(i, j);
                        }
                    }
            }

            // 规范顶点：簇内算术平均
            var idOf = new Dictionary<int, int>();
            var cx = new List<double>();
            var cy = new List<double>();
            var cnt = new List<int>();
            for (int i = 0; i < n; i++)
            {
                int r = Find(i);
                if (!idOf.TryGetValue(r, out int id))
                {
                    id = cx.Count;
                    idOf[r] = id;
                    cx.Add(0.0); cy.Add(0.0); cnt.Add(0);
                }
                cx[id] += px[i]; cy[id] += py[i]; cnt[id]++;
            }
            int nv = cx.Count;
            for (int i = 0; i < nv; i++) { cx[i] /= cnt[i]; cy[i] /= cnt[i]; }

            // ---- 2) 建图 ----
            var eu = new List<int>();
            var ev = new List<int>();
            for (int i = 0; i < m; i++)
            {
                int u = idOf[Find(2 * i)];
                int v = idOf[Find(2 * i + 1)];
                if (u == v) continue; // 退化：拉直段末尾车体几乎重合产生的碎片
                eu.Add(u); ev.Add(v);
            }
            int ne = eu.Count;
            if (ne < 3) return loops;

            var adj = new List<int>[nv];
            for (int i = 0; i < nv; i++) adj[i] = new List<int>();
            for (int e = 0; e < ne; e++) { adj[eu[e]].Add(e); adj[ev[e]].Add(e); }

            // 从边 e 的 v 端走出去，返回另一端
            int Other(int e, int v) => eu[e] == v ? ev[e] : eu[e];

            // =================================================================
            // 3) 剥悬挂边（v4.5.1 新增，成环鲁棒性的关键一步）
            //
            // 度为 1 的顶点上那条边不可能属于任何环（进得去出不来），可以安全地整条删掉；
            // 删掉后邻居度减 1，可能又变成 1，用队列反复剥，直到全图最小度 ≥ 2。
            // 剥完剩下的是若干「每点至少两条边」的连通块，每块必含环。
            //
            // 为什么不剥就完蛋：外边界图里只要混进一个度为 1 的碎片段端点，
            // 游走一旦从它起步（或半路拐进去），就会一直走到另一个死胡同才停；
            // 这一路经过的全是**真正的外边界**，却被永久标记成 used。
            // 后续再也拼不回完整环 → loops=0 → EnvelopeTracks 返回空 →
            // CAD 里黄色包络整条消失（用户 v4.5 报的「连续转弯有概率包络消失」）。
            // 实测「刚性 10°+直行1m+10°」：94 帧里 328 个顶点有 321 个度为 2，
            // 只因 6 个度为 1 的碎片段就把整条边界吃掉，一条环都出不来。
            // =================================================================
            var live = new bool[ne];
            for (int e = 0; e < ne; e++) live[e] = true;
            var deg = new int[nv];
            for (int e = 0; e < ne; e++) { deg[eu[e]]++; deg[ev[e]]++; }

            var pending = new Queue<int>();
            for (int v = 0; v < nv; v++) if (deg[v] == 1) pending.Enqueue(v);
            while (pending.Count > 0)
            {
                int v = pending.Dequeue();
                if (deg[v] != 1) continue;
                int e = -1;
                foreach (int e2 in adj[v])
                    if (live[e2]) { e = e2; break; }
                if (e < 0) { deg[v] = 0; continue; }
                live[e] = false;
                int other = Other(e, v);
                deg[v] = 0;
                deg[other]--;
                if (deg[other] == 1) pending.Enqueue(other);
            }

            // ---- 4) 游走成环 ----
            var used = new bool[ne];
            // 起始边只试一次：失败的游走会回滚，若不记 tried 会在最坏情况下退化成 O(ne²)。
            var tried = new bool[ne];

            for (int e0 = 0; e0 < ne; e0++)
            {
                if (!live[e0] || used[e0] || tried[e0]) continue;
                tried[e0] = true;

                int startV = eu[e0];
                int v = startV;
                int e = e0;
                double inX = 0.0, inY = 0.0;
                var poly = new List<Point2d>();
                var walked = new List<int>();
                bool closed = false;

                // 起步：沿 e0 从 eu 走到 ev，建立入射方向
                {
                    int nxt = ev[e0];
                    double dx = cx[nxt] - cx[v], dy = cy[nxt] - cy[v];
                    double len = Math.Sqrt(dx * dx + dy * dy);
                    if (len < 1e-12) { continue; }
                    used[e0] = true; walked.Add(e0);
                    poly.Add(new Point2d(cx[v], cy[v]));
                    inX = dx / len; inY = dy / len;
                    v = nxt;
                }

                while (true)
                {
                    // 在顶点 v 上选「相对入射方向最顺时针」的未用边
                    int bestE = -1;
                    double bestTurn = double.MinValue; // turn = atan2(-cross, dot)，越大越顺时针
                    foreach (int e2 in adj[v])
                    {
                        if (!live[e2] || used[e2] || e2 == e) continue;
                        int o = Other(e2, v);
                        double ox = cx[o] - cx[v], oy = cy[o] - cy[v];
                        double ol = Math.Sqrt(ox * ox + oy * oy);
                        if (ol < 1e-12) continue;
                        ox /= ol; oy /= ol;
                        double cross = inX * oy - inY * ox;
                        double dot = inX * ox + inY * oy;
                        double turn = Math.Atan2(-cross, dot);
                        if (turn > bestTurn) { bestTurn = turn; bestE = e2; }
                    }
                    if (bestE < 0) break; // 断链：奇度顶点上偶有发生，靠回滚兜底

                    poly.Add(new Point2d(cx[v], cy[v]));
                    e = bestE;
                    used[e] = true; walked.Add(e);
                    int nv2 = Other(e, v);
                    double dx2 = cx[nv2] - cx[v], dy2 = cy[nv2] - cy[v];
                    double len2 = Math.Sqrt(dx2 * dx2 + dy2 * dy2);
                    if (len2 < 1e-12) break;
                    inX = dx2 / len2; inY = dy2 / len2;
                    v = nv2;

                    if (v == startV) { closed = true; break; }
                }

                if (closed && poly.Count >= 3)
                {
                    loops.Add(poly);
                }
                else
                {
                    // 回滚（v4.5.1）：一次失败的游走往往已经走过了几百条真正的外边界边。
                    // 不把它们还回去，后面的起始边就永远拼不出完整环（这正是 loops=0 的直接原因）。
                    for (int wi = 0; wi < walked.Count; wi++) used[walked[wi]] = false;
                }
            }

            return loops;
        }



        private static double PolygonArea(List<Point2d> poly)
        {
            if (poly == null || poly.Count < 3) return 0.0;
            double s = 0.0;
            int n = poly.Count;
            for (int i = 0; i < n; i++)
            {
                var a = poly[i];
                var b = poly[(i + 1) % n];
                s += a.X * b.Y - b.X * a.Y;
            }
            return Math.Abs(s) * 0.5;
        }

        #endregion

        /// <summary>Chaikin 角点切割平滑（开放折线，端点不动）。iter 次迭代。</summary>
        private static List<Point2d> ChaikinSmooth(List<Point2d> poly, int iter)
        {
            if (poly == null || poly.Count < 3 || iter <= 0) return poly;
            var cur = new List<Point2d>(poly);
            for (int it = 0; it < iter; it++)
            {
                var next = new List<Point2d>();
                next.Add(cur[0]);
                for (int i = 0; i < cur.Count - 1; i++)
                {
                    Point2d a = cur[i], b = cur[i + 1];
                    next.Add(new Point2d(0.75 * a.X + 0.25 * b.X, 0.75 * a.Y + 0.25 * b.Y));
                    next.Add(new Point2d(0.25 * a.X + 0.75 * b.X, 0.25 * a.Y + 0.75 * b.Y));
                }
                next.Add(cur[cur.Count - 1]);
                cur = next;
            }
            return cur;
        }

        #region 轮胎外观（v4.0）
        /// <summary>当前前轮转角（弧度）。由轴距与转弯半径反推，与路径曲率一致。</summary>
        public static double FrontSteerAngle(VehicleParams p)
        {
            double R = TurnRadius(p);
            return Math.Atan(p.TractorWheelbase / R);
        }

        /// <summary>计算一个轮胎的四个角点（矩形）。
        /// center：轮胎接地面中心；heading：轮胎滚动方向（已含前轮转角）；
        /// tireLen：滚动方向长度；tireWidth：横向宽度。</summary>
        public static Point2d[] TireRect(Point2d center, Vector2d heading, double tireLen, double tireWidth)
        {
            Vector2d u = heading.GetNormal();
            Vector2d n = new Vector2d(-u.Y, u.X);
            double hl = tireLen / 2.0;
            double hw = tireWidth / 2.0;
            return new Point2d[]
            {
                center + hl * u + hw * n,
                center + hl * u - hw * n,
                center - hl * u - hw * n,
                center - hl * u + hw * n
            };
        }

        /// <summary>返回一帧车辆的全部轮胎矩形：牵引车前轴（可转向）、后轴，挂车后轴（铰接）。
        ///  steerSign: +1 左转（前轮左偏）、-1 右转（前轮右偏）、0 直行（前轮与车头平行）。
        ///  tireLen/tireWidth 为 CAD 单位。</summary>
        public static List<Point2d[]> TireRects(Frame f, VehicleParams p, double tireLen, double tireWidth, int steerSign)
        {
            var res = new List<Point2d[]>();
            double half = p.WheelTrack / 2.0;
            Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);

            // 前轴中心与前轮转角
            Point2d frontAxle = f.O1 + p.TractorWheelbase * f.U1;
            double steer = 0.0;
            if (steerSign != 0) steer = steerSign * FrontSteerAngle(p);
            double phi1 = Math.Atan2(f.U1.Y, f.U1.X);
            Vector2d frontTireDir = new Vector2d(Math.Cos(phi1 + steer), Math.Sin(phi1 + steer));

            Point2d fl = frontAxle + half * n1;
            Point2d fr = frontAxle - half * n1;
            res.Add(TireRect(fl, frontTireDir, tireLen, tireWidth));
            res.Add(TireRect(fr, frontTireDir, tireLen, tireWidth));

            // 后轴轮胎（与牵引车航向一致）
            Point2d rl = f.O1 + half * n1;
            Point2d rr = f.O1 - half * n1;
            res.Add(TireRect(rl, f.U1, tireLen, tireWidth));
            res.Add(TireRect(rr, f.U1, tireLen, tireWidth));

            // 挂车后轴轮胎（与挂车航向一致）
            if (p.Articulated)
            {
                Vector2d n2 = new Vector2d(-f.U2.Y, f.U2.X);
                Point2d tl = f.O2 + half * n2;
                Point2d tr = f.O2 - half * n2;
                res.Add(TireRect(tl, f.U2, tireLen, tireWidth));
                res.Add(TireRect(tr, f.U2, tireLen, tireWidth));
            }
            return res;
        }

        /// <summary>由帧序列计算每帧的前轮转向符号（+1 左转，-1 右转，0 直行）。</summary>
        public static int[] ComputeSteerSigns(List<Frame> frames)
        {
            int n = frames.Count;
            var signs = new int[n];
            for (int i = 0; i < n - 1; i++)
            {
                double cr = frames[i].U1.X * frames[i + 1].U1.Y - frames[i].U1.Y * frames[i + 1].U1.X;
                if (Math.Abs(cr) > 1e-9) signs[i] = cr > 0 ? 1 : -1;
            }
            signs[n - 1] = signs[n - 2];
            // 消除孤立的 0（极小段直行）
            for (int i = 1; i < n; i++)
                if (signs[i] == 0 && signs[i - 1] != 0) signs[i] = signs[i - 1];
            for (int i = n - 2; i >= 0; i--)
                if (signs[i] == 0 && signs[i + 1] != 0) signs[i] = signs[i + 1];
            return signs;
        }

        /// <summary>
        /// 判断帧序列是否沿同一圆心做圆周运动（刚性/牵引车）。若是，返回圆心与半径。
        /// 用前两帧后轴中心求中垂线交点，再校验所有后轴中心到圆心距离一致。
        /// 直行或退化情况返回 false。
        /// </summary>
        public static bool IsCircularPath(List<Frame> frames, out Point2d center, out double radius)
        {
            center = default;
            radius = 0;
            if (frames == null || frames.Count < 3) return false;

            var f0 = frames[0];
            var f1 = frames[1];
            // 圆心必在过 O1[0] 且垂直于 U1[0] 的直线上；又在中垂线上。
            Vector2d n = new Vector2d(-f0.U1.Y, f0.U1.X); // U1 的左法向
            Vector2d d = f1.O1 - f0.O1;
            double nd = n.X * d.X + n.Y * d.Y;
            if (Math.Abs(nd) < 1e-12) return false; // 两帧共线/直行
            double t = (d.X * d.X + d.Y * d.Y) / (2.0 * nd);
            center = f0.O1 + t * n;
            radius = Math.Abs(t);
            if (radius < 1e-3) return false;

            // 校验：所有 O1 到圆心距离与 radius 偏差 < 0.1%
            double tol = Math.Max(radius * 1e-3, 1e-6);
            for (int i = 2; i < frames.Count; i++)
            {
                var dc = frames[i].O1 - center;
                double r = Math.Sqrt(dc.X * dc.X + dc.Y * dc.Y);
                if (Math.Abs(r - radius) > tol) return false;
            }
            return true;
        }

        /// <summary>
        /// 判断挂车后轴 O2 是否绕固定圆心做圆周运动（稳态转弯时成立）。
        /// 与牵引车用同一套几何：圆心在过 O2 且垂直于 U2 的直线上，再解中垂线交点。
        /// 注意 O2 绕的是 <b>C2</b>，与牵引车的 C1 不同 —— 这个差值就是 off-tracking。
        /// </summary>
        private static bool IsCircularTrailerPath(List<Frame> frames, out Point2d center, out double radius)
        {
            center = default;
            radius = 0;
            if (frames == null || frames.Count < 3) return false;

            var f0 = frames[0];
            var f1 = frames[1];
            Vector2d n = new Vector2d(-f0.U2.Y, f0.U2.X); // U2 的左法向
            Vector2d d = f1.O2 - f0.O2;
            double nd = n.X * d.X + n.Y * d.Y;
            if (Math.Abs(nd) < 1e-12) return false;
            double t = (d.X * d.X + d.Y * d.Y) / (2.0 * nd);
            center = f0.O2 + t * n;
            radius = Math.Abs(t);
            if (radius < 1e-3) return false;

            double tol = Math.Max(radius * 1e-3, 1e-6);
            for (int i = 2; i < frames.Count; i++)
            {
                var dc = frames[i].O2 - center;
                double r = Math.Sqrt(dc.X * dc.X + dc.Y * dc.Y);
                if (Math.Abs(r - radius) > tol) return false;
            }
            return true;
        }

        /// <summary>
        /// 由相邻两帧的位姿求瞬时圆心：刚体上一点 P 沿方向 u 运动且无侧滑，其瞬时中心
        /// 必在「过 P 且垂直于 u」的直线上，再由 PP' 的中垂线交点确定。
        /// 与 IsCircularPath 完全同一套几何，只是<b>不做</b>「全程同一个圆心」的校验，
        /// 因此也适用于转弯起始的瞬态段（挂车圆心逐帧漂移）。
        /// </summary>
        static bool InstantCenter(Point2d p0, Vector2d u0, Point2d p1, out Point2d center, out double radius)
        {
            center = default;
            radius = 0;
            Vector2d n = new Vector2d(-u0.Y, u0.X); // u0 的左法向
            Vector2d d = p1 - p0;
            double nd = n.X * d.X + n.Y * d.Y;
            if (Math.Abs(nd) < 1e-9) return false; // 直行/共线，圆心在无穷远
            double t = (d.X * d.X + d.Y * d.Y) / (2.0 * nd);
            center = p0 + t * n;
            radius = Math.Abs(t);
            // 半径上限 500m：更大的"圆"在米级车辆图上就是直线，交给通用拟合处理；
            // 也顺带滤掉转弯起始瞬间（铰接角为 0、挂车瞬时直行）的无穷远圆心。
            return radius > 1e-3 && radius < 5e2;
        }

        static void AddCenter(List<Point2d> list, Point2d c, double minDist)
        {
            if (double.IsNaN(c.X) || double.IsNaN(c.Y)) return;
            foreach (var e in list)
            {
                var dv = c - e;
                if (dv.Length < minDist) return;
            }
            list.Add(c);
        }

        /// <summary>
        /// 收集这一段路径上所有「刚体做圆周运动」的圆心候选，供 bulge 圆弧拟合使用。
        /// 刚性车/牵引车给出 C1；铰接车额外给出挂车的 C2（稳态）或一串瞬时圆心（瞬态）。
        /// 返回空列表表示该段不是圆周运动（含直行），调用方应改用通用圆弧拟合。
        ///
        /// 为什么要一串瞬时圆心：从直行起转时铰接角由 0 渐增到稳态，挂车并不绕固定圆心
        /// 转动，IsCircularTrailerPath 会判 false。此时若只给牵引车的 C1，挂车那一段外廓
        /// 一条弧都拟合不出来 —— 实测 1215 个顶点里会残留 128 个近 90° 的离散台阶折角
        /// （也就是用户看到的黄色锯齿）。沿路径均匀采样 ~20 个挂车瞬时圆心后即可覆盖。
        /// </summary>
        public static List<Point2d> TurnCenters(List<Frame> frames, VehicleParams p)
        {
            var list = new List<Point2d>();
            if (IsCircularPath(frames, out Point2d c1, out double _))
                list.Add(c1);

            if (p != null && p.Articulated && frames != null && frames.Count >= 3)
            {
                if (IsCircularTrailerPath(frames, out Point2d c2, out double _))
                {
                    AddCenter(list, c2, 1e-6);
                }
                else
                {
                    // 采样越密，越能贴合挂车瞬心沿路径的连续漂移，拟合容差就可以取得越小，
                    // 弧的外扩量也随之越小。代价可忽略：对「错误圆心」半径检查会立刻 break，
                    // 单次尝试平均只跑 2~3 个 j，实测 361 帧 × 200 个圆心仍是毫秒级。
                    int stride = Math.Max(1, (frames.Count - 2) / 120);
                    for (int k = 1; k + 1 < frames.Count; k += stride)
                    {
                        if (InstantCenter(frames[k].O2, frames[k].U2, frames[k + 1].O2,
                                out Point2d ic, out double _))
                            AddCenter(list, ic, 0.25);
                    }
                }
            }
            return list;
        }
        #endregion
    }
}
