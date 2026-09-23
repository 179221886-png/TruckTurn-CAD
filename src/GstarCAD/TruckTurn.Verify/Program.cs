using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>
    /// v4.3 并集包络引擎的 C# 侧数值校验（脱离浩辰CAD 运行）。
    /// 编译的是 ../TruckTurn 下的真实源码，不是副本，因此验证结论对实际发布的 DLL 有效。
    ///
    /// 判定正确性靠两条不变量，不需要看图：
    ///   1) corners_outside = 0                —— 所有帧的车体角点都被包络覆盖（包络不内缩）
    ///   2) boundary_pts_strictly_inside = 0   —— 包络顶点没有一个落在车体内部（无虚构边界点）
    /// 另看 max_turn：只应等于车体自身直角 90°，出现更大值即为伪尖刺。
    /// </summary>
    static partial class VerifyProgram
    {
        // 几何容差：1 mm。
        // 不能取 1e-6：bulge 圆弧拟合会把弧向外偏置最多 tol(=1e-4 m) 以保证不内缩包络，
        // 由此带来的 0.01~0.1mm 级「越界/内侵」是数值副产品，对米级车辆图毫无意义。
        // 真正的问题（历史 bug 表现为米级内缩、140°~180° 伪尖刺）远大于 1mm，仍会被抓到。
        const double Tol = 1e-3;

        // bulge 圆弧拟合的独立容差。
        // 拟合器按设计会把弧向外推到窗口内最大半径（消除离散帧并集的 3cm 级台阶），
        // 因此"外扩"是预期行为而非缺陷，判据必须比 Tol 宽，否则会把正确结果误判为失败。
        // 5cm 仍然远小于工程上扫掠路径 0.3~0.5m 的安全净距，也远小于历史 bug 的米级误差。
        const double FitTol = 5e-2;

        /// <summary>当前生效的几何容差：原始并集边界用 Tol，bulge 拟合结果用 FitTol。</summary>
        static double s_geomTol = Tol;




        static int Main(string[] args)
        {
            Console.OutputEncoding = Encoding.UTF8;

            // Phase 1 跨运车运动学快速回归：不启动 CAD，不跑原货车长耗时用例。
            if (args.Length > 0 && args[0] == "carrier")
            {
                int carrierFail = RunCarrierPhase1();
                return carrierFail == 0 ? 0 : 1;
            }
            if (args.Length > 0 && args[0] == "carrierperf")
            {
                RunCarrierPreviewPerformance();
                return 0;
            }
            if (args.Length > 0 && args[0] == "truckperf")
            {
                RunTruckPreviewPerformance();
                return 0;
            }
            if (args.Length > 0 && args[0] == "trucklongperf")
            {
                RunTruckLongEnvelopePerformance();
                return 0;
            }
            if (args.Length > 0 && args[0] == "truckpreview")
            {
                return RunTruckPreviewKeyframes() == 0 ? 0 : 1;
            }

            // 快速迭代用：`dotnet TruckTurn.Verify.dll kink` 只跑折角归因，跳过全套回归
            if (args.Length > 0 && args[0] == "kink")
            {
                Console.WriteLine("==== 折角归因（单跑）====\n");
                RunKinkAttribution();
                return 0;
            }
            if (args.Length > 0 && args[0] == "sweep")
            {
                Console.WriteLine("==== 方案三调参：平滑迭代次数 × RDP 容差（单跑）====\n");
                RunSplineParamSweep();
                return 0;
            }
            if (args.Length > 0 && args[0] == "turn90")
            {
                TruckTurn.VerifyProgram._dumpTurn90();
                return 0;
            }
            if (args.Length > 0 && args[0] == "dump_env")
            {
                TruckTurn.Verify.DumpEnvelope.Run();
                return 0;
            }
            if (args.Length > 0 && args[0] == "scurve")
            {
                _dumpSCurve();
                return 0;
            }
            if (args.Length > 0 && args[0] == "revrigid")
            {
                _dumpRevRigid();
                return 0;
            }
            if (args.Length > 0 && args[0] == "radii")
            {
                _dumpRadii();
                return 0;
            }
            if (args.Length > 0 && args[0] == "revsteep")
            {
                _dumpRevSteep();
                return 0;
            }
            if (args.Length > 0 && args[0] == "revfull")
            {
                _dumpRevFull();
                return 0;
            }
            if (args.Length > 0 && args[0] == "sliver")
            {
                _dumpStraightSliver();
                return 0;
            }
            if (args.Length > 0 && args[0] == "straightenv")
            {
                _dumpStraightEnv();
                return 0;
            }
            if (args.Length > 0 && args[0] == "rigidbody")
            {
                _dumpRigidBody();
                return 0;
            }
            if (args.Length > 0 && args[0] == "fillet")
            {
                Console.WriteLine("==== 方案 B 外倒角实验（单跑）====\n");
                RunFilletPlanB();
                return 0;
            }

            Console.WriteLine("==== v4.3 并集包络引擎 C# 校验（链接主工程真实源码）====\n");

            int fail = 0;

            // ---- 与 Python 原型同采样步长(1°)，用于逐项对齐 ----
            Console.WriteLine("-- 与 Python 原型对齐（step=1.0°）--");
            fail += Run("刚性", articulated: false, 45.0, 1.0);
            fail += Run("刚性", articulated: false, 90.0, 1.0);
            fail += Run("铰接", articulated: true, 45.0, 1.0);
            fail += Run("铰接", articulated: true, 90.0, 1.0);

            // ---- 生产实际采样步长(0.25°)，即 CAD 里真正画出来的包络 ----
            Console.WriteLine("\n-- 生产采样（step=0.25°，CAD 实际出图）--");
            fail += Run("刚性", articulated: false, 90.0, TruckKinematics.FineStepDeg);
            fail += Run("铰接", articulated: true, 90.0, TruckKinematics.FineStepDeg);

            // ---- v4.4 新增验证：90°拉直 + 无车体穿透 ----
            Console.WriteLine("\n-- v4.4 新增：90°转弯并拉直挂车 + 车体无穿透 --");
            fail += RunStraighten("铰接-90°拉直", articulated: true, 90.0, TruckKinematics.FineStepDeg);
            fail += RunNoPenetration("铰接-无穿透", articulated: true, 90.0, TruckKinematics.FineStepDeg);

            // ---- v4.5 新增：TRUCKTURN90 整条路径（转弯+拉直）的包络必须能画出来 ----
            // 用户反馈「铰链车转动 90° 不显示包络」，纯圆弧场景 Run() 却一直是 OK，
            // 说明问题出在「圆弧 + 25m 直行」这条混合路径上，必须单独校验。
            Console.WriteLine("\n-- v4.5 新增：TRUCKTURN90 混合路径（转弯+拉直）的包络拟合 --");
            fail += RunStraightenEnvelope("铰接-90°包络", articulated: true, TruckKinematics.FineStepDeg);
            fail += RunStraightenEnvelope("刚性-90°包络", articulated: false, TruckKinematics.FineStepDeg);

            // ---- v4.5.1 新增：连续多段（TRUCKDRIVE 段间）包络成环扫描 ----
            // 用户 v4.5 反馈「连续转弯有概率黄色包络消失」。TRUCKDRIVE 每段确认后立刻
            // 单独画该段包络，段间用 SimulateFromStateFront 续接（携带挂车航向）。
            // 单看一个角度抓不到「有概率」，这里扫车型 × 转角 × 直行长度的参数网格。
            Console.WriteLine("\n-- v4.5.1 新增：连续多段包络成环扫描（车型 × 转角 × 直行长度）--");
            DumpAdaptiveStraightBug();
            fail += RunConsecutiveSegments();

            // ---- v4.5.2 新增：用户报"某些段黄色包络仍是折线" ----
            // 复刻用户截图2的多段路径：左转 90° + 拉直 + 右转 90° + 拉直，
            // 检查拟合后的 envPts 顶点数（>150 即还是折线锯齿）。
            Console.WriteLine("\n-- v4.5.2 新增：多段急转+L拉直的包络拟合（复刻用户截图2右侧场景）--");
            fail += RunMultiSharpTurns();

            // ---- v4.7 新增：TRUCKDRIVE 光标反解不得「吸」到固定 90° ----
            // 用户反馈「指针划到大概 90° 转弯的时候会自动吸附过去」。回退分支原本写死 90°，
            // 改为沿最小半径圆尽量转向光标后，必须验证跨可行性边界仍然连续、且角度确实随光标变。
            Console.WriteLine("\n-- v4.7 新增：最小半径回退的连续性与非锁死（光标反解不再硬跳 90°）--");
            fail += RunMinRadiusContinuity();

            // ---- v4.8 新增：预览步长 1°→0.5° 对包络锯齿的改善 ----
            Console.WriteLine("\n-- v4.8 新增：预览步长对包络锯齿的影响 --");
            fail += RunPreviewStepSmoothness();

            // ---- v4.8 方案三：Spline 出图前的稀化 + 外推补偿 ----
            // fit-spline 是插值样条，精确穿过顶点。若直接把带 1~3cm 台阶的原始包络丢进去，
            // 锯齿只会从「尖角」变成「波浪」。所以出图前必须稀化，并保证外包（不内缩）。
            Console.WriteLine("\n-- v4.8 方案三：Spline 出图的稀化 + 外包补偿 --");
            fail += RunSplineSimplify();

            // ---- v4.9：倒退行驶 ----
            // 倒退不是「把前进路径反过来画」：瞬心 C 仍在转向侧，只是绕 C 的行进方向反过来，
            // 于是半径公式不变、方向判据多乘一个 dir。下面三条分别锁死几何、铰接角动力学、包络。
            Console.WriteLine("\n-- v4.9 新增：倒退行驶（TRUCKDRIVE 的 B 键 / TRUCKTURN90 的行驶方向选项）--");
            fail += RunReverseGeometry();
            fail += RunReverseJackknife();
            fail += RunReverseEnvelope();

            // v4.9 BUG 复现：直行+90°，dir=+1/-1 各跑一次
            fail += RunStraightTurnSvg();

            // v4.9.18：刚性近直行（自适应大R/∞）帧数与包络宽度不变量
            fail += RunRigidNearStraightEnvelope();

            // v5.2.2：货车实时预览只抽取关键帧；正式出图继续使用完整帧。
            fail += RunTruckPreviewKeyframes();

            // v5.2.3：连续路径车辆姿态显示模式必须可持久化，旧参数默认迁移为“仅首尾”。
            fail += RunVehicleOutputModeSerialization();

            // Phase 1：跨运车参数、四轮独立/联动转向、蟹行、横移和约束验证。
            fail += RunCarrierPhase1();

            Console.WriteLine();
            Console.WriteLine(fail == 0
                ? "结论：全部通过（两条不变量均为 0，最大折角 90°，挂车拉直，无穿透）"
                : "结论：存在 " + fail + " 个未通过场景！");

            return fail == 0 ? 0 : 1;
        }

        static int RunVehicleOutputModeSerialization()
        {
            Console.WriteLine("\n-- v5.2.3：货车节点车辆显示模式持久化 --");
            VehicleParams p = VehicleParams.Defaults();
            bool defaultOnlyEndpoints = !p.ShowAllNodeVehicles;
            p.ShowAllNodeVehicles = true;
            VehicleParams restored = VehicleParams.FromCsv(p.ToCsv());

            // 模拟升级前的 28 字段参数；缺少新字段时必须采用新默认“仅首尾”。
            string[] fields = p.ToCsv().Split(',');
            string oldCsv = string.Join(",", fields.Take(28));
            VehicleParams upgraded = VehicleParams.FromCsv(oldCsv);
            bool ok = defaultOnlyEndpoints && restored.ShowAllNodeVehicles &&
                      !upgraded.ShowAllNodeVehicles;
            Console.WriteLine(ok
                ? "  [PASS] 默认仅首尾；新参数可往返；旧参数安全迁移为仅首尾"
                : "  [FAIL] 节点车辆显示模式默认值或 CSV 持久化异常");
            return ok ? 0 : 1;
        }

        static int Run(string label, bool articulated, double turnDeg, double stepDeg)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = articulated;
            if (!articulated) p.RigidRearOverhang = 2.0;

            var startFront = new Point2d(0.0, 0.0);
            var heading = new Vector2d(1.0, 0.0);

            var frames = TruckKinematics.SimulateFromFrontAxle(
                p, startFront, heading, +1, turnDeg, 0.0, stepDeg);

            var env = TruckKinematics.EnvelopeTracks(frames, p);

            // ---- 原始并集边界指标 ----
            double area = PolygonArea(env);
            int cornersOutside = CountCornersOutside(frames, p, env);
            int strictlyInside = CountBoundaryPtsStrictlyInside(frames, p, env);
            double maxTurn = MaxTurnAngleDeg(env);


            // 折角上界：
            //  - 车体自身直角 90°；
            //  - 相邻两帧车身转角增量造成离散并集的固有台阶（≤ stepRot）；
            //  - v4.4 新增：铰接车底盘比挂车窄，二者接缝处会真实暴露一个 90°+铰接角 的折角。
            // 超过这个界才说明有真正的伪尖刺（历史 bug 表现为 140°~180°）。
            double stepRot = MaxStepRotationDeg(frames);
            double maxArt = articulated ? MaxArticulationDeg(frames) : 0.0;
            double bound = 90.0 + stepRot + maxArt + 1e-6;

            bool ok = env.Count >= 3 && cornersOutside == 0 && strictlyInside == 0 && maxTurn <= bound
                      && !TruckKinematics.EnvelopeFallbackUsed;

            // ---- v4.3.1：拟合成 bulge 弧后的指标 ----
            // 圆周运动用已知转弯中心做圆弧拟合；否则用通用拟合。
            // 注意：CAD 里只有生产采样（FineStepDeg = 0.25°）才做 bulge 拟合；
            //       预览采样（1.0°）只画折线，因此这里只对生产采样检查 bulge 拟合。
            bool checkBulgeFit = stepDeg <= TruckKinematics.FineStepDeg + 1e-9;
            List<Point2d> envFit = null;
            List<double> bulges = null;
            int cornersOutsideFit = 0, strictlyInsideFit = 0, bulgeArcs = 0;
            double maxTurnFit = 0.0;
            bool okFit = true;
            if (checkBulgeFit)
            {
                // 铰接车有两个瞬时中心（牵引车 C1、挂车 C2），必须都交给拟合器
                var centers = TruckKinematics.TurnCenters(frames, p);
                // 与生产（Commands.DrawSegmentTracks）共用同一个入口，
                // 否则校验的就不是实际出图的算法（v4.5 教训：两边各写一套判据，
                // 生产已加伪尖刺守卫，验证还在裸取顶点更少者，于是漏判 128° 尖刺）。
                GeometryUtil.FitEnvelopeBulges(env, centers, out envFit, out bulges);
                // 采样密度必须远大于 16：bulge 弧按弦采样会内缩一个 sagitta，
                // 16 段时可达 1.4mm，会被误判成"虚构内部点"。取 64 段后该误差 <0.1mm。
                var envSampled = SampleBulgePolyline(envFit, bulges, 64);
                // 外扩量：拟合结果跑到原始并集边界之外的最大距离。
                // 拟合器按设计会把弧推到窗口内最大半径，外扩是预期的；但它是"包络虚胖"，
                // 必须量化，否则会把"容差开到 20cm 换光滑"这种虚假优化当成胜利。
                double maxInf = 0.0;
                Point2d infPt = default;
                int infIdx = -1;
                for (int qi = 0; qi < envSampled.Count; qi++)
                {
                    var q = envSampled[qi];
                    if (PointInPolygon(q, env)) continue;
                    double d = DistanceToPolyline(q, env);
                    if (d > maxInf) { maxInf = d; infPt = q; infIdx = qi; }
                }
                s_maxInflation = maxInf;
                s_geomTol = FitTol;
                s_maxInsideDepth = 0.0;
                cornersOutsideFit = CountCornersOutside(frames, p, envSampled);
                strictlyInsideFit = CountBoundaryPtsStrictlyInside(frames, p, envSampled);
                s_geomTol = Tol;
                maxTurnFit = MaxTurnAngleDeg(envSampled);
                bulgeArcs = bulges.Count(b => Math.Abs(b) > 1e-6);
                okFit = envFit.Count >= 3 && cornersOutsideFit == 0 && strictlyInsideFit == 0 && maxTurnFit <= bound;
            }

            Console.WriteLine(
                string.Format(CultureInfo.InvariantCulture,
                    "  {0} turn={1,5:F0}deg step={2:F2} frames={3,4}  env_pts={4,5}  " +
                    "corners_outside={5}  strictly_inside={6}  max_turn={7,6:F2}deg (bound={8,6:F2})  area={9,9:F2}m2   {10}",
                    label, turnDeg, stepDeg, frames.Count, env.Count,
                    cornersOutside, strictlyInside, maxTurn, bound, area, ok ? "OK" : "*** FAIL ***"));

            if (checkBulgeFit)
            {
                Console.WriteLine(
                    string.Format(CultureInfo.InvariantCulture,
                        "       -> bulge_fit: vertices={0,4} arcs={1,3} sampled_pts={2,5}  " +
                        "corners_outside={3} strictly_inside={4} max_turn={5,6:F2}deg  max_out={6:E2}m max_in={7:E2}m max_inf={8:F3}m  "
                        + "zigzag={9}→{10} 直线段最大折角={11,5:F1}deg   {12}",
                        envFit.Count, bulgeArcs, SampleBulgePolyline(envFit, bulges, 64).Count,
                        cornersOutsideFit, strictlyInsideFit, maxTurnFit,
                        s_maxOutsideDist, s_maxInsideDepth, s_maxInflation,
                        CountZigzag(env), CountZigzag(SampleBulgePolyline(envFit, bulges, 64)),
                        MaxTurnInStraightRuns(envFit, bulges),
                        okFit ? "OK" : "*** FAIL ***"));
                // 仅在失败时展开详细诊断，保持正常输出干净
                if (!okFit)
                {
                    DumpSharpCorners(SampleBulgePolyline(envFit, bulges, 64), envFit, bulges, 6);
                    DumpBulgeStats(envFit, bulges, env);
                    DumpArcDeviation(envFit, bulges, env);
                }
            }

            WriteSvg(label, turnDeg, stepDeg, frames, p, env);
            if (checkBulgeFit && envFit != null)
                WriteSvgFit(label, turnDeg, stepDeg, frames, p, env, envFit, bulges);

            return (ok && okFit) ? 0 : 1;
        }

        // ================= v4.4 验证 1：TRUCKTURN90 拉直挂车 =================
        static int RunStraighten(string label, bool articulated, double turnDeg, double stepDeg)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = articulated;
            if (!articulated) p.RigidRearOverhang = 2.0;

            var startFront = new Point2d(0.0, 0.0);
            var heading = new Vector2d(1.0, 0.0);

            var frames = TruckKinematics.SimulateTurn90AndStraighten(p, startFront, heading, +1, stepDeg);
            var last = frames[frames.Count - 1];
            double phi1 = Math.Atan2(last.U1.Y, last.U1.X);
            double phi2 = Math.Atan2(last.U2.Y, last.U2.X);
            // v4.5：Math.Atan2 返回的是「弧度」，必须先转度再交给 NormalizeAngleDeg。
            // 之前漏了这一步，0.0575 弧度被当成 0.0575 度打印成 0.06，
            // 于是「已拉直到 0.06°」这个结论是假的 —— 实际残余铰接角 3.29°。
            double deltaDeg = Math.Abs(NormalizeAngleDeg((phi2 - phi1) * 180.0 / Math.PI));
            // 判据直接用生产常量，避免两侧各写各的阈值再次跑偏（v4.5 教训）。
            // +0.25° 留给离散积分的 residual：解析解算的是连续解，实际是 0.25° 一步走出来的。
            bool ok = deltaDeg <= TruckKinematics.StraightenToleranceDeg + 0.25;

            // 测一下直行段实际走了多少米（首尾两帧的前轴中心距离 - 弯段起点的前轴中心位置）
            int turnN = CountTurnFrames(frames);
            double straightMeters = 0;
            if (turnN + 1 < frames.Count)
            {
                var p1 = TruckKinematics.FrontAxle(frames[turnN - 1], p);
                var p2 = TruckKinematics.FrontAxle(frames[frames.Count - 1], p);
                straightMeters = Math.Sqrt((p2.X - p1.X) * (p2.X - p1.X) + (p2.Y - p1.Y) * (p2.Y - p1.Y));
            }

            Console.WriteLine(
                string.Format(CultureInfo.InvariantCulture,
                    "  {0} frames={1,4}  final_articulation={2,6:F2}deg  turn_frames={3,4}  straight_frames={4,4}  straight_m={5:F1}m   {6}",
                    label, frames.Count, deltaDeg,
                    turnN, frames.Count - turnN, straightMeters,
                    ok ? "OK" : "*** FAIL ***"));
            return ok ? 0 : 1;
        }

        // ================= v4.5.1 验证：连续多段（典型 TRUCKDRIVE 段间）====================
        //
        // 用户 v4.5 反馈「连续转弯有概率黄色包络消失」。TRUCKDRIVE 在每段确认后立刻
        // 画该段包络，段与段之间用 SimulateFromStateFront 精确续接（携带挂车航向
        // startPhi2）。因此这里的拼接必须**完全复刻 Commands.TruckDrive 的调用序列**：
        // 早先这版用 Simulate(后轴中心, 航向, …) 拼段，而 Simulate 把挂车初始化成与
        // 车头同向，会在段界上让挂车瞬移 3.6m —— 那是校验脚本自己的假象，不是产品缺陷。
        //
        // 「有概率」= 特定角度下看不出问题。所以这里扫参数网格（车型 × 转角 × 直行长度），
        // 把成环失败的场景逼出来，而不是只跑一个幸运角度。
        static int RunConsecutiveSegments()
        {
            double[] turns = { 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0 };
            // 直行长度取到 60m：铰接车挂车收敛是指数式的（theta(s)=2·atan(tan(theta0/2)·e^(-s/L2))），
            // 直行越长越接近「整车精确平移」，正是并集边界最容易互相覆盖、成不了环的极端工况。
            double[] straights = { 0.0, 1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0 };
            int fail = 0, total = 0, fb = 0;
            foreach (bool articulated in new[] { true, false })
                foreach (double t in turns)
                    foreach (double s in straights)
                    {
                        total++;
                        if (TruckKinematics.EnvelopeFallbackUsed) fb++;
                        fail += RunConsecutiveOne(articulated, t, s);
                    }
            Console.WriteLine(string.Format("       连续段扫描：{0} 个组合，{1} 个失败", total, fail));
            return fail;
        }

        // ================= v4.7：光标反解「硬跳 90°」回归 =================
        // 用户反馈：TRUCKDRIVE 时「指针划到大概 90° 转弯的地方会自动吸附过去，很不舒服」。
        // 根因：TrySolveFromStateToTargetFrontAdaptive 在目标太近（所需半径 < 最小转弯半径）
        // 时返回 false，旧代码一律回退成「按最小半径硬转 90°」—— 与光标位置完全无关，
        // 光标一进这个区域预览就锁死在同一个 90° 姿态上，看着就是"吸附"。
        // v4.7 改为沿最小半径圆把车头尽量转向光标（TurnTowardTargetAtMinRadius）。
        // 这里验证三件事：
        //   1) 连续性：光标沿射线由远及近穿过可行性边界时，转角不跳变
        //      （边界处自适应解与回退解必须给出同一个角度）。
        //   2) 不锁死：回退分支的角度必须随光标变化，绝不能是常数 90°。
        //   3) 值域：0 ≤ 转角 ≤ 180°。
        static int RunMinRadiusContinuity()
        {
            var p = VehicleParams.Defaults();
            p.Articulated = true;
            var S = new Point2d(0.0, 0.0);          // 起点前轴
            var u = new Vector2d(1.0, 0.0);         // 车头朝 +X
            double Rmin = TruckKinematics.TurnRadius(p);

            int fail = 0, boundaryChecked = 0;
            double worstJump = 0.0;
            string worstWhere = "-";
            double fbMin = double.MaxValue, fbMax = double.MinValue;
            const double Step = 0.05;               // 射线采样步长(m)：越小，跨边界的采样差越小

            // 只扫左侧（steerSign=+1），右侧是镜像
            double[] betas = { 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 105.0, 120.0, 135.0, 150.0 };
            foreach (double betaDeg in betas)
            {
                double b = betaDeg * Math.PI / 180.0;
                var dir = new Vector2d(Math.Cos(b), Math.Sin(b));

                // 由远及近扫，可行性随 r 单调（Rrear=(r+2L·cosβ)/(2|sinβ|) 随 r 递增），
                // 因此只会出现一次「可行 → 不可行」的翻转。
                double rIn = double.NaN, rOut = double.NaN;   // rIn=最远的不可行点，rOut=最近的可行点
                double prevR = -1.0;
                bool prevOk = true;
                for (double r = 400.0; r >= 0.5; r -= Step)
                {
                    var T = S + r * dir;
                    bool ok = TruckKinematics.TrySolveFromStateToTargetFrontAdaptive(p, S, u, +1, T,
                                  out double deg, out double straight, out double rearR);
                    if (!ok && prevOk && prevR > 0.0) { rOut = prevR; rIn = r; }
                    if (!ok)
                    {
                        double fa = TruckKinematics.TurnTowardTargetAtMinRadius(p, S, u, +1, T);
                        if (fa < fbMin) fbMin = fa;
                        if (fa > fbMax) fbMax = fa;
                        if (fa < -1e-9 || fa > 180.0 + 1e-9)
                        {
                            Console.WriteLine("       [失败] 方向 {0}°：回退转角 {1:F3}° 越界（应在 0~180）", betaDeg, fa);
                            fail++;
                        }
                    }
                    prevR = r; prevOk = ok;
                }
                if (double.IsNaN(rIn)) continue;    // 该方向全程可行 → 无边界可测

                bool okOut = TruckKinematics.TrySolveFromStateToTargetFrontAdaptive(p, S, u, +1, S + rOut * dir,
                                 out double degOut, out double stOut, out double rrOut);
                double angIn = TruckKinematics.TurnTowardTargetAtMinRadius(p, S, u, +1, S + rIn * dir);
                if (!okOut) continue;

                boundaryChecked++;
                double jump = Math.Abs(degOut - angIn);
                if (jump > worstJump) { worstJump = jump; worstWhere = betaDeg.ToString("F0") + "°"; }
                // Step=0.05m、R≈18.5m ⇒ 跨边界的理论采样差量级 0.05/20 rad ≈ 0.15°，
                // 阈值取 1° 留足余量；旧版（固定 90°）在这里会直接跳到几十度。
                if (jump > 1.0)
                {
                    Console.WriteLine("       [失败] 方向 {0}°：跨可行性边界转角跳变 {1:F3}°（自适应 {2:F3}° / 回退 {3:F3}°）",
                        betaDeg, jump, degOut, angIn);
                    fail++;
                }
            }

            Console.WriteLine(string.Format("       边界连续性：检查 {0} 个方向，最大跳变 {1:F3}°（最差方向 {2}）",
                boundaryChecked, worstJump, worstWhere));

            // 回退角度必须有跨度：旧版写死 90°，fbMax-fbMin 会等于 0
            double span = (fbMax >= fbMin) ? (fbMax - fbMin) : 0.0;
            Console.WriteLine(string.Format("       回退转角范围：{0:F2}° ~ {1:F2}°（跨度 {2:F2}°）", fbMin, fbMax, span));
            if (span < 5.0)
            {
                Console.WriteLine("       [失败] 回退转角几乎不随光标变化（跨度 {0:F2}° < 5°），说明又被写死了", span);
                fail++;
            }
            return fail;
        }

        // v4.8 新增：量化「预览步长 1° → 0.5°」对包络锯齿的改善。
        //
        // 用户反馈「铰链车总出现折线包络而非曲线」。锯齿有两层成因：
        //   ① 横向齿距 = 采样步长 × 弧长 —— 加密采样可直接减小；
        //   ② 纵向齿深 ≈ 车体宽度 —— 多刚体 swept volume 的几何不连续点，
        //      加密采样消不掉，只能靠拟合压低。
        // 本测试把三种配置（v4.7 预览 / v4.8 预览 / 生产出图）放在同一批路径上对比，
        // 用数据确认 ① 确实被改善，并记录 ② 的残留量，避免「改完就说好了」。
        static int RunPreviewStepSmoothness()
        {
            var p = VehicleParams.Defaults();
            p.Articulated = true;   // 用户场景是铰接车

            // 代表性路径：小角度转弯是用户明确提到的场景，另加常规 90° 与 S 弯
            var cases = new (string label, double[] turns, double[] straights)[]
            {
                ("小角度 30°", new[] { 30.0 }, new[] { 0.0 }),
                ("小角度 30°+直5", new[] { 30.0 }, new[] { 5.0 }),
                ("常规 90°", new[] { 90.0 }, new[] { 0.0 }),
                ("S弯 左45+右45", new[] { 45.0, -45.0 }, new[] { 5.0, 5.0 }),
            };

            // 三档生产/预览配置：(标签, 步长, 容差)
            var configs = new (string label, double step, double tol)[]
            {
                ("v4.7预览 1°/8cm", 1.0, 8e-2),
                ("v4.8预览 0.5°/5cm", 0.5, 5e-2),
                ("生产出图 0.25°/3cm", 0.25, 3e-2),
            };

            Console.WriteLine("\n-- v4.8 新增：预览步长对包络锯齿的影响（铰接车）--");
            Console.WriteLine("   {0,-16} {1,10} {2,10} {3,10} {4,10} {5,8}",
                "配置", "原始顶点", "拟合顶点", "锯齿数", "最大折角", "");

            int fail = 0;
            foreach (var c in cases)
            {
                Console.WriteLine("  · " + c.label);
                foreach (var cfg in configs)
                {
                    var frames = BuildMultiSegPath(p, c.turns, c.straights, cfg.step);
                    var env = TruckKinematics.EnvelopeTracks(frames, p);
                    var centers = TruckKinematics.TurnCenters(frames, p);
                    List<Point2d> envPts; List<double> bulges;
                    // 与 TruckJigs.JigDraw.EnvelopeTracks / Commands.DrawSegmentTracks
                    // 同一个入口，只是步长与容差按配置换
                    GeometryUtil.FitEnvelopeBulges(env, centers, out envPts, out bulges,
                        cfg.tol, 4.0, 4, cfg.tol, cfg.tol);
                    var sampled = envPts == null ? null : SampleBulgePolyline(envPts, bulges, 24);
                    int zig = sampled == null ? 0 : CountZigzag(sampled);
                    double maxTurn = sampled == null ? 0 : MaxTurnAngleDeg(sampled);

                    Console.WriteLine("    {0,-18} {1,10} {2,10} {3,10} {4,9:F2}°",
                        cfg.label, env.Count,
                        envPts == null ? 0 : envPts.Count, zig, maxTurn);
                }
            }

            // ---- v4.8 关键诊断：分离「步长」与「容差」两个变量 ----
            // 上面的三档配置同时改了两个变量，无法归因。这里做对照实验：
            //   A 组：固定容差 8e-2，只动步长   → 看步长的净效应
            //   B 组：固定步长 1°，只动容差    → 看容差的净效应
            Console.WriteLine("\n  【变量分离】A 组：固定容差 8cm，只动步长");
            foreach (double st in new[] { 1.0, 0.5, 0.25 })
            {
                var sb = new System.Text.StringBuilder();
                sb.Append(string.Format("    步长 {0,5}°  ", st));
                foreach (var c in cases)
                {
                    var frames = BuildMultiSegPath(p, c.turns, c.straights, st);
                    var env = TruckKinematics.EnvelopeTracks(frames, p);
                    var centers = TruckKinematics.TurnCenters(frames, p);
                    List<Point2d> ep; List<double> bg;
                    GeometryUtil.FitEnvelopeBulges(env, centers, out ep, out bg, 8e-2, 4.0, 4, 8e-2, 8e-2);
                    var sp = ep == null ? null : SampleBulgePolyline(ep, bg, 24);
                    sb.Append(string.Format("| {0,-14} 齿{1,4} 拟合{2,4} 折角{3,6:F1}° ",
                        c.label, sp == null ? 0 : CountZigzag(sp), ep == null ? 0 : ep.Count,
                        sp == null ? 0 : MaxTurnAngleDeg(sp)));
                }
                Console.WriteLine(sb.ToString());
            }

            // v4.8 关键结论：容差越松，拟合越激进，齿数越少（B 组实测 8cm→5cm 齿数反而涨 3 倍）。
            // 因此往「放宽」方向扫描。但放宽有安全红线：包络必须覆盖所有车体角点，
            // 不能为了光滑而内缩。这里对每个容差同时检查 cornersOutside（必须 = 0）。
            Console.WriteLine("\n  【容差扫描】固定步长 1°，容差 8→20cm，含覆盖性检查");
            // 只取 2 条代表性路径：放宽容差后拟合迭代次数暴增，全量扫描会跑几分钟
            var scanCases = new[] { cases[0], cases[2] };
            foreach (double tl in new[] { 8e-2, 0.12, 0.16, 0.20 })
            {
                var sb = new System.Text.StringBuilder();
                sb.Append(string.Format("    容差 {0,4:F0}cm ", tl * 100));
                int outsideTotal = 0;
                foreach (var c in scanCases)
                {
                    var frames = BuildMultiSegPath(p, c.turns, c.straights, 1.0);
                    var env = TruckKinematics.EnvelopeTracks(frames, p);
                    var centers = TruckKinematics.TurnCenters(frames, p);
                    List<Point2d> ep; List<double> bg;
                    GeometryUtil.FitEnvelopeBulges(env, centers, out ep, out bg, tl, 4.0, 4, tl, tl);
                    var sp = ep == null ? null : SampleBulgePolyline(ep, bg, 24);
                    int outside = sp == null ? 0 : CountCornersOutside(frames, p, sp);
                    outsideTotal += outside;
                    sb.Append(string.Format("| {0,-14} 齿{1,4} 拟合{2,4} 折角{3,6:F1}° 露{4,3} ",
                        c.label, sp == null ? 0 : CountZigzag(sp), ep == null ? 0 : ep.Count,
                        sp == null ? 0 : MaxTurnAngleDeg(sp), outside));
                }
                sb.Append(outsideTotal == 0 ? "  [覆盖OK]" : "  *** 内缩 ***");
                Console.WriteLine(sb.ToString());
            }

            // ---- v4.8 决定性诊断：按「振幅」分级统计锯齿 ----
            // CountZigzag 默认 ampMin=3mm，数出来的几十个齿绝大多数在屏幕上根本看不见。
            // 用户截图里的齿深是车宽量级（~1m），必须按振幅分级才能定位真正刺眼的那部分。
            Console.WriteLine("\n  【振幅分级】步长 1° / 容差 8cm，统计不同振幅档的齿数");
            Console.WriteLine("    （振幅 = 齿的纵向深度；3mm 以下肉眼不可见，10cm 以上才明显）");
            double[] amps = { 3e-3, 1e-2, 3e-2, 0.10, 0.30, 1.0 };
            foreach (double amp in amps)
            {
                var sb = new System.Text.StringBuilder();
                sb.Append(string.Format("    振幅>{0,6}  ", amp >= 1.0 ? (amp * 100).ToString("F0") + "cm"
                    : amp >= 0.01 ? (amp * 100).ToString("F0") + "cm" : (amp * 1000).ToString("F0") + "mm"));
                foreach (var c in cases)
                {
                    var frames = BuildMultiSegPath(p, c.turns, c.straights, 1.0);
                    var env = TruckKinematics.EnvelopeTracks(frames, p);
                    var centers = TruckKinematics.TurnCenters(frames, p);
                    List<Point2d> ep; List<double> bg;
                    GeometryUtil.FitEnvelopeBulges(env, centers, out ep, out bg, 8e-2, 4.0, 4, 8e-2, 8e-2);
                    var sp = ep == null ? null : SampleBulgePolyline(ep, bg, 24);
                    sb.Append(string.Format("| {0,-14} {1,4}个 ",
                        c.label, sp == null ? 0 : CountZigzag(sp, 5, amp)));
                }
                Console.WriteLine(sb.ToString());
            }

            Console.WriteLine("  【变量分离】B 组：固定步长 1°，只动容差");
            foreach (double tl in new[] { 8e-2, 5e-2, 3e-2 })
            {
                var sb = new System.Text.StringBuilder();
                sb.Append(string.Format("    容差 {0,5:F0}cm  ", tl * 100));
                foreach (var c in cases)
                {
                    var frames = BuildMultiSegPath(p, c.turns, c.straights, 1.0);
                    var env = TruckKinematics.EnvelopeTracks(frames, p);
                    var centers = TruckKinematics.TurnCenters(frames, p);
                    List<Point2d> ep; List<double> bg;
                    GeometryUtil.FitEnvelopeBulges(env, centers, out ep, out bg, tl, 4.0, 4, tl, tl);
                    var sp = ep == null ? null : SampleBulgePolyline(ep, bg, 24);
                    sb.Append(string.Format("| {0,-14} 齿{1,4} 拟合{2,4} 折角{3,6:F1}° ",
                        c.label, sp == null ? 0 : CountZigzag(sp), ep == null ? 0 : ep.Count,
                        sp == null ? 0 : MaxTurnAngleDeg(sp)));
                }
                Console.WriteLine(sb.ToString());
            }

            Console.WriteLine("       预览步长对比：{0} 条路径，{1} 个未通过", cases.Length, fail);
            return fail;
        }

        // =====================================================================
        // v4.9 折角归因：包络上的「大折角」到底是哪个部件、哪一帧产生的？
        //
        // 背景：用户反馈「铰接车总是有概率出现折线的黄色包络而非曲线」。
        // v4.8 已用振幅分级证明调步长/容差无效（>30cm 锯齿恒为 0，>10cm 恒为 4 个），
        // 所以不能再靠调参，必须先回答「这几道折是谁制造的」：
        //   · 若是「底盘纵梁 ↔ 挂车」的固定接缝 → 属多刚体 swept volume 的几何不连续，
        //     只能靠局部倒角（方案 B）消掉；
        //   · 若是某个部件的扫掠段整体拟合失败 → 那是拟合算法问题，倒角治标不治本。
        //
        // 判据：拟合+采样后，顶点转向角 > 30° 即计为折角。
        //       圆弧按 24 段采样，单段转向角只有几度，不会误计。
        // =====================================================================
        /// <summary>
        /// v4.8 方案三：Spline 出图前的「稀化 + 外推补偿」必须满足三条不变量。
        ///   ① 顶点数断崖式下降 —— 台阶伪影被抹掉，插值样条才不会把锯齿变成波浪；
        ///   ② 简化环仍然外包原始环（车辆 footprint 角点全在环内）—— 只外扩、绝不内缩；
        ///      内缩意味着包络比实际扫掠区小，是安全事故；外扩只是保守。
        ///   ③ 外推量有界（远小于工程安全净距 0.3~0.5 m），且锯齿数显著下降。
        /// 任何一条不达标都不能出图。
        /// </summary>
        /// <summary>
        /// v4.9：倒退行驶的几何闭环验证（最硬的一条）。
        ///
        /// 不变量：反解 → 模拟之后，末帧前轴必须落在用户点击的目标点上。
        /// 这一条能同时抓住三类错误，任何一处写错末帧都会偏离目标甚至跑到镜像位置：
        ///   ① 半径公式 R=(D²+2La)/(2·d·steerSign) 的分母符号写反
        ///   ② 方向判据 dir·steerSign·θ≥0 漏乘 dir（倒退会走出前进的镜像轨迹）
        ///   ③ 转角 θ 的符号在 SimulateFromStateFront 里没跟着 dir 翻
        ///
        /// 同时验证「头一步的位移方向必须与 dir 一致」：整段净位移在大转角时可能绕回起点前方，
        /// 所以只看第一步 —— 那一步的方向就是行驶方向的定义。
        /// </summary>
        /// <summary>构造共线起始帧（与 SimulateTurn90AndStraighten 内部一致）</summary>
        static Frame MakeStartFrame(VehicleParams p, Point2d front, Vector2d heading)
        {
            var u = heading.GetNormal();
            double L = p.TractorWheelbase;
            var O1 = front - L * u;
            if (!p.Articulated)
                return new Frame { O1 = O1, U1 = u, K = O1 + L * u, U2 = u, O2 = O1 + L * u };
            var K = O1 + p.TractorRearToKingpin * u;
            var O2 = K - p.TrailerKingpinToRearAxle * u;
            return new Frame { O1 = O1, U1 = u, K = K, U2 = u, O2 = O2 };
        }

        /// <summary>倒退控制点（铰接=挂车后轴 O2，刚性=后轴 O1）</summary>
        static Point2d ReverseControlPoint(VehicleParams p, Frame f)
        {
            return p.Articulated ? f.O2 : f.O1;
        }

        /// <summary>
        /// v4.9 起：倒退行驶几何闭环（反解→模拟，末帧必须落在目标点）。
        /// v4.9.9：倒退改走「车尾控制」新管线（TrySolveReverseArc + SimulateReverseFromState），
        /// 闭环判据相应改为「末帧车尾控制点 == 目标点」；另加两条新断言：
        ///   ③ 全程铰接角 ≤ 机械限位（新模型的核心承诺：不再 jackknife）
        ///   ④ 头一步车尾位移必须朝「后」走（u2 反方向）
        /// 前进分支不变（TrySolveAutoSide + SimulateFromStateFront，末帧前轴 == 目标）。
        /// </summary>
        static int RunReverseGeometry()
        {
            Console.WriteLine("\n-- 倒退行驶几何闭环（v4.9.9 车尾控制：末帧车尾 == 目标点，铰接角不超限）--");
            int fail = 0;
            var cases = new (string name, bool arti)[] { ("刚性", false), ("铰接", true) };

            Console.WriteLine("  {0,-8} {1,-6} {2,10} {3,9} {4,10} {5,8} {6,10} {7}",
                              "车型", "方向", "可行/总", "闭环失败", "最大误差m", "方向错", "铰接超限", "结论");

            foreach (var c in cases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = c.arti;
                if (!c.arti) p.RigidRearOverhang = 2.0;

                foreach (int dir in new[] { 1, -1 })
                {
                    int total = 0, solvable = 0, closed = 0, wrongDir = 0, artViol = 0;
                    double maxErr = 0.0;
                    double maxArt = p.MaxArticulationAngleDeg;

                    // 目标点网格：沿航向 a（前进取正、倒退取负）× 横向 d（两侧都扫）
                    for (double a = 1.0; a <= 40.0; a += 3.0)
                        for (double d = -20.0; d <= 20.0; d += 2.5)
                        {
                            if (Math.Abs(d) < 1e-9) continue;      // 正前/正后走直线分支，不参与圆弧闭环
                            var startFront = new Point2d(0.0, 0.0);
                            var heading = new Vector2d(1.0, 0.0);
                            total++;

                            if (dir > 0)
                            {
                                var target = new Point2d(a, d);
                                if (!TruckKinematics.TrySolveAutoSide(p, startFront, heading, dir, target,
                                                                      out int steer, out double deg, out double str, out double R))
                                    continue;
                                solvable++;
                                var frames = TruckKinematics.SimulateFromStateFront(p, startFront, heading, 0.0,
                                                                                    steer, deg, str, dir,
                                                                                    TruckKinematics.FineStepDeg, R);
                                if (frames == null || frames.Count < 2) continue;
                                var endFront = TruckKinematics.FrontAxle(frames[frames.Count - 1], p);
                                double err = Math.Sqrt((endFront.X - target.X) * (endFront.X - target.X)
                                                     + (endFront.Y - target.Y) * (endFront.Y - target.Y));
                                if (err > maxErr) maxErr = err;
                                if (err <= 0.02) closed++;
                                var f0 = TruckKinematics.FrontAxle(frames[0], p);
                                var f1 = TruckKinematics.FrontAxle(frames[1], p);
                                double along = (f1.X - f0.X) * heading.X + (f1.Y - f0.Y) * heading.Y;
                                if (dir * along <= 0) wrongDir++;
                            }
                            else
                            {
                                // v4.9.9 车尾控制：目标点定义在「车尾控制点」坐标系里
                                var f0frame = MakeStartFrame(p, startFront, heading);
                                var ctrl0 = ReverseControlPoint(p, f0frame);
                                var u2 = f0frame.U2.GetNormal();
                                var n2 = new Vector2d(-u2.Y, u2.X);
                                // 车尾后方 a 米、横向 d 米
                                var target = ctrl0 - a * u2 + d * n2;

                                if (!TruckKinematics.TrySolveReverseArc(p, f0frame, target,
                                                                        out int rside, out double deg, out double R2, out double str))
                                    continue;
                                solvable++;
                                var frames = TruckKinematics.SimulateReverseFromState(p, f0frame, rside, deg, str, R2,
                                                                                      TruckKinematics.FineStepDeg);
                                if (frames == null || frames.Count < 2) continue;

                                // ① 闭环：末帧车尾控制点 == 目标点
                                var endCtrl = ReverseControlPoint(p, frames[frames.Count - 1]);
                                double err = Math.Sqrt((endCtrl.X - target.X) * (endCtrl.X - target.X)
                                                     + (endCtrl.Y - target.Y) * (endCtrl.Y - target.Y));
                                if (err > maxErr) maxErr = err;
                                if (err <= 0.02) closed++;

                                // ② 方向：头一步车尾必须朝后走（−u2 方向）
                                var c0 = ReverseControlPoint(p, frames[0]);
                                var c1 = ReverseControlPoint(p, frames[1]);
                                double back = (c1.X - c0.X) * u2.X + (c1.Y - c0.Y) * u2.Y;
                                if (back >= 0) wrongDir++;

                                // ③ 全程铰接角 ≤ 机械限位 + 0.01°（数值余量）
                                if (p.Articulated)
                                {
                                    foreach (var fr in frames)
                                    {
                                        double a1 = Math.Atan2(fr.U1.Y, fr.U1.X);
                                        double a2 = Math.Atan2(fr.U2.Y, fr.U2.X);
                                        double art = Math.Abs(NormalizeAngleDeg((a2 - a1) * 180.0 / Math.PI));
                                        if (art > maxArt + 0.01) { artViol++; break; }
                                    }
                                }
                            }
                        }

                    bool ok = wrongDir == 0 && artViol == 0 && closed == solvable && solvable > 0;
                    if (!ok) fail++;
                    Console.WriteLine("  {0,-8} {1,-6} {2,10} {3,9} {4,10:F6} {5,8} {6,10} {7}",
                                      c.name, dir > 0 ? "前进" : "倒退",
                                      solvable + "/" + total, solvable - closed, maxErr, wrongDir, artViol,
                                      ok ? "OK" : "★失败");
                }
            }
            return fail;
        }

        /// <summary>
        /// v4.9.9：倒退新模型（挂车牵引 + 司机修正）的两条核心承诺：
        ///   ① 不再 jackknife：带初始铰接角倒退直行 60m，铰接角必须收敛（司机拉正），
        ///      而不是旧模型的指数发散撞限位。
        ///   ② 稳态铰接角公式 θ_eq = −atan2(s·(R1·L2−ak·R2), R1·R2+ak·L2) 必须与
        ///      前进恒转角模拟的稳态值一致（同一套同心圆几何，与行进方向无关）。
        /// </summary>
        static int RunReverseJackknife()
        {
            Console.WriteLine("\n-- v4.9.9：倒退铰接角收敛（司机修正，不 jackknife）+ 稳态公式校验 --");
            int fail = 0;
            var p = VehicleParams.Defaults();
            p.Articulated = true;

            var startFront = new Point2d(0.0, 0.0);
            var heading = new Vector2d(1.0, 0.0);

            // ---- ① 带初始铰接角倒退直行 60m：必须收敛 ----
            var warm = TruckKinematics.SimulateFromStateFront(p, startFront, heading, 0.0, +1, 30.0, 0.0, 1,
                                                              TruckKinematics.FineStepDeg);
            if (warm == null || warm.Count < 2) { Console.WriteLine("  预热段生成失败 ★"); return 1; }
            var s0 = warm[warm.Count - 1];
            double phi1_0 = Math.Atan2(s0.U1.Y, s0.U1.X);
            double phi2_0 = Math.Atan2(s0.U2.Y, s0.U2.X);
            double art0 = Math.Abs(NormalizeAngleDeg((phi2_0 - phi1_0) * 180.0 / Math.PI));

            var fRev = TruckKinematics.SimulateReverseFromState(p, s0, +1, 0.0, 60.0,
                                                                double.PositiveInfinity, TruckKinematics.FineStepDeg);
            if (fRev == null || fRev.Count < 2) { Console.WriteLine("  倒退直行生成失败 ★"); return 1; }
            var e = fRev[fRev.Count - 1];
            double artRev = Math.Abs(NormalizeAngleDeg(
                (Math.Atan2(e.U2.Y, e.U2.X) - Math.Atan2(e.U1.Y, e.U1.X)) * 180.0 / Math.PI));
            // 全程不超限
            double maxSeen = 0.0;
            foreach (var fr in fRev)
            {
                double art = Math.Abs(NormalizeAngleDeg(
                    (Math.Atan2(fr.U2.Y, fr.U2.X) - Math.Atan2(fr.U1.Y, fr.U1.X)) * 180.0 / Math.PI));
                if (art > maxSeen) maxSeen = art;
            }
            bool conv = artRev < art0 * 0.5 && maxSeen <= p.MaxArticulationAngleDeg + 0.01;
            if (!conv) fail++;
            Console.WriteLine("  初始铰接角 {0:F2}° → 倒退直行60m后 {1:F2}°（全程最大 {2:F2}°，限位 {3:F0}°）  {4}",
                              art0, artRev, maxSeen, p.MaxArticulationAngleDeg, conv ? "OK（收敛不折叠）" : "★失败");

            // ---- ② 稳态铰接角公式 vs 前进恒转角模拟 ----
            // 前进以最小半径恒转角转 3 圈，铰接角收敛到稳态 θ_fwd；
            // 公式 θ_eq = −atan2(s·(R1·L2+ak·R2), R1·R2+ak·L2)，R2 = sqrt(R1²+ak²−L2²)。
            double L2 = p.TrailerKingpinToRearAxle, ak = p.TractorRearToKingpin;
            double R1 = TruckKinematics.TurnRadius(p);
            double R2f = Math.Sqrt(Math.Max(1e-6, R1 * R1 + ak * ak - L2 * L2));
            foreach (int s in new[] { 1, -1 })
            {
                var fCw = TruckKinematics.SimulateFromStateFront(p, startFront, heading, 0.0, s, 1080.0, 0.0, 1,
                                                                 TruckKinematics.FineStepDeg);
                if (fCw == null || fCw.Count < 2) { fail++; continue; }
                var ef = fCw[fCw.Count - 1];
                double thetaFwd = NormalizeAngleDeg(
                    (Math.Atan2(ef.U2.Y, ef.U2.X) - Math.Atan2(ef.U1.Y, ef.U1.X)) * 180.0 / Math.PI);
                // 前进稳态公式（同一几何，s 为转向侧）：θ_fwd = −atan2(s·(R1·L2−ak·R2), R1·R2+ak·L2)。
                // 与 SimulateFromStateFront 的 ODE 稳态条件 φ1dot·(L2−ak·cosθ)=−s·sinθ 等价。
                double thetaFormula = Math.Atan2(R1 * L2 - ak * R2f, R1 * R2f + ak * L2) * 180.0 / Math.PI;
                double diff = Math.Abs(Math.Abs(thetaFwd) - thetaFormula);
                bool okF = diff < 1.0;
                if (!okF) fail++;
                Console.WriteLine("  前进稳态（{0}转）：模拟 {1:F2}° vs 公式 {2:F2}°，差 {3:F3}°  {4}",
                                  s > 0 ? "左" : "右", thetaFwd, thetaFormula, diff, okF ? "OK" : "★失败");
            }
            return fail;
        }

        /// <summary>
        /// v4.9：倒退路径的包络必须满足与前进完全相同的三条不变量。
        /// 包络引擎是按「帧序列的 footprint 并集」算的，理论上与行驶方向无关，
        /// 但倒退时挂车会折叠，footprint 形状分布不同，值得单独跑一遍。
        /// </summary>
        /// <summary>
        /// v4.9：倒退路径的可视化。倒车的转向手感与前进相反、铰接车还会折叠，
        /// 光看数字不足以确认方向对不对 —— 必须出图肉眼核对。
        /// 黄=扫掠包络，绿=起始车体，青=结束车体，灰=中间姿态，洋红=前轴轨迹。
        /// </summary>
        static void WriteSvgReverse(string label, List<Frame> frames, VehicleParams p, int dir)
        {
            if (frames == null || frames.Count < 2) return;

            var env = TruckKinematics.EnvelopeTracks(frames, p);
            var pts = new List<Point2d>();
            if (env != null) pts.AddRange(env);
            foreach (var f in frames)
            {
                pts.AddRange(TruckKinematics.TractorRect(f, p));
                if (p.Articulated) pts.AddRange(TruckKinematics.TrailerRect(f, p));
            }

            double minX = pts.Min(q => q.X), maxX = pts.Max(q => q.X);
            double minY = pts.Min(q => q.Y), maxY = pts.Max(q => q.Y);
            double pad = 2.0;
            minX -= pad; maxX += pad; minY -= pad; maxY += pad;
            double w = maxX - minX, h = maxY - minY;
            double scale = 820.0 / Math.Max(w, h);

            Func<double, double> Sx = x => (x - minX) * scale + 40.0;
            Func<double, double> Sy = y => (maxY - y) * scale + 40.0;
            Func<List<Point2d>, string> Poly = qs => string.Join(" ", qs.Select(q =>
                string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));

            var sb = new StringBuilder();
            int W = (int)(w * scale) + 80, H = (int)(h * scale) + 80;
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<svg xmlns='http://www.w3.org/2000/svg' width='{0}' height='{1}' viewBox='0 0 {0} {1}'>", W, H);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture, "<rect width='{0}' height='{1}' fill='white'/>", W, H);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<text x='10' y='22' font-size='15' fill='#333'>{0}（{1}）  共 {2} 帧</text>",
                label, dir > 0 ? "前进" : "倒退", frames.Count);
            sb.AppendLine();

            // 扫掠包络（黄，半透明填充）
            if (env != null && env.Count >= 3)
            {
                sb.AppendFormat("<polygon points='{0}' fill='rgba(255,193,7,0.15)' stroke='#D6A800' stroke-width='2.0'/>", Poly(env));
                sb.AppendLine();
            }

            // 前轴轨迹（洋红）
            var frontPath = frames.Select(f => TruckKinematics.FrontAxle(f, p)).ToList();
            sb.AppendFormat("<polyline points='{0}' fill='none' stroke='#C026D3' stroke-width='1.6'/>", Poly(frontPath));
            sb.AppendLine();

            // 挂车后轴轨迹（蓝）—— 倒退时它与前轴轨迹分离得最明显，是 off-tracking 的直观体现
            if (p.Articulated)
            {
                var o2Path = frames.Select(f => f.O2).ToList();
                sb.AppendFormat("<polyline points='{0}' fill='none' stroke='#1565C0' stroke-width='1.6'/>", Poly(o2Path));
                sb.AppendLine();
            }

            // 中间姿态（灰细）
            int step = Math.Max(1, frames.Count / 12);
            for (int i = step; i < frames.Count - 1; i += step)
            {
                var f = frames[i];
                sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#BBBBBB' stroke-width='0.9'/>",
                                Poly(new List<Point2d>(TruckKinematics.TractorRect(f, p))));
                sb.AppendLine();
                if (p.Articulated)
                {
                    sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#BBBBBB' stroke-width='0.9'/>",
                                    Poly(new List<Point2d>(TruckKinematics.TrailerRect(f, p))));
                    sb.AppendLine();
                }
            }

            // 首帧（绿）与末帧（青）
            var f0 = frames[0];
            var fE = frames[frames.Count - 1];
            sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#2E7D32' stroke-width='2.6'/>",
                            Poly(new List<Point2d>(TruckKinematics.TractorRect(f0, p))));
            sb.AppendLine();
            if (p.Articulated)
            {
                sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#2E7D32' stroke-width='2.6'/>",
                                Poly(new List<Point2d>(TruckKinematics.TrailerRect(f0, p))));
                sb.AppendLine();
            }
            sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#00838F' stroke-width='2.6'/>",
                            Poly(new List<Point2d>(TruckKinematics.TractorRect(fE, p))));
            sb.AppendLine();
            if (p.Articulated)
            {
                sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#00838F' stroke-width='2.6'/>",
                                Poly(new List<Point2d>(TruckKinematics.TrailerRect(fE, p))));
                sb.AppendLine();
            }

            sb.AppendLine("</svg>");
            string name = string.Format(CultureInfo.InvariantCulture, "VerifyReverse_{0}.svg", label);
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, name);
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        /// <summary>
        /// v4.9.18：刚性车近直行（自适应反解 R 巨大 / +∞）时，刚性私有模拟变体
        /// 漏了铰接变体的 ds≤0.5m 钳制与 R≥1e9 转弯跳过 —— 20m 直行只出 2 帧，
        /// 包络被两个不重叠车体矩形串成细缝（实机「直行时黄色包络消失」）。
        /// 不变量：帧数 ≥ 距离/0.5m 的 80%；无 NaN 帧；包络角点在外=0、边界点在内=0、不回退。
        /// </summary>
        static int RunRigidNearStraightEnvelope()
        {
            Console.WriteLine("\n-- v4.9.18 新增：刚性近直行（大R/∞）包络不变量 --");
            int fail = 0;
            var p = VehicleParams.Defaults();
            p.Articulated = false;

            Console.WriteLine("  {0,-18} {1,6} {2,8} {3,10} {4,10} {5,6} {6}",
                              "横向偏移m", "帧数", "NaN帧", "角点在外", "边界点在内", "回退", "结论");
            foreach (double lat in new[] { 0.0, 0.001, 0.01, 0.05, 0.2, 1.0 })
            {
                var f0 = MakeStartFrame(p, new Point2d(0, 0), new Vector2d(1, 0));
                var front0 = TruckKinematics.FrontAxle(f0, p);
                double phi2 = Math.Atan2(f0.U2.Y, f0.U2.X);
                var target = new Point2d(front0.X + 20.0, front0.Y + lat);
                bool ok0 = TruckKinematics.TrySolveAutoSide(p, front0, f0.U1, 1, target,
                    out int steer, out double turnDeg, out double straight, out double rOv);
                List<Frame> frames;
                if (ok0)
                    frames = TruckKinematics.SimulateFromStateFront(p, front0, f0.U1, phi2,
                        steer, turnDeg, straight, 1, TruckKinematics.FineStepDeg, rOv);
                else
                {
                    double td = TruckKinematics.TurnTowardTargetAtMinRadius(p, front0, f0.U1, steer, target, 1);
                    frames = TruckKinematics.SimulateFromStateFront(p, front0, f0.U1, phi2,
                        steer, td, 0.0, 1, TruckKinematics.FineStepDeg, null);
                }

                int nanN = 0;
                foreach (var fr in frames)
                    if (double.IsNaN(fr.O1.X) || double.IsNaN(fr.O1.Y) ||
                        double.IsNaN(fr.O2.X) || double.IsNaN(fr.O2.Y)) nanN++;

                var env = TruckKinematics.EnvelopeTracks(frames, p);
                int outside = CountCornersOutside(frames, p, env);
                int inside = CountBoundaryPtsStrictlyInside(frames, p, env);
                bool fb = TruckKinematics.EnvelopeFallbackUsed;

                // 20m 行程、0.5m 步长 ⇒ ≥32 帧（40 的 80%）
                bool ok = frames.Count >= 32 && nanN == 0 && outside == 0 && inside == 0 && !fb;
                if (!ok) fail++;
                Console.WriteLine("  {0,-18:F3} {1,6} {2,8} {3,10} {4,10} {5,6} {6}",
                                  lat, frames.Count, nanN, outside, inside, fb ? "是" : "否",
                                  ok ? "OK" : "★失败");
            }
            return fail;
        }

        static int RunReverseEnvelope()
        {
            Console.WriteLine("\n-- v4.9 新增：倒退路径的包络不变量 --");
            int fail = 0;
            var cases = new (string name, bool arti, double deg)[] {
                ("刚性-倒退60°", false, 60.0),
                ("铰接-倒退60°", true, 60.0),
                ("铰接-倒退90°", true, 90.0),
            };

            Console.WriteLine("  {0,-14} {1,6} {2,10} {3,10} {4,8} {5}",
                              "场景", "帧数", "角点在外", "边界点在内", "回退", "结论");

            foreach (var c in cases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = c.arti;
                if (!c.arti) p.RigidRearOverhang = 2.0;

                // v4.9.9：倒退改走「车尾控制」新管线。半径取「稳态铰接角 ≤ 60% 限位」的平缓值
                //（与 SimulateTurn90AndStraighten 倒退分支同一规则），刚性车取后轴最小半径。
                var f0 = MakeStartFrame(p, new Point2d(0, 0), new Vector2d(1, 0));
                double R2 = TruckKinematics.ReverseMinControlRadius(p);
                if (c.arti)
                {
                    double ts = p.MaxArticulationAngleDeg * 0.6 * Math.PI / 180.0;
                    double gentle = p.TrailerKingpinToRearAxle / Math.Tan(ts);
                    if (gentle > R2) R2 = gentle;
                }
                if (R2 < 0.3) R2 = 0.3;
                var frames = TruckKinematics.SimulateReverseFromState(p, f0, +1, c.deg, 0.0, R2,
                                                                      TruckKinematics.FineStepDeg);
                if (frames == null || frames.Count < 2) { fail++; Console.WriteLine("  {0,-14} 路径为空 ★", c.name); continue; }

                var env = TruckKinematics.EnvelopeTracks(frames, p);
                int outside = CountCornersOutside(frames, p, env);
                int inside = CountBoundaryPtsStrictlyInside(frames, p, env);
                bool fb = TruckKinematics.EnvelopeFallbackUsed;

                bool ok = outside == 0 && inside == 0 && !fb;
                if (!ok) fail++;
                Console.WriteLine("  {0,-14} {1,6} {2,10} {3,10} {4,8} {5}",
                                  c.name, frames.Count, outside, inside, fb ? "是" : "否", ok ? "OK" : "★失败");
            }

            // 出图供肉眼核对：倒退 90°（铰接车挂车折叠）与前进 90°（挂车拉直）的正反对照
            foreach (var arti in new[] { true, false })
            {
                var pv = VehicleParams.Defaults();
                pv.Articulated = arti;
                if (!arti) pv.RigidRearOverhang = 2.0;
                foreach (int d in new[] { -1, 1 })
                {
                    var fv = TruckKinematics.SimulateTurn90AndStraighten(pv, new Point2d(0, 0),
                                                                         new Vector2d(1, 0), +1,
                                                                         TruckKinematics.FineStepDeg, d);
                    if (fv != null && fv.Count > 1)
                        WriteSvgReverse(string.Format("{0}-{1}90", arti ? "铰接" : "刚性", d > 0 ? "前进" : "倒退"),
                                        fv, pv, d);
                }
            }
            Console.WriteLine("  （对比图已写入 VerifyReverse_*.svg）");
            return fail;
        }

        /// <summary>
        /// v4.9 BUG 复现：模拟 TRUCKDRIVE 的「直行 + 90°转弯」两段拼接，dir=+1/-1 各跑一次。
        /// 检查：① 段间衔接是否连续（无 stride gap）；② 包络是否正确包围两段；
        /// ③ 倒退模式下两段是否都沿正确方向行驶。输出 SVG 供肉眼核对。
        /// </summary>
        static int RunStraightTurnSvg()
        {
            Console.WriteLine("\n-- v4.9 复现：直行+90°（dir=+1/-1），输出 SVG 肉眼核对 --");
            int fail = 0;
            var cases = new (string name, bool arti)[] { ("刚性", false), ("铰接", true) };
            foreach (var c in cases)
            {
                foreach (int dir in new[] { 1, -1 })
                {
                    var p = VehicleParams.Defaults();
                    p.Articulated = c.arti;
                    if (!c.arti) p.RigidRearOverhang = 2.0;

                    // 模拟 TRUCKDRIVE 两段拼接（与 RunConsecutiveOne 完全一致）：
                    // 段 1: 纯直行 5 m（dir 决定正/负方向）
                    // 段 2: 90° 转弯（方向 = 自动判向；dir 决定沿 +x 还是 -x 绕）
                    // v4.9.9：倒退两段都改走「车尾控制」新管线。
                    var frames = new List<Frame>();
                    var f0 = new Point2d(0, 0);
                    var u0 = new Vector2d(1, 0);
                    List<Frame> seg1, seg2;
                    if (dir > 0)
                    {
                        seg1 = TruckKinematics.SimulateFromStateFront(p, f0, u0, 0, +1, 0, 5.0, dir, TruckKinematics.FineStepDeg);
                        frames.AddRange(seg1);
                        var lastF = seg1[seg1.Count - 1];
                        Console.WriteLine("  {0} {1:+#;-#;0}：seg1={2} 帧 直行5m末前轴=({3:F3},{4:F3})",
                                          c.name, dir, seg1.Count, TruckKinematics.FrontAxle(lastF, p).X, TruckKinematics.FrontAxle(lastF, p).Y);
                        Console.Write("    seg1 前轴轨迹：");
                        for (int k = 0; k < Math.Min(seg1.Count, 12); k++)
                            Console.Write("({0:F2},{1:F2}) ", TruckKinematics.FrontAxle(seg1[k], p).X, TruckKinematics.FrontAxle(seg1[k], p).Y);
                        Console.WriteLine();
                        seg2 = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(lastF, p), lastF.U1,
                            Math.Atan2(lastF.U2.Y, lastF.U2.X), +1, 90, 0, dir, TruckKinematics.FineStepDeg);
                    }
                    else
                    {
                        var fs0 = MakeStartFrame(p, f0, u0);
                        seg1 = TruckKinematics.SimulateReverseFromState(p, fs0, +1, 0, 5.0,
                                                                        double.PositiveInfinity, TruckKinematics.FineStepDeg);
                        frames.AddRange(seg1);
                        var lastF = seg1[seg1.Count - 1];
                        var ctrl1 = ReverseControlPoint(p, lastF);
                        Console.WriteLine("  {0} {1:+#;-#;0}：seg1={2} 帧 倒退5m末车尾=({3:F3},{4:F3})",
                                          c.name, dir, seg1.Count, ctrl1.X, ctrl1.Y);
                        // 段 2：车尾甩 90°，平缓半径（同 SimulateTurn90AndStraighten 倒退分支）
                        double R2 = TruckKinematics.ReverseMinControlRadius(p);
                        if (c.arti)
                        {
                            double ts = p.MaxArticulationAngleDeg * 0.6 * Math.PI / 180.0;
                            double gentle = p.TrailerKingpinToRearAxle / Math.Tan(ts);
                            if (gentle > R2) R2 = gentle;
                        }
                        if (R2 < 0.3) R2 = 0.3;
                        seg2 = TruckKinematics.SimulateReverseFromState(p, lastF, +1, 90, 0, R2, TruckKinematics.FineStepDeg);
                    }
                    for (int i = 1; i < seg2.Count; i++) frames.Add(seg2[i]);

                    // 段衔接检查：frames[seg1.Count] 应该是 seg2[1]（与 seg1 末帧连续）
                    var joinIdx = seg1.Count;            // seg2[1] 在合并后的下标
                    if (joinIdx < frames.Count)
                    {
                        var j0 = seg1[seg1.Count - 1];
                        var j1 = seg2[0];
                        double jump;
                        if (dir > 0)
                        {
                            var f0f = TruckKinematics.FrontAxle(j0, p);
                            var f1f = TruckKinematics.FrontAxle(j1, p);
                            jump = Math.Sqrt((f0f.X - f1f.X) * (f0f.X - f1f.X)
                                           + (f0f.Y - f1f.Y) * (f0f.Y - f1f.Y));
                        }
                        else
                        {
                            // v4.9.9：倒退段衔接比较「全姿态」（控制点 + 两个航向角）
                            var cA = ReverseControlPoint(p, j0);
                            var cB = ReverseControlPoint(p, j1);
                            jump = Math.Sqrt((cA.X - cB.X) * (cA.X - cB.X) + (cA.Y - cB.Y) * (cA.Y - cB.Y));
                            double dAng = Math.Abs(NormalizeAngleDeg(
                                (Math.Atan2(j1.U1.Y, j1.U1.X) - Math.Atan2(j0.U1.Y, j0.U1.X)) * 180.0 / Math.PI));
                            double dAng2 = Math.Abs(NormalizeAngleDeg(
                                (Math.Atan2(j1.U2.Y, j1.U2.X) - Math.Atan2(j0.U2.Y, j0.U2.X)) * 180.0 / Math.PI));
                            if (dAng > 0.5 || dAng2 > 0.5) jump = Math.Max(jump, Math.Max(dAng, dAng2) * 0.1);
                        }
                        Console.WriteLine("  {0} {1:+#;-#;0}：共 {2} 帧 段衔接跳变 {3:F4} m{4}",
                                          c.name, dir, frames.Count, jump,
                                          jump > 0.01 ? " ★过大" : "");
                        if (jump > 0.01) fail++;
                    }

                    WriteSvgReverse(string.Format("{0}-{1}-直5+转90", c.name, dir > 0 ? "前进" : "倒退"),
                                          frames, p, dir);

                    // 复现用户场景：直行 4.6 m + 小角度 13.4° 转弯（看是 v4.9 哪里出了平移错位）
                    var frames2 = new List<Frame>();
                    if (dir > 0)
                    {
                        var seg1b = TruckKinematics.SimulateFromStateFront(p, f0, u0, 0, +1, 0, 4.6, dir, TruckKinematics.FineStepDeg);
                        frames2.AddRange(seg1b);
                        var last1b = seg1b[seg1b.Count - 1];
                        var seg2b = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(last1b, p), last1b.U1,
                            Math.Atan2(last1b.U2.Y, last1b.U2.X), +1, 13.4, 0, dir, TruckKinematics.FineStepDeg);
                        for (int i = 1; i < seg2b.Count; i++) frames2.Add(seg2b[i]);
                    }
                    else
                    {
                        var fs0b = MakeStartFrame(p, f0, u0);
                        var seg1b = TruckKinematics.SimulateReverseFromState(p, fs0b, +1, 0, 4.6,
                                                                             double.PositiveInfinity, TruckKinematics.FineStepDeg);
                        frames2.AddRange(seg1b);
                        var last1b = seg1b[seg1b.Count - 1];
                        double R2b = TruckKinematics.ReverseMinControlRadius(p);
                        if (c.arti)
                        {
                            double ts = p.MaxArticulationAngleDeg * 0.6 * Math.PI / 180.0;
                            double gentle = p.TrailerKingpinToRearAxle / Math.Tan(ts);
                            if (gentle > R2b) R2b = gentle;
                        }
                        if (R2b < 0.3) R2b = 0.3;
                        var seg2b = TruckKinematics.SimulateReverseFromState(p, last1b, +1, 13.4, 0, R2b, TruckKinematics.FineStepDeg);
                        for (int i = 1; i < seg2b.Count; i++) frames2.Add(seg2b[i]);
                    }
                    WriteSvgReverse(string.Format("{0}-{1}-直4.6+转13", c.name, dir > 0 ? "前进" : "倒退"),
                                          frames2, p, dir);
                }
            }
            Console.WriteLine("  （图已写入 VerifyReverse_*-直5+转90*.svg）");
            return fail;
        }

        static int RunSplineSimplify()
        {
const double tol = 0.002;          // 必须与 Commands.EnvelopeSimplifyTol 保持一致
        const int smoothIters = 60;        // 必须与 Commands.EnvelopeSmoothIters 保持一致 (v4.9.2: 20→60)
        const double outsetLimit = 0.10;   // 外推上限：远小于 0.3 m 工程安全净距
            int fail = 0;

            Console.WriteLine("  {0,-14} {1,6} {2,6} {3,8} {4,9} {5,12} {6}",
                              "场景", "原始点", "简化点", "外扩m", "面积增", "锯齿", "判定");

            var cases = new[]
            {
                new { Name = "刚性90°",      Art = false, Turn =  90.0, Straight =  0.0 },
                new { Name = "刚性90°+拉直", Art = false, Turn =  90.0, Straight = 25.0 },
                new { Name = "铰接90°",      Art = true,  Turn =  90.0, Straight =  0.0 },
                new { Name = "铰接90°+拉直", Art = true,  Turn =  90.0, Straight = 25.0 },
                new { Name = "铰接45°",      Art = true,  Turn =  45.0, Straight =  0.0 },
                new { Name = "铰接右转90°",  Art = true,  Turn = -90.0, Straight =  0.0 },
            };

            foreach (var c in cases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = c.Art;
                if (!c.Art) p.RigidRearOverhang = 2.0;

                var startFront = new Point2d(0.0, 0.0);
                var heading = new Vector2d(1.0, 0.0);
                var frames = TruckKinematics.SimulateFromFrontAxle(
                    p, startFront, heading, c.Turn > 0 ? +1 : -1, Math.Abs(c.Turn), c.Straight,
                    TruckKinematics.FineStepDeg);
                var env = TruckKinematics.EnvelopeTracks(frames, p);

                if (env == null || env.Count < 3 || TruckKinematics.EnvelopeFallbackUsed)
                {
                    Console.WriteLine("  {0,-14} 包络生成失败（顶点 {1}，fallback={2}）",
                                      c.Name, env == null ? 0 : env.Count,
                                      TruckKinematics.EnvelopeFallbackUsed);
                    fail++; continue;
                }

                // --- 分步诊断：平滑 / 稀化 / 外推 各自对面积的影响 ---
                var smoothed = GeometryUtil.SmoothEnvelopeLoop(env, iters: smoothIters);
                var simp0 = GeometryUtil.SimplifyClosedLoop(smoothed, tol);
                double area0 = Math.Abs(PolygonArea(env));
                double areaSmooth = Math.Abs(PolygonArea(smoothed));
                double areaSimp0 = Math.Abs(PolygonArea(simp0));
                var res = GeometryUtil.SimplifyEnvelopeForSpline(env, tol, smoothIters: smoothIters);
                var simp = res.loop;
                double outset = res.outset;
                double area1 = Math.Abs(PolygonArea(simp));

                Console.WriteLine("      ↳ 面积 {0:F3} → 平滑 {1:F3} → 稀化 {2:F3} → 外推 {3:F3} m²；点 {4}→{5}→{6}→{7}",
                                  area0, areaSmooth, areaSimp0, area1,
                                  env.Count, smoothed.Count, simp0.Count, simp.Count);

                // ① 顶点数下降（残留 > 一半说明台阶没被抹掉，样条只会把尖角变波浪）
                bool fewer = simp.Count >= 3 && simp.Count <= env.Count * 0.5;

                // ② 外包：原始环每个顶点都必须在简化环内或环上。
                //    注意射线法对「恰好落在边界上」的点判定不稳定（可能判成外部），
                //    所以距离为 0 的一律算在环上，不算漏出。
                int outside = 0; double worst = 0;
                foreach (var q in env)
                {
                    double dq = DistanceToPolyline(q, simp);
                    if (dq < 1e-9) continue;
                    if (GeometryUtil.PointInClosedPolygon(q, simp)) continue;
                    outside++;
                    if (dq > worst) worst = dq;
                }

                // ③ 面积不得缩小 + 外推有界
                double areaGain = area0 > 1e-9 ? (area1 - area0) / area0 : 0.0;

                // 锯齿数（CountZigzag 内部已做等弧长重采样，直接传原始点）
                int zig0 = CountZigzag(env);
                int zig1 = CountZigzag(simp);
                // 振幅分级：3mm 以下肉眼不可见，1cm 以上才看得出，3cm 以上工程上明显
                int z3mm = CountZigzag(simp, 5, 3e-3);
                int z1cm = CountZigzag(simp, 5, 1e-2);
                int z3cm = CountZigzag(simp, 5, 3e-2);
                // 最大转角：平滑生效的话应回落到真实折角量级（90°+铰接角），
                // 而不是 bulge 伪尖刺的 128°~152°
                double maxTurn1 = MaxTurnAngleDeg(simp);

                bool ok = fewer && outside == 0 && outset <= outsetLimit
                          && areaGain >= -1e-9 && zig1 <= zig0;
                if (!ok) fail++;

                Console.WriteLine("  {0,-14} {1,6} {2,6} {3,8:F4} {4,8:F2}% {5,6:F1}° {6}",
                                  c.Name, env.Count, simp.Count, outset, areaGain * 100.0,
                                  maxTurn1, ok ? "OK" : "★失败");
                Console.WriteLine("      ↳ 锯齿 {0}→{1}（>3mm {2} 个，>1cm {3} 个，>3cm {4} 个）",
                                  zig0, zig1, z3mm, z1cm, z3cm);

                WriteSvgSpline(c.Name, env, simp);
                if (outside > 0)
                    Console.WriteLine("      ↳ {0} 个原始顶点露在简化环外，最大 {1:F4} m", outside, worst);
                if (outset > outsetLimit)
                    Console.WriteLine("      ↳ 外推 {0:F4} m 超过上限 {1:F4} m", outset, outsetLimit);
            }
            return fail;
        }

        /// <summary>
        /// v4.8 方案三调参：扫描「平滑迭代次数 × RDP 容差」。
        /// 权衡三条：外扩量（越小越好，但要 ≥0 才不内缩）、残余齿数（越少越好）、
        /// 拟合点数（越少样条越流畅，但形状精度下降）。
        /// 单跑：dotnet TruckTurn.Verify.dll sweep
        /// </summary>
        static void RunSplineParamSweep()
        {
            var cases = new[]
            {
                new { Name = "刚性90°",     Art = false, Turn = 90.0, Straight =  0.0 },
                new { Name = "铰接90°+拉直", Art = true,  Turn = 90.0, Straight = 25.0 },
            };

            // 预生成包络，避免每个参数组合重复仿真
            var envs = new List<(string name, List<Point2d> env)>();
            foreach (var c in cases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = c.Art;
                if (!c.Art) p.RigidRearOverhang = 2.0;
                var frames = TruckKinematics.SimulateFromFrontAxle(
                    p, new Point2d(0, 0), new Vector2d(1, 0), +1, c.Turn, c.Straight,
                    TruckKinematics.FineStepDeg);
                envs.Add((c.Name, TruckKinematics.EnvelopeTracks(frames, p)));
            }

            Console.WriteLine("  {0,5} {1,6} | {2}", "iters", "tol(m)",
                string.Join(" | ", envs.Select(e => e.name)));
            Console.WriteLine("  " + new string('-', 100));

            foreach (int iters in new[] { 10, 20, 40, 80 })
            {
                foreach (double tol in new[] { 0.005, 0.01, 0.02, 0.04 })
                {
                    var cells = new List<string>();
                    foreach (var e in envs)
                    {
                        var r = GeometryUtil.SimplifyEnvelopeForSpline(e.env, tol, 6, iters);
                        double a0 = Math.Abs(PolygonArea(e.env));
                        double a1 = Math.Abs(PolygonArea(r.loop));
                        double gain = a0 > 1e-9 ? (a1 - a0) / a0 * 100.0 : 0.0;
                        int z1cm = CountZigzag(r.loop, 5, 1e-2);
                        int z3cm = CountZigzag(r.loop, 5, 3e-2);
                        double mt = MaxTurnAngleDeg(r.loop);
                        cells.Add(string.Format(CultureInfo.InvariantCulture,
                            "{0,3}点 外扩{1:F3}m 齿>1cm {2,2}/>3cm {3,2} 面积{4,5:F2}% 转角{5,5:F1}°",
                            r.loop.Count, r.outset, z1cm, z3cm, gain, mt));
                    }
                    Console.WriteLine("  {0,5} {1,6:F3} | {2}", iters, tol, string.Join(" | ", cells));
                }
            }
        }

        /// <summary>
        /// v4.8 方案三专用：把「原始包络(灰细) vs 平滑稀化外推后(黄粗)+顶点红点」写成 SVG。
        /// 锯齿是几何问题，数值指标只能旁证，最终必须靠肉眼看这张图确认（与 WriteSvgFit 同理）。
        /// 输出到 TruckTurn.Verify/bin/Release/net8.0/VerifySpline_*.svg
        /// </summary>
        static void WriteSvgSpline(string label, List<Point2d> env, List<Point2d> simp)
        {
            if (env == null || env.Count < 3 || simp == null || simp.Count < 3) return;

            double minX = Math.Min(env.Min(q => q.X), simp.Min(q => q.X));
            double maxX = Math.Max(env.Max(q => q.X), simp.Max(q => q.X));
            double minY = Math.Min(env.Min(q => q.Y), simp.Min(q => q.Y));
            double maxY = Math.Max(env.Max(q => q.Y), simp.Max(q => q.Y));
            double pad = 1.0;
            minX -= pad; maxX += pad; minY -= pad; maxY += pad;
            double w = maxX - minX, h = maxY - minY;
            double scale = 900.0 / Math.Max(w, h);

            Func<double, double> Sx = x => (x - minX) * scale;
            Func<double, double> Sy = y => (maxY - y) * scale;

            var sb = new StringBuilder();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<svg xmlns='http://www.w3.org/2000/svg' width='{0:F0}' height='{1:F0}' viewBox='0 0 {0:F0} {1:F0}'>",
                w * scale, h * scale);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<rect width='{0:F0}' height='{1:F0}' fill='white'/>", w * scale, h * scale);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<text x='10' y='22' font-size='16' fill='#333'>{0}  原始 {1} 点 → 简化 {2} 点（黄=交给CAD画Spline的点，灰=原始包络）</text>",
                label, env.Count, simp.Count);
            sb.AppendLine();

            // 原始包络（灰细）
            var d0 = string.Join(" ", env.Select(q =>
                string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
            sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#999999' stroke-width='1.0'/>", d0);
            sb.AppendLine();

            // 平滑+稀化+外推后（黄粗）+ 顶点红点
            var d1 = string.Join(" ", simp.Select(q =>
                string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
            sb.AppendFormat("<polygon points='{0}' fill='rgba(255,193,7,0.18)' stroke='#D6A800' stroke-width='2.4'/>", d1);
            sb.AppendLine();
            foreach (var q in simp)
                sb.AppendFormat(CultureInfo.InvariantCulture,
                    "<circle cx='{0:F2}' cy='{1:F2}' r='2.4' fill='#D60000'/>", Sx(q.X), Sy(q.Y));
            sb.AppendLine();

            sb.AppendLine("</svg>");
            string name = string.Format(CultureInfo.InvariantCulture, "VerifySpline_{0}.svg", label);
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, name);
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        static int RunKinkAttribution()
        {
            Console.WriteLine("\n-- v4.9 新增：包络折角归因（定位大折角的来源部件 / 帧号）--");
            Console.WriteLine("   判据：拟合+采样后顶点转向角 > 30° 即为折角（弧按 24 段采样，单段仅几度）");

            var p = VehicleParams.Defaults();
            p.Articulated = true;

            var cases = new (string label, double[] turns, double[] straights)[]
            {
                ("常规90",    new[] { 90.0 },          new[] { 0.0 }),
                ("小30+直5",  new[] { 30.0 },          new[] { 5.0 }),
                ("S弯45+45",  new[] { 45.0, -45.0 },   new[] { 5.0, 5.0 }),
                ("折返90+90", new[] { 90.0, -90.0 },   new[] { 0.0, 0.0 }),
            };

            string[] partName = { "驾驶室", "底盘纵梁", "挂车" };
            const int parts = 3;

            foreach (var c in cases)
            {
                var frames = BuildMultiSegPath(p, c.turns, c.straights, TruckKinematics.FineStepDeg);
                var rects = FootprintRects(frames, p);
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                var centers = TruckKinematics.TurnCenters(frames, p);
                List<Point2d> ep; List<double> bg;
                GeometryUtil.FitEnvelopeBulges(env, centers, out ep, out bg);
                var sp = ep == null ? null : SampleBulgePolyline(ep, bg, 24);
                if (sp == null || sp.Count < 3) continue;

                int n = sp.Count;
                var byPart = new int[parts];
                var kinks = new List<Kink>();

                for (int i = 0; i < n; i++)
                {
                    var a = sp[(i - 1 + n) % n];
                    var v = sp[i];
                    var b = sp[(i + 1) % n];
                    double ux = v.X - a.X, uy = v.Y - a.Y;
                    double wx = b.X - v.X, wy = b.Y - v.Y;
                    double turn = Math.Atan2(ux * wy - uy * wx, ux * wx + uy * wy) * 180.0 / Math.PI;
                    if (Math.Abs(turn) < 30.0) continue;

                    // 归因：找离该顶点最近的 footprint 矩形边 → 帧号 + 部件
                    int fIdx = -1, pIdx = 0;
                    double best = double.MaxValue;
                    for (int ri = 0; ri < rects.Count; ri++)
                    {
                        var r = rects[ri];
                        for (int e = 0; e < 4; e++)
                        {
                            double d = DistPointSeg(v, r[e], r[(e + 1) % 4]);
                            if (d < best) { best = d; pIdx = ri % parts; fIdx = ri / parts; }
                        }
                    }
                    byPart[pIdx]++;
                    kinks.Add(new Kink { Turn = turn, Part = pIdx, Frame = fIdx, Pt = v, Dist = best });
                }

                Console.WriteLine("  · {0,-12} 原始{1,5} → 拟合{2,4} → 采样{3,5} | 折角>30° 共{4,3}个  [驾驶室{5,3} 底盘{6,3} 挂车{7,3}]  总帧{8}",
                    c.label, env.Count, ep.Count, n, kinks.Count, byPart[0], byPart[1], byPart[2], frames.Count);

                foreach (var k in kinks.OrderByDescending(k => Math.Abs(k.Turn)).Take(6))
                {
                    Console.WriteLine("       {0,8:F1}°  第{1,4}/{2}帧  {3,-5}  ({4,8:F2},{5,8:F2})  离车体{6:F3}m",
                        k.Turn, k.Frame, frames.Count, partName[k.Part], k.Pt.X, k.Pt.Y, k.Dist);
                }

                DumpEnvCsv(c.label, env, sp);
            }

            Console.WriteLine("       折角归因：{0} 条路径（纯诊断，不判失败）", cases.Length);
            return 0;
        }

        struct Kink
        {
            public double Turn; public int Part; public int Frame; public Point2d Pt; public double Dist;
        }

        /// <summary>点到线段的距离。</summary>
        static double DistPointSeg(Point2d q, Point2d a, Point2d b)
        {
            double dx = b.X - a.X, dy = b.Y - a.Y;
            double L2 = dx * dx + dy * dy;
            double t = L2 < 1e-18 ? 0.0 : ((q.X - a.X) * dx + (q.Y - a.Y) * dy) / L2;
            if (t < 0.0) t = 0.0; else if (t > 1.0) t = 1.0;
            double px = a.X + t * dx - q.X, py = a.Y + t * dy - q.Y;
            return Math.Sqrt(px * px + py * py);
        }

        /// <summary>导出包络点列到 CSV，供 Python 绘图肉眼复核。</summary>
        static void DumpEnvCsv(string label, List<Point2d> raw, List<Point2d> fitted)
        {
            var dir = AppDomain.CurrentDomain.BaseDirectory;
            var ci = CultureInfo.InvariantCulture;
            Action<string, List<Point2d>> write = (name, pts) =>
            {
                if (pts == null) return;
                var sb = new StringBuilder();
                sb.AppendLine("x,y");
                foreach (var q in pts) sb.AppendLine(string.Format(ci, "{0:F4},{1:F4}", q.X, q.Y));
                File.WriteAllText(Path.Combine(dir, name), sb.ToString(), new UTF8Encoding(false));
            };
            write(string.Format(ci, "env_{0}_raw.csv", label), raw);
            write(string.Format(ci, "env_{0}_fit.csv", label), fitted);
        }

        // =====================================================================
        // v4.9 方案 B：凸角外倒角（圆心 = 原顶点 V，半径 = r，圆心角 = turn）
        //
        // 几何推导：
        //   V → P1 = V + r·n_out1，P2 = V + r·n_out2（n_out 为边的外法向）
        //   弧 P1 → P2：圆心 V 半径 r，圆心角 |turn|，bulge = tan(turn/4)
        //   弧的最远点距 V = r·(1/cos(|turn|/2) − 1)
        //     · |turn|=90° → 外扩 0.414r（r=30cm 时 12.4cm）
        //     · |turn|=106° → 外扩 0.662r（r=30cm 时 19.9cm）
        //   与原边相连的新折角 ≈ atan(r / |邻边长|)
        //     · 邻边长 3r 时 ~18°，邻边长 5r 时 ~11°（仍能看见折角）
        //
        // 限制（必须满足才能做）：
        //   · 两侧邻段都是直线（bulge≈0），避免破坏已有圆弧
        //   · 邻边长 > 3r，否则新折角 >18° 比原来的折角还大
        //   · 仅处理凸角（turn > minTurn），凹角（off-tracking 凹口）必须保留
        //     否则会填平内轮差凹口 → 高估通过性，违反工程约束
        // =====================================================================
        static void FilletOutsideCorners(List<Point2d> pts, List<double> bulges,
            double radius, double minTurnDeg,
            out List<Point2d> outPts, out List<double> outBulges)
        {
            outPts = null;
            outBulges = null;
            int n = pts == null ? 0 : pts.Count;
            if (n < 3 || bulges == null || bulges.Count != n) return;

            double minTurn = minTurnDeg * Math.PI / 180.0;
            var np = new List<Point2d>();
            var nb = new List<double>();

            for (int i = 0; i < n; i++)
            {
                var a = pts[(i - 1 + n) % n];
                var v = pts[i];
                var b = pts[(i + 1) % n];
                double v1x = v.X - a.X, v1y = v.Y - a.Y;
                double v2x = b.X - v.X, v2y = b.Y - v.Y;
                double l1 = Math.Sqrt(v1x * v1x + v1y * v1y);
                double l2 = Math.Sqrt(v2x * v2x + v2y * v2y);
                double bIn = bulges[i];
                double bPrev = bulges[(i - 1 + n) % n];

                bool ok = l1 > 3.0 * radius && l2 > 3.0 * radius
                       && Math.Abs(bIn) < 1e-9 && Math.Abs(bPrev) < 1e-9;
                double turn = 0;
                if (ok)
                {
                    turn = Math.Atan2(v1x * v2y - v1y * v2x, v1x * v2x + v1y * v2y);
                    if (turn <= minTurn) ok = false;        // 仅凸角 (turn > 0)
                }
                if (!ok) { np.Add(v); nb.Add(bIn); continue; }

                // CCW 环材料在左 → 外侧在右 → 外法向 = (v.y, -v.x)/|v|
                double ox1 =  v1y / l1, oy1 = -v1x / l1;
                double ox2 =  v2y / l2, oy2 = -v2x / l2;
                var P1 = new Point2d(v.X + radius * ox1, v.Y + radius * oy1);
                var P2 = new Point2d(v.X + radius * ox2, v.Y + radius * oy2);
                double bul = Math.Tan(turn / 4.0);

                np.Add(P1); nb.Add(bul);     // P1 → P2：外凸弧
                np.Add(P2); nb.Add(bIn);     // P2 → B：原直线段（bIn = 0）
            }
            outPts = np;
            outBulges = nb;
        }

        static int RunFilletPlanB()
        {
            Console.WriteLine("\n-- v4.9 方案 B 实验：凸角外倒角（保留凹口）--");
            var p = VehicleParams.Defaults();
            p.Articulated = true;

            var cases = new (string label, double[] turns, double[] straights)[]
            {
                ("常规90",    new[] { 90.0 },          new[] { 0.0 }),
                ("小30+直5",  new[] { 30.0 },          new[] { 5.0 }),
                ("S弯45+45",  new[] { 45.0, -45.0 },   new[] { 5.0, 5.0 }),
                ("折返90+90", new[] { 90.0, -90.0 },   new[] { 0.0, 0.0 }),
            };
            double[] radii = { 0.10, 0.20, 0.30, 0.50 };
            double minTurn = 25.0;

            foreach (var c in cases)
            {
                var frames = BuildMultiSegPath(p, c.turns, c.straights, TruckKinematics.FineStepDeg);
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                var centers = TruckKinematics.TurnCenters(frames, p);
                List<Point2d> ep; List<double> bg;
                GeometryUtil.FitEnvelopeBulges(env, centers, out ep, out bg);
                if (ep == null) continue;

                var spOrig = SampleBulgePolyline(ep, bg, 24);
                int zig0 = CountZigzag(spOrig);
                double max0 = MaxTurnAngleDeg(spOrig);
                Console.WriteLine("  · {0,-10} 拟合 {1,3} 顶点 | 锯齿 {2,3} 最大折角 {3,6:F1}°",
                    c.label, ep.Count, zig0, max0);

                foreach (double r in radii)
                {
                    List<Point2d> op; List<double> ob;
                    FilletOutsideCorners(ep, bg, r, minTurn, out op, out ob);
                    if (op == null) { Console.WriteLine("       r={0:F2}m  跳过（不满足条件）", r); continue; }
                    var sp = SampleBulgePolyline(op, ob, 24);
                    int zig = CountZigzag(sp);
                    double max = MaxTurnAngleDeg(sp);

                    // 覆盖性：所有车体角点必须仍在包络内
                    int outside = CountCornersOutside(frames, p, sp);

                    // 外扩量：原采样点列上每个点，到新包络最近距离的 max
                    double maxExpand = 0.0;
                    foreach (var q in spOrig)
                    {
                        double d = -1.0;
                        // 点到多边形（带 bulge）的距离需要更精细的算法；
                        // 简化：点到采样点列的最小距离
                        foreach (var qq in sp)
                        {
                            double dx = qq.X - q.X, dy = qq.Y - q.Y;
                            double dd = Math.Sqrt(dx * dx + dy * dy);
                            if (d < 0 || dd < d) d = dd;
                        }
                        // 这里 d 是「原点到新点列最小距离」，不是「外扩量」；作为近似
                    }
                    // 用凸包近似算外扩更准：取新多边形上离原多边形最远的点
                    foreach (var q in sp)
                    {
                        double d = double.MaxValue;
                        foreach (var qq in spOrig)
                        {
                            double dx = q.X - qq.X, dy = q.Y - qq.Y;
                            double dd = Math.Sqrt(dx * dx + dy * dy);
                            if (dd < d) d = dd;
                        }
                        if (d > maxExpand) maxExpand = d;
                    }

                    Console.WriteLine("       r={0:F2}m  顶点{2,3} 锯齿{3,3} 最大折角{4,6:F1}°  外扩max={5:F3}m  覆盖{6}",
                        r, 0, op.Count, zig, max, maxExpand, outside == 0 ? "OK" : "*** 内缩 ***");

                    if (c.label == "常规90" && r == 0.30)
                        DumpEnvCsv(string.Format(CultureInfo.InvariantCulture, "{0}_r{1:F0}cm_fillet", c.label, r * 100), null, sp);
                }
            }
            Console.WriteLine("       方案 B 实验：4 条路径 × 4 半径（纯诊断）");
            return 0;
        }

        // v4.5.2 复刻用户报告「直线行驶时车尾乱飘」：
        // TRUCKDRIVE 在每段调用 TrySolveFromStateToTargetFrontAdaptive 自适应反解半径。
        // 当用户把光标移到几乎正前方（|d| < 1e-9）时，函数返回 rearRadius=+INF。
        // SimulateFromStateFront 拿到 turnRadiusOverride=+INF 后计算 ds=R*dTheta=+INF，
        // 直行段 nS=1, step=straightLen，单步 Euler 积分 dphi2/ds=sin(delta)/L2 严重过冲。
        // 这里直接调用 SimulateFromStateFront 复刻该路径，看 phi2 是否爆掉。
        static void DumpAdaptiveStraightBug()
        {
            var p = VehicleParams.Defaults(); p.Articulated = true;
            var f0 = new Frame
            {
                O1 = new Point2d(0, 0), U1 = new Vector2d(1, 0),
                K  = new Point2d(2.5, 0), O2 = new Point2d(13.5, 0), U2 = new Vector2d(1, 0)
            };
            var seg1 = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(f0, p),
                f0.U1, 0.0, +1, 2.6, 0.0, 1, TruckKinematics.FineStepDeg, 18.5);
            var last1 = seg1[seg1.Count - 1];
            double phi1_end = System.Math.Atan2(last1.U1.Y, last1.U1.X) * 180.0 / Math.PI;
            double phi2_end = System.Math.Atan2(last1.U2.Y, last1.U2.X) * 180.0 / Math.PI;
            System.Console.WriteLine(string.Format("  seg1 2.6deg末  phi1={0:F4}deg phi2={1:F4}deg delta={2:F4}deg",
                phi1_end, phi2_end, phi2_end - phi1_end));
            var seg2 = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(last1, p),
                last1.U1, System.Math.Atan2(last1.U2.Y, last1.U2.X), +1, 0.0, 60.0, 1, TruckKinematics.FineStepDeg,
                double.PositiveInfinity);
            System.Console.WriteLine("  seg2 60m (turnRadiusOverride=+INF): " + seg2.Count + " frames");
            for (int k = 0; k < seg2.Count; k++)
            {
                var f = seg2[k];
                double phi1 = System.Math.Atan2(f.U1.Y, f.U1.X) * 180.0 / Math.PI;
                double phi2 = System.Math.Atan2(f.U2.Y, f.U2.X) * 180.0 / Math.PI;
                System.Console.WriteLine(string.Format("    [{0,-3}] phi1={1,9:F5}deg phi2={2,9:F5}deg  delta={3,9:F5}deg",
                    k, phi1, phi2, phi2 - phi1));
            }
            // 额外：end-to-end 测试，调 TrySolveFromStateToTargetFrontAdaptive 走完整 TRUCKDRIVE 路径
            var t0 = TruckKinematics.FrontAxle(last1, p);
            // 沿当前车头方向走 60m：t1 - t0 必须与 U1 共线才能让 d→0（触发 R=+∞ 分支）
            var t1 = new Point2d(t0.X + 60.0 * last1.U1.X, t0.Y + 60.0 * last1.U1.Y);
            bool ok2 = TruckKinematics.TrySolveFromStateToTargetFrontAdaptive(p, t0, last1.U1, +1, t1,
                out double tTurn2, out double tStraight2, out double tR2);
            System.Console.WriteLine(string.Format("  TrySolve: ok={0} turn={1:F4}deg straight={2:F2}m R={3}", ok2, tTurn2, tStraight2, tR2));
            if (ok2)
            {
                var seg3 = TruckKinematics.SimulateFromStateFront(p, t0, last1.U1,
                    System.Math.Atan2(last1.U2.Y, last1.U2.X), +1, tTurn2, tStraight2, 1,
                    TruckKinematics.FineStepDeg, tR2);
                var last3 = seg3[seg3.Count - 1];
                double phi1e = System.Math.Atan2(last3.U1.Y, last3.U1.X) * 180.0 / Math.PI;
                double phi2e = System.Math.Atan2(last3.U2.Y, last3.U2.X) * 180.0 / Math.PI;
                System.Console.WriteLine(string.Format("  end-to-end 末  phi1={0:F4}deg phi2={1:F4}deg delta={2:F4}deg  ({3} frames)",
                    phi1e, phi2e, phi2e - phi1e, seg3.Count));
            }
        }
        // 复刻用户截图2：左侧光滑 + 右侧折线的多段急转场景。
        // 扫几种典型组合，看拟合后的 envPts 顶点数（>150 即还是折线锯齿）。
        static int RunMultiSharpTurns()
        {
            var p = VehicleParams.Defaults();
            p.Articulated = true;

            var cases = new (string label, double[] turns, double[] straights)[]
            {
                ("左90+直5+右90+直5", new[] { 90.0, -90.0 }, new[] { 5.0, 5.0 }),
                ("左90+直10+右90+直10", new[] { 90.0, -90.0 }, new[] { 10.0, 10.0 }),
                ("左90+直15+右90+直15", new[] { 90.0, -90.0 }, new[] { 15.0, 15.0 }),
                ("左90+直25+右90+直25（拉直）", new[] { 90.0, -90.0 }, new[] { 25.0, 25.0 }),
                ("左45+直5+右45+直5+左45+直5", new[] { 45.0, -45.0, 45.0 }, new[] { 5.0, 5.0, 5.0 }),
                ("左180+直5", new[] { 180.0 }, new[] { 5.0 }),
                ("左90+直0+右90+直0（折返）", new[] { 90.0, -90.0 }, new[] { 0.0, 0.0 }),
                ("左90+直2+右90+直2", new[] { 90.0, -90.0 }, new[] { 2.0, 2.0 }),
            };

            int fail = 0;
            foreach (var c in cases)
            {
                var frames = BuildMultiSegPath(p, c.turns, c.straights);
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                var centers = TruckKinematics.TurnCenters(frames, p);
                List<Point2d> envPts; List<double> bulges;
                GeometryUtil.FitEnvelopeBulges(env, centers, out envPts, out bulges);
                int arcs = bulges == null ? 0 : bulges.Count(b => Math.Abs(b) > 1e-6);
                double maxTurnFit = envPts == null ? 0 : MaxTurnAngleDeg(SampleBulgePolyline(envPts, bulges, 64));
                int zig = envPts == null ? 0 : CountZigzag(SampleBulgePolyline(envPts, bulges, 64));
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  {0,-40} env={1,5} fit_pts={2,4} arcs={3,3} max_turn={4,6:F2}deg zigzag={5}  {6}",
                    c.label, env.Count,
                    envPts == null ? 0 : envPts.Count, arcs, maxTurnFit, zig,
                    envPts != null && envPts.Count < 250 ? "OK" : "*** 仍为折线 ***"));
            }
            Console.WriteLine(string.Format("       多段急转：{0} 个组合，{1} 个失败", cases.Length, fail));
            return fail;
        }

        // v4.8：新增可选 step 参数，便于对比预览步长（1°/0.5°）与出图步长（0.25°）。
        static List<Frame> BuildMultiSegPath(VehicleParams p, double[] turns, double[] straights,
                                             double step = 0.0)
        {
            if (step <= 0) step = TruckKinematics.FineStepDeg;
            var frames = new List<Frame>();
            Point2d f0 = new Point2d(0, 0);
            Vector2d u0 = new Vector2d(1, 0);
            double phi2 = 0.0;
            int n = Math.Min(turns.Length, straights.Length);
            for (int i = 0; i < n; i++)
            {
                var seg = TruckKinematics.SimulateFromStateFront(p, f0, u0, phi2,
                    Math.Sign(turns[i]) >= 0 ? +1 : -1,
                    Math.Abs(turns[i]), straights[i], 1, step);
                if (i == 0) frames.AddRange(seg);
                else for (int k = 1; k < seg.Count; k++) frames.Add(seg[k]);
                var last = seg[seg.Count - 1];
                f0 = TruckKinematics.FrontAxle(last, p);
                u0 = last.U1;
                phi2 = System.Math.Atan2(last.U2.Y, last.U2.X);
            }
            return frames;
        }

        static int RunConsecutiveOne(bool articulated, double turnDeg, double straightLen)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = articulated;
            if (!articulated) p.RigidRearOverhang = 2.0;

            // 与 Commands.TruckDrive 逐行一致：段间携带挂车航向，以前轴中心续接
            var frames = new List<Frame>();
            var f0 = new Point2d(0.0, 0.0);
            var u0 = new Vector2d(1.0, 0.0);
            var seg1 = TruckKinematics.SimulateFromStateFront(p, f0, u0, 0.0, +1,
                            turnDeg, 0.0, 1, TruckKinematics.FineStepDeg);
            frames.AddRange(seg1);
            var last = seg1[seg1.Count - 1];
            if (straightLen > 1e-9)
            {
                var seg2 = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(last, p), last.U1,
                    Math.Atan2(last.U2.Y, last.U2.X), +1, 0.0, straightLen, 1, TruckKinematics.FineStepDeg);
                for (int i = 1; i < seg2.Count; i++) frames.Add(seg2[i]);
                last = seg2[seg2.Count - 1];
            }
            var seg3 = TruckKinematics.SimulateFromStateFront(p, TruckKinematics.FrontAxle(last, p), last.U1,
                Math.Atan2(last.U2.Y, last.U2.X), +1, turnDeg, 0.0, 1, TruckKinematics.FineStepDeg);
            for (int i = 1; i < seg3.Count; i++) frames.Add(seg3[i]);

            var env = TruckKinematics.EnvelopeTracks(frames, p);
            var centers = TruckKinematics.TurnCenters(frames, p);
            List<Point2d> envPts;
            List<double> bulges;
            GeometryUtil.FitEnvelopeBulges(env, centers, out envPts, out bulges);

            int outside = int.MaxValue, inside = int.MaxValue;
            if (env.Count >= 3 && envPts != null && envPts.Count >= 3)
            {
                var sampled = SampleBulgePolyline(envPts, bulges, 64);
                s_geomTol = FitTol;
                outside = CountCornersOutside(frames, p, sampled);
                inside = CountBoundaryPtsStrictlyInside(frames, p, sampled);
                s_geomTol = Tol;
            }

            // 保底分支（凸包）被触发 = 并集边界引擎退化。凸包会填平铰接车 off-tracking 的
            // 真实凹口，不能当正常结果接受，直接判失败，逼着回去修引擎。
            bool fallback = TruckKinematics.EnvelopeFallbackUsed;
            bool ok = env.Count >= 3 && envPts != null && envPts.Count >= 3
                      && outside == 0 && inside == 0 && !fallback;
            if (!ok)
            {
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "       {0} 转{1,4:F0}° + 直{2,4:F0}m + 转{1,4:F0}°：frames={3,5} raw_env={4,5} fit={5,4}"
                    + "  outside={6} inside={7}{8}   *** FAIL ***",
                    articulated ? "铰接" : "刚性", turnDeg, straightLen, frames.Count, env.Count,
                    envPts == null ? 0 : envPts.Count, outside, inside,
                    fallback ? " [凸包保底被触发：并集边界成环失败]" : ""));
                return 1;
            }
            return 0;
        }

        // ================= v4.5 验证：TRUCKTURN90 整条路径（转弯+拉直）的包络与拟合 =================
        // 动机：用户反馈「铰链车转动 90° 不显示包络」。Run() 只校验纯 90° 圆弧，
        // 而 TRUCKTURN90 实际走的是 SimulateTurn90AndStraighten —— 末尾多出一段 25m 直行，
        // 是一条「圆弧 + 长直」的混合路径。混合路径上瞬时中心会漂移，
        // 拟合器走的是与纯圆弧完全不同的分支，此前从未被校验过。
        // 因此这里完全复刻 Commands.DrawSegmentTracks 的调用序列，并检查
        // 结果可直接交给 CAD 建多段线（无 NaN/Inf、无退化 bulge）。
        static int RunStraightenEnvelope(string label, bool articulated, double stepDeg)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = articulated;
            if (!articulated) p.RigidRearOverhang = 2.0;

            var startFront = new Point2d(0.0, 0.0);
            var heading = new Vector2d(1.0, 0.0);
            var frames = TruckKinematics.SimulateTurn90AndStraighten(p, startFront, heading, +1, stepDeg);
            var env = TruckKinematics.EnvelopeTracks(frames, p);

            double stepRot = MaxStepRotationDeg(frames);
            double maxArt = articulated ? MaxArticulationDeg(frames) : 0.0;
            double bound = 90.0 + stepRot + maxArt + 1e-6;

            // ---- 完全复刻 Commands.DrawSegmentTracks 的调用序列 ----
            // 用同一个入口（含伪尖刺守卫），否则验证的不是实际出图结果。
            var centers = TruckKinematics.TurnCenters(frames, p);
            List<Point2d> envPts;
            List<double> bulges;
            GeometryUtil.FitEnvelopeBulges(env, centers, out envPts, out bulges);

            // ---- 能否安全交给 CAD ----
            bool finite = envPts != null && envPts.Count >= 3 && bulges != null && bulges.Count == envPts.Count;
            double maxAbsBulge = 0.0;
            if (finite)
            {
                for (int i = 0; i < envPts.Count; i++)
                {
                    double x = envPts[i].X, y = envPts[i].Y;
                    if (double.IsNaN(x) || double.IsNaN(y) || double.IsInfinity(x) || double.IsInfinity(y))
                    { finite = false; break; }
                    double bv = bulges[i];
                    if (double.IsNaN(bv) || double.IsInfinity(bv)) { finite = false; break; }
                    if (Math.Abs(bv) > maxAbsBulge) maxAbsBulge = Math.Abs(bv);
                }
            }
            // bulge = tan(θ/4)，θ 上限取 179° → |bulge| ≤ tan(44.75°) ≈ 0.991。
            // 超界意味着圆心跑到了弦的错误一侧，CAD 里会画出镜像/翻转的弧。
            bool bulgeSane = maxAbsBulge <= 1.0 + 1e-9;

            int arcs = bulges != null ? bulges.Count(b => Math.Abs(b) > 1e-6) : 0;

            // ---- 覆盖性不变量（在拟合结果上量）----
            var envSampled = SampleBulgePolyline(envPts, bulges, 64);
            s_geomTol = FitTol;
            int cornersOutside = CountCornersOutside(frames, p, envSampled);
            int strictlyInside = CountBoundaryPtsStrictlyInside(frames, p, envSampled);
            s_geomTol = Tol;
            double maxTurn = MaxTurnAngleDeg(envSampled);

            double maxInf = 0.0;
            for (int qi = 0; qi < envSampled.Count; qi++)
            {
                var q = envSampled[qi];
                if (PointInPolygon(q, env)) continue;
                double d = DistanceToPolyline(q, env);
                if (d > maxInf) maxInf = d;
            }

            bool ok = finite && bulgeSane && cornersOutside == 0 && strictlyInside == 0
                      && maxTurn <= bound && maxInf <= 0.10 && !TruckKinematics.EnvelopeFallbackUsed;

            Console.WriteLine(
                string.Format(CultureInfo.InvariantCulture,
                    "  {0} frames={1,4}  env_pts={2,5}  fit_vertices={3,4} arcs={4,3}  |bulge|max={5:F3}"
                    + "  corners_outside={6} strictly_inside={7}  max_turn={8,6:F2}deg (bound={9,6:F2})"
                    + "  max_inf={10:F3}m  zigzag={11}→{12} 直线段最大折角={13,5:F1}deg   {14}",
                    label, frames.Count, env.Count, envPts == null ? 0 : envPts.Count, arcs,
                    maxAbsBulge, cornersOutside, strictlyInside, maxTurn, bound, maxInf,
                    CountZigzag(env), CountZigzag(envSampled),
                    MaxTurnInStraightRuns(envPts, bulges), ok ? "OK" : "*** FAIL ***"));
            if (!finite)
                Console.WriteLine("       [fit] 拟合结果含 NaN/Inf 或顶点数 <3，CAD 建多段线会抛异常 → 包络不显示");
            else if (!bulgeSane)
                Console.WriteLine("       [fit] |bulge| 越界（>1.0），弧会翻转");
            // 混合路径（转弯+拉直）是用户 v4.5 报「铰接车 90° 不显示包络」的场景，
            // 必须留图人工复核，光看数字确认不了 CAD 里到底画没画出来。
            WriteSvgFit(label, 90.0, stepDeg, frames, p, env, envPts, bulges);
            return ok ? 0 : 1;
        }

        // 统计连续转弯帧数（航向仍在变化），近似区分转弯段与拉直段
        static int CountTurnFrames(List<Frame> frames)
        {
            int n = frames.Count;
            if (n < 2) return n;
            // 从末尾向前找，直到牵引车航向变化重新出现（即进入转弯段）
            // 更简单：拉直段的特点是 U1 不变；从末尾向前统计 U1 不变的连续帧
            int straight = 1;
            for (int i = n - 2; i >= 0; i--)
            {
                if ((frames[i].U1 - frames[i + 1].U1).Length < 1e-9) straight++;
                else break;
            }
            return n - straight;
        }

        static double NormalizeAngleDeg(double a)
        {
            while (a > 180.0) a -= 360.0;
            while (a < -180.0) a += 360.0;
            return a;
        }

        // ================= v4.4 验证 2：挂车不得侵入驾驶室 =================
        // 判据用 SAT 有符号间隙：>0 分离，<0 穿透（|值| = 把两者分开所需的最小平移）。
        // 注意只检查「挂车 vs 驾驶室」：挂车压在窄车架上方是真车的正常形态，不算穿透。
        static int RunNoPenetration(string label, bool articulated, double turnDeg, double stepDeg)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = articulated;
            if (!articulated) p.RigidRearOverhang = 2.0;

            var startFront = new Point2d(0.0, 0.0);
            var heading = new Vector2d(1.0, 0.0);

            var frames = TruckKinematics.SimulateTurn90AndStraighten(p, startFront, heading, +1, stepDeg);
            int bad = 0;
            double minGap = double.MaxValue;
            double worst = 0.0;
            foreach (var f in frames)
            {
                var c = TruckKinematics.Corners(f, p);
                var tractor = new[] { c[0], c[1], c[3], c[2] };
                var trailer = new[] { c[4], c[5], c[7], c[6] };
                double gap = RectSignedGap(tractor, trailer);
                if (gap < minGap) minGap = gap;
                if (gap < worst) worst = gap;
                if (gap < -1e-6) bad++;
            }

            bool ok = bad == 0;
            Console.WriteLine(
                string.Format(CultureInfo.InvariantCulture,
                    "  {0} frames={1,4}  min_gap={2,8:F4}m  worst_penetration={3,7:F4}m  penetrations={4}/{5}   {6}",
                    label, frames.Count, minGap, Math.Min(0.0, worst), bad, frames.Count,
                    ok ? "OK" : "*** FAIL ***"));
            return ok ? 0 : 1;
        }

        /// <summary>
        /// 两个凸多边形的有符号间隙（分离轴定理 SAT）：
        ///   &gt; 0 → 分离，值 = 最小间距；&lt; 0 → 穿透，|值| = 最小穿透深度（把两者分开所需的最小平移）。
        ///
        /// 注意：不能用「点到边最近距离」来判断穿透。当 A 的一个角插进 B 内部（但不含 B 的重心）时，
        /// 点到边的距离反而是正值，会被误判成「有间隙」。这正是上一版报 min_gap=0.0000 却漏检穿透的原因。
        /// SAT 对凸多边形是充要判据：存在不重叠的投影轴 ⇔ 分离。
        /// </summary>
        static double RectSignedGap(Point2d[] a, Point2d[] b)
        {
            bool separated = false;
            double maxSeparation = double.MinValue;   // 分离轴上最大的间隙
            double minOverlap = double.MaxValue;      // 全部轴重叠时的最小重叠量 = 穿透深度

            for (int poly = 0; poly < 2; poly++)
            {
                var P = poly == 0 ? a : b;
                int n = P.Length;
                for (int i = 0; i < n; i++)
                {
                    var p1 = P[i];
                    var p2 = P[(i + 1) % n];
                    double ex = p2.X - p1.X, ey = p2.Y - p1.Y;
                    double len = Math.Sqrt(ex * ex + ey * ey);
                    if (len < 1e-12) continue;
                    // 边的外法线作为分离轴
                    var axis = new Vector2d(-ey / len, ex / len);

                    Project(a, axis, out double aMin, out double aMax);
                    Project(b, axis, out double bMin, out double bMax);

                    double overlap = Math.Min(aMax, bMax) - Math.Max(aMin, bMin);
                    if (overlap <= 0.0)
                    {
                        separated = true;
                        if (-overlap > maxSeparation) maxSeparation = -overlap;
                    }
                    else if (overlap < minOverlap)
                    {
                        minOverlap = overlap;
                    }
                }
            }

            return separated ? maxSeparation : -minOverlap;
        }

        static void Project(Point2d[] poly, Vector2d axis, out double min, out double max)
        {
            min = double.MaxValue; max = double.MinValue;
            for (int i = 0; i < poly.Length; i++)
            {
                double d = poly[i].X * axis.X + poly[i].Y * axis.Y;
                if (d < min) min = d;
                if (d > max) max = d;
            }
        }

        static Point2d RectCenter(Point2d[] r)
        {
            double sx = 0, sy = 0;
            foreach (var p in r) { sx += p.X; sy += p.Y; }
            return new Point2d(sx / r.Length, sy / r.Length);
        }

        static bool PointInRect(Point2d p, Point2d[] r)
        {
            int n = r.Length;
            for (int i = 0; i < n; i++)
            {
                var a = r[i];
                var b = r[(i + 1) % n];
                var e = b - a;
                var m = new Vector2d(-e.Y, e.X);
                double c = m.X * a.X + m.Y * a.Y;
                if (m.X * p.X + m.Y * p.Y - c < -1e-9) return false;
            }
            return true;
        }

        // ================= 不变量 1：车体角点必须被包络覆盖 =================
        /// <summary>最近一次 CountCornersOutside 统计到的最大越界距离（米）。用于区分
        /// 「数值噪声」与「真的漏覆盖」——包络拟合允许 1e-4 量级的外形偏差。</summary>
        static double s_maxOutsideDist = 0.0;

        static int CountCornersOutside(List<Frame> frames, VehicleParams p, List<Point2d> env)
        {
            if (env == null || env.Count < 3) return int.MaxValue;
            int bad = 0;
            double maxDist = 0.0;
            foreach (var f in frames)
            {
                foreach (var c in BodyCorners(f, p))
                {
                    if (PointInPolygon(c, env)) continue;
                    double d = DistanceToPolyline(c, env);
                    if (d > maxDist) maxDist = d;
                    if (d <= s_geomTol) continue; // 落在边界上算覆盖
                    bad++;
                }
            }
            s_maxOutsideDist = maxDist;
            return bad;
        }

        /// <summary>一帧的全部车体角点：驾驶室 4 + 车架 4（铰接）+ 挂车 4。
        /// v4.4 起车架也是真实车体，必须被包络覆盖。</summary>
        static IEnumerable<Point2d> BodyCorners(Frame f, VehicleParams p)
        {
            foreach (var c in TruckKinematics.Corners(f, p)) yield return c;
            var chassis = TruckKinematics.ChassisRect(f, p);
            if (chassis != null)
                foreach (var c in chassis) yield return c;
        }

        // ================= 不变量 2：包络顶点不得严格位于任何车体矩形内部 =================
        static int CountBoundaryPtsStrictlyInside(List<Frame> frames, VehicleParams p, List<Point2d> env)
        {
            if (env == null || env.Count < 3) return int.MaxValue;
            var rects = FootprintRects(frames, p);
            int bad = 0;
            foreach (var v in env)
                foreach (var r in rects)
                    if (StrictlyInsideRect(v, r)) { bad++; break; }
            return bad;
        }

        // ================= 车体 footprint 矩形（与引擎内实现一致） =================
        static List<Point2d[]> FootprintRects(List<Frame> frames, VehicleParams p)
        {
            var rects = new List<Point2d[]>();
            foreach (var f in frames)
            {
                var c = TruckKinematics.Corners(f, p);
                rects.Add(MakeCCW(new[] { c[0], c[2], c[3], c[1] }));   // 牵引车（驾驶室）
                if (p.Articulated)
                {
                    var chassis = TruckKinematics.ChassisRect(f, p);
                    if (chassis != null)
                        rects.Add(MakeCCW(new[] { chassis[0], chassis[3], chassis[2], chassis[1] })); // 车架
                    rects.Add(MakeCCW(new[] { c[4], c[6], c[7], c[5] })); // 挂车
                }
            }
            return rects;
        }

        static Point2d[] MakeCCW(Point2d[] poly)
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

        // ================= 基础几何 =================
        static bool PointInPolygon(Point2d pt, List<Point2d> poly)
        {
            bool inside = false;
            int n = poly.Count;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var a = poly[i];
                var b = poly[j];
                if (((a.Y > pt.Y) != (b.Y > pt.Y)) &&
                    (pt.X < (b.X - a.X) * (pt.Y - a.Y) / (b.Y - a.Y) + a.X))
                    inside = !inside;
            }
            return inside;
        }

        /// <summary>最近一次统计中「最深的虚构内部点」的内侵深度（米）。</summary>
        static double s_maxInsideDepth = 0.0;

        /// <summary>最近一次 bulge 拟合跑到原始并集边界之外的最大距离（米）。</summary>
        static double s_maxInflation = 0.0;

        static bool StrictlyInsideRect(Point2d pt, Point2d[] rect)
        {
            int n = rect.Length;
            double minDepth = double.MaxValue;
            for (int i = 0; i < n; i++)
            {
                var a = rect[i];
                var b = rect[(i + 1) % n];
                var e = b - a;
                var m = new Vector2d(-e.Y, e.X);          // 左法向 = 内法向（CCW）
                double len = m.Length;
                if (len < 1e-12) return false;
                m = m * (1.0 / len);                       // 归一化，使 Tol 有实际意义
                double c = m.X * a.X + m.Y * a.Y;
                double depth = (m.X * pt.X + m.Y * pt.Y) - c;
                if (depth < minDepth) minDepth = depth;
                if (depth <= s_geomTol) return false; // 必须严格在内
            }
            if (minDepth > s_maxInsideDepth) s_maxInsideDepth = minDepth;
            return true;
        }

        static double DistanceToPolyline(Point2d pt, List<Point2d> poly)
        {
            double best = double.MaxValue;
            int n = poly.Count;
            for (int i = 0; i < n; i++)
                best = Math.Min(best, DistancePointSegment(pt, poly[i], poly[(i + 1) % n]));
            return best;
        }

        static double DistancePointSegment(Point2d p, Point2d a, Point2d b)
        {
            var ab = b - a;
            double len2 = ab.X * ab.X + ab.Y * ab.Y;
            double t = 0.0;
            if (len2 > 1e-18)
            {
                t = ((p.X - a.X) * ab.X + (p.Y - a.Y) * ab.Y) / len2;
                if (t < 0) t = 0; else if (t > 1) t = 1;
            }
            var proj = a + ab * t;
            var d = p - proj;
            return Math.Sqrt(d.X * d.X + d.Y * d.Y);
        }

        static double MaxTurnAngleDeg(List<Point2d> poly)
        {
            int n = poly.Count;
            if (n < 3) return 0.0;
            double maxA = 0.0;
            for (int i = 0; i < n; i++)
            {
                var prev = poly[(i - 1 + n) % n];
                var cur = poly[i];
                var next = poly[(i + 1) % n];
                var v1 = cur - prev;
                var v2 = next - cur;
                if (v1.Length < 1e-9 || v2.Length < 1e-9) continue;
                double cross = v1.X * v2.Y - v1.Y * v2.X;
                double dot = v1.X * v2.X + v1.Y * v2.Y;
                double a = Math.Abs(Math.Atan2(cross, dot)) * 180.0 / Math.PI;
                // 折角取较小的那个（转向的偏离角），而非 360-该值
                if (a > 180.0) a = 360.0 - a;
                if (a > maxA) maxA = a;
            }
            return maxA;
        }

        /// <summary>相邻两帧之间车身（牵引车/挂车）的最大转角增量，单位度。用于推导折角上界。</summary>
        static double MaxStepRotationDeg(List<Frame> frames)
        {
            double maxRot = 0.0;
            for (int i = 1; i < frames.Count; i++)
            {
                maxRot = Math.Max(maxRot, Math.Abs(AngleDeltaDeg(frames[i - 1].U1, frames[i].U1)));
                maxRot = Math.Max(maxRot, Math.Abs(AngleDeltaDeg(frames[i - 1].U2, frames[i].U2)));
            }
            return maxRot;
        }

        /// <summary>铰接车在整个序列中的最大绝对铰接角（度）。底盘/挂车接缝会暴露真实折角 90°+铰接角。</summary>
        static double MaxArticulationDeg(List<Frame> frames)
        {
            double maxA = 0.0;
            foreach (var f in frames)
            {
                double a = Math.Abs(AngleDeltaDeg(f.U1, f.U2));
                if (a > maxA) maxA = a;
            }
            return maxA;
        }

        static double AngleDeltaDeg(Vector2d a, Vector2d b)
        {
            double cross = a.X * b.Y - a.Y * b.X;
            double dot = a.X * b.X + a.Y * b.Y;
            return Math.Atan2(cross, dot) * 180.0 / Math.PI;
        }

        static double PolygonArea(List<Point2d> poly)
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

        /// <summary>把闭合折线按等弧长重采样，使下游指标不受顶点分布疏密影响。</summary>
        static List<Point2d> ResampleUniform(List<Point2d> pts, double step)
        {
            var res = new List<Point2d>();
            int n = pts == null ? 0 : pts.Count;
            if (n < 3) return res;
            var seg = new double[n];
            double total = 0.0;
            for (int i = 0; i < n; i++)
            {
                var a = pts[i]; var b = pts[(i + 1) % n];
                seg[i] = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
                total += seg[i];
            }
            if (total < 1e-9) return res;
            int m = Math.Max(16, (int)(total / step));
            double ds = total / m;
            int k = 0; double acc = 0.0;
            for (int j = 0; j < m; j++)
            {
                double target = j * ds;
                while (k < n - 1 && acc + seg[k] < target) { acc += seg[k]; k++; }
                double t = seg[k] < 1e-12 ? 0.0 : (target - acc) / seg[k];
                if (t < 0) t = 0; else if (t > 1) t = 1;
                var a = pts[k]; var b = pts[(k + 1) % n];
                res.Add(new Point2d(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t));
            }
            return res;
        }

        /// <summary>
        /// 直线段（bulge=0）内部顶点的最大折角（度）。
        ///
        /// zigzag 测的是「整条边界的振荡个数」，这个测的是「没被拟合成弧的那部分
        /// 到底有多不平」—— 拉直段走廊边若还是一串台阶，顶点处的折角就会很大。
        /// 拟合良好的直线段，顶点折角应接近 0（共线）或小角度（弧与直线的相切接缝）。
        /// </summary>
        static double MaxTurnInStraightRuns(List<Point2d> pts, List<double> bulges)
        {
            int n = pts == null ? 0 : pts.Count;
            if (n < 3 || bulges == null || bulges.Count != n) return 0.0;
            double maxDeg = 0.0;
            for (int i = 0; i < n; i++)
            {
                if (Math.Abs(bulges[i]) > 1e-9) continue;
                var prev = pts[(i - 1 + n) % n];
                var cur = pts[i];
                var next = pts[(i + 1) % n];
                double ax = cur.X - prev.X, ay = cur.Y - prev.Y;
                double bx = next.X - cur.X, by = next.Y - cur.Y;
                double la = Math.Sqrt(ax * ax + ay * ay);
                double lb = Math.Sqrt(bx * bx + by * by);
                if (la < 1e-9 || lb < 1e-9) continue;
                double deg = Math.Abs(Math.Atan2(ax * by - ay * bx, ax * bx + ay * by)) * 180.0 / Math.PI;
                if (deg > maxDeg) maxDeg = deg;
            }
            return maxDeg;
        }

        /// <summary>
        /// 边界上的「锯齿数」：法向偏移的高频振荡极值点个数。
        ///
        /// 「包络看着扎不扎眼」这件事，顶点数、最大折角、覆盖性都测不出来 ——
        /// 一条 30m 长的走廊边上有 800 个 3cm 台阶，最大折角仍是 90°（和车体直角
        /// 一样），覆盖性也满分，但放大看就是明显的锯条。这里直接量高频振荡：
        /// 先算每点相对局部弦（前后各 win 个点）的有符号法向偏移，再数这个偏移
        /// 序列的极值点；光滑曲线上只有少数几个曲率极值，锯齿带上每个台阶一个。
        ///
        /// ampMin 只统计幅度超过它的振荡，滤掉数值噪声。取 3mm：台阶振幅实测 3cm，
        /// 比它小一个量级，不会被漏掉；浮点噪声则在微米级。
        ///
        /// 测量前必须等弧长重采样： bulge 结果里弧按 64 段采样、直线只 1 段，
        /// 点距相差两个数量级，固定点数的窗口会横跨密度突变处，
        /// 把局部弦方向估计的跳变当成振荡数进来（实测能凭空多出上百个伪极值）。
        /// </summary>
        static int CountZigzag(List<Point2d> pts, int win = 5, double ampMin = 3e-3)
        {
            pts = ResampleUniform(pts, 0.05);
            int n = pts == null ? 0 : pts.Count;
            if (n < 3 * win) return 0;
            var off = new double[n];
            for (int i = 0; i < n; i++)
            {
                var a = pts[(i - win + n) % n];
                var b = pts[(i + win) % n];
                var c = pts[i];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double L = Math.Sqrt(dx * dx + dy * dy);
                off[i] = L < 1e-12 ? 0.0 : ((c.X - a.X) * dy - (c.Y - a.Y) * dx) / L;
            }
            int cnt = 0;
            for (int i = 0; i < n; i++)
            {
                double p = off[(i - 1 + n) % n], c = off[i], q = off[(i + 1) % n];
                if ((c - p) * (q - c) < 0 && Math.Abs(c) > ampMin) cnt++;
            }
            return cnt;
        }

        /// <summary>把带 bulge 的多段线密集采样为普通点列，用于不变量验证。</summary>
        static List<Point2d> SampleBulgePolyline(List<Point2d> pts, List<double> bulges, int samplesPerArc)
        {
            var result = new List<Point2d>();
            int n = pts.Count;
            if (n < 2) return result;
            for (int i = 0; i < n; i++)
            {
                Point2d a = pts[i];
                Point2d b = pts[(i + 1) % n];
                double bval = (bulges != null && i < bulges.Count) ? bulges[i] : 0.0;
                result.Add(a);
                if (Math.Abs(bval) < 1e-9) continue;

                double theta = 4.0 * Math.Atan(bval);
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double chord = Math.Sqrt(dx * dx + dy * dy);
                double denom = 2.0 * Math.Sin(theta / 2.0);
                if (Math.Abs(denom) < 1e-9 || chord < 1e-9) continue;
                double radius = Math.Abs(chord / denom);
                if (radius > 1e6 || double.IsNaN(radius)) continue;

                // 弦中点 + 垂直偏移得到圆心。正 bulge：弧向弦左侧凸出，圆心在左侧。
                Point2d mid = new Point2d((a.X + b.X) / 2.0, (a.Y + b.Y) / 2.0);
                Vector2d u = (b - a).GetNormal();
                Vector2d left = new Vector2d(-u.Y, u.X);
                double d = radius * Math.Cos(theta / 2.0);
                Point2d center = mid + Math.Sign(bval) * d * left;

                Vector2d va = a - center;
                double startAng = Math.Atan2(va.Y, va.X);
                for (int s = 1; s < samplesPerArc; s++)
                {
                    double t = s / (double)samplesPerArc;
                    double ang = startAng + t * theta;
                    result.Add(new Point2d(center.X + radius * Math.Cos(ang),
                                           center.Y + radius * Math.Sin(ang)));
                }
            }
            return result;
        }

        /// <summary>诊断：逐条 bulge 弧计算其采样点相对原始包络的有符号偏差
        /// （正=跑到包络外，负=缩进包络内），定位错位的弧。</summary>
        static void DumpArcDeviation(List<Point2d> envFit, List<double> bulges, List<Point2d> envRaw)
        {
            if (bulges == null) return;
            int n = envFit.Count;
            var worst = new List<string>();
            for (int i = 0; i < n; i++)
            {
                double b = i < bulges.Count ? bulges[i] : 0.0;
                if (Math.Abs(b) < 1e-9) continue;
                Point2d a = envFit[i], bb = envFit[(i + 1) % n];
                double ang = 4.0 * Math.Atan(b) * 180.0 / Math.PI;
                double maxDev = 0;
                for (int s = 1; s < 16; s++)
                {
                    double t = s / 16.0;
                    Point2d pt = ArcPointAt(a, bb, b, t);
                    double d = DistanceToPolyline(pt, envRaw);
                    if (!PointInPolygon(pt, envRaw)) d = -d; // 在原始包络外 → 记为负（内缩）
                    if (Math.Abs(d) > Math.Abs(maxDev)) maxDev = d;
                }
                worst.Add(string.Format(CultureInfo.InvariantCulture,
                    "v{0}(ang={1:F2},dev={2:E1})", i, ang, maxDev));
            }
            worst.Sort((x, y) => Math.Abs(double.Parse(y.Split(new[] { "dev=" }, StringSplitOptions.None)[1].TrimEnd(')'),
                                          CultureInfo.InvariantCulture))
                                .CompareTo(Math.Abs(double.Parse(x.Split(new[] { "dev=" }, StringSplitOptions.None)[1].TrimEnd(')'),
                                          CultureInfo.InvariantCulture))));
            Console.WriteLine("       [arcdev] worst: " + string.Join("  ", worst.Take(5)));
        }

        /// <summary>bulge 弧上参数 t∈[0,1] 处的点。</summary>
        static Point2d ArcPointAt(Point2d a, Point2d b, double bulge, double t)
        {
            double theta = 4.0 * Math.Atan(bulge);
            double chord = Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
            double denom = 2.0 * Math.Sin(theta / 2.0);
            if (Math.Abs(denom) < 1e-12 || chord < 1e-12) return a;
            double radius = Math.Abs(chord / denom);
            var mid = new Point2d((a.X + b.X) / 2.0, (a.Y + b.Y) / 2.0);
            var u = new Vector2d((b.X - a.X) / chord, (b.Y - a.Y) / chord);
            var left = new Vector2d(-u.Y, u.X);
            double d = radius * Math.Cos(theta / 2.0);
            var center = mid + Math.Sign(bulge) * d * left;
            var va = a - center;
            double startAng = Math.Atan2(va.Y, va.X);
            double ang = startAng + t * theta;
            return new Point2d(center.X + radius * Math.Cos(ang), center.Y + radius * Math.Sin(ang));
        }

        /// <summary>诊断：统计 bulge 弧的圆心角分布与面积变化。
        /// 圆心角 &gt; 180° 时 BulgeFromArc（atan2）会折返成负角，弧会朝反方向弯。</summary>
        static void DumpBulgeStats(List<Point2d> envFit, List<double> bulges, List<Point2d> envRaw)
        {
            if (bulges == null || bulges.Count == 0) return;
            double maxAbs = 0; int nOver180 = 0, nArc = 0;
            double minA = double.MaxValue, maxA = 0;
            for (int i = 0; i < bulges.Count; i++)
            {
                double b = Math.Abs(bulges[i]);
                if (b < 1e-6) continue;
                nArc++;
                if (b > maxAbs) maxAbs = b;
                double ang = 4.0 * Math.Atan(b) * 180.0 / Math.PI;
                if (ang < minA) minA = ang;
                if (ang > maxA) maxA = ang;
                if (b > 1.0 + 1e-9) nOver180++; // |bulge|>tan(45°) 意味着圆心角 > 180°
            }
            double areaFit = Math.Abs(PolygonArea(envFit));
            double areaRaw = Math.Abs(PolygonArea(envRaw));
            // 顶点直线面积会忽略 bulge，必须按 bulge 采样后才代表真实形状
            double areaSamp = Math.Abs(PolygonArea(SampleBulgePolyline(envFit, bulges, 32)));
            double maxGap = 0;
            for (int i = 0; i < envFit.Count; i++)
            {
                var d0 = envFit[(i + 1) % envFit.Count] - envFit[i];
                double g = Math.Sqrt(d0.X * d0.X + d0.Y * d0.Y);
                if (g > maxGap) maxGap = g;
            }
            double maxGapRaw = 0;
            for (int i = 0; i < envRaw.Count; i++)
            {
                var d0 = envRaw[(i + 1) % envRaw.Count] - envRaw[i];
                double g = Math.Sqrt(d0.X * d0.X + d0.Y * d0.Y);
                if (g > maxGapRaw) maxGapRaw = g;
            }
            Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                "       [bulge] arcs={0} |bulge|max={1:F4} ang={2:F2}~{3:F2}deg over180={4} area_raw={5:F3} area_sampled={6:F3} maxgap_raw={7:F4} maxgap_fit={8:F4}",
                nArc, maxAbs, nArc > 0 ? minA : 0, maxA, nOver180, areaRaw, areaSamp, maxGapRaw, maxGap));
        }

        /// <summary>临时诊断：打印折角最大的若干顶点，定位 bulge 拟合引入的折角来源。</summary>
        static void DumpSharpCorners(List<Point2d> sampled, List<Point2d> fitPts, List<double> bulges, int topN)
        {
            int n = sampled.Count;
            if (n < 3) return;
            var list = new List<(double ang, int idx)>();
            for (int i = 0; i < n; i++)
            {
                var prev = sampled[(i - 1 + n) % n];
                var cur = sampled[i];
                var next = sampled[(i + 1) % n];
                var v1 = cur - prev;
                var v2 = next - cur;
                if (v1.Length < 1e-9 || v2.Length < 1e-9) continue;
                double a = Math.Abs(Math.Atan2(v1.X * v2.Y - v1.Y * v2.X, v1.X * v2.X + v1.Y * v2.Y)) * 180.0 / Math.PI;
                if (a > 180.0) a = 360.0 - a;
                list.Add((a, i));
            }
            list.Sort((x, y) => y.ang.CompareTo(x.ang));
            Console.WriteLine("       [diag] fit_vertices=" + fitPts.Count + "  nonzero_bulge_idx=" +
                string.Join(",", Enumerable.Range(0, bulges.Count).Where(k => Math.Abs(bulges[k]) > 1e-6).Take(20)));
            foreach (var it in list.Take(topN))
            {
                var q = sampled[it.idx];
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "       [diag] turn={0,6:F2}deg at ({1,9:F4}, {2,9:F4})", it.ang, q.X, q.Y));
            }
        }

        // ================= 可视化（便于人工复核） =================

        /// <summary>
        /// 拟合前后对比图：原始并集边界（灰细线，密集折线）+ CAD 实际会画出的
        /// bulge 结果（黄粗线）。「包络还有没有锯齿」只能靠这张图肉眼确认，
        /// 数值不变量（顶点数、折角）都测不出「一段 3cm 台阶看起来扎不扎眼」。
        /// </summary>
        static void WriteSvgFit(string label, double turnDeg, double stepDeg,
                                List<Frame> frames, VehicleParams p, List<Point2d> env,
                                List<Point2d> fitPts, List<double> bulges)
        {
            var all = new List<Point2d>();
            foreach (var f in frames) all.AddRange(BodyCorners(f, p));
            if (all.Count == 0) return;

            double minX = all.Min(q => q.X), maxX = all.Max(q => q.X);
            double minY = all.Min(q => q.Y), maxY = all.Max(q => q.Y);
            double pad = 2.0;
            minX -= pad; maxX += pad; minY -= pad; maxY += pad;
            double w = maxX - minX, h = maxY - minY;
            double scale = 900.0 / Math.Max(w, h);

            Func<double, double> Sx = x => (x - minX) * scale;
            Func<double, double> Sy = y => (maxY - y) * scale;

            var sb = new StringBuilder();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<svg xmlns='http://www.w3.org/2000/svg' width='{0:F0}' height='{1:F0}' viewBox='0 0 {0:F0} {1:F0}'>",
                w * scale, h * scale);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<rect width='{0:F0}' height='{1:F0}' fill='white'/>", w * scale, h * scale);
            sb.AppendLine();
            int arcs = bulges == null ? 0 : bulges.Count(b => Math.Abs(b) > 1e-6);
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<text x='10' y='22' font-size='16' fill='#333'>{0} {1}deg  原始 {2} 顶点  →  CAD 出图 {3} 顶点 / {4} 弧</text>",
                label, turnDeg, env.Count, fitPts == null ? 0 : fitPts.Count, arcs);
            sb.AppendLine();

            foreach (var r in FootprintRects(frames, p))
            {
                var d = string.Join(" ", r.Select(q =>
                    string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
                sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#00AA00' stroke-width='0.5'/>", d);
                sb.AppendLine();
            }

            // 原始并集边界（灰，细）
            if (env != null && env.Count >= 3)
            {
                var d = string.Join(" ", env.Select(q =>
                    string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
                sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#999999' stroke-width='0.8'/>", d);
                sb.AppendLine();
            }

            // CAD 实际出图：bulge 结果按弧采样（蓝，粗）
            var sampled = SampleBulgePolyline(fitPts, bulges, 24);
            if (sampled.Count >= 3)
            {
                var d = string.Join(" ", sampled.Select(q =>
                    string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
                sb.AppendFormat("<polygon points='{0}' fill='rgba(255,193,7,0.18)' stroke='#D6A800' stroke-width='2.4'/>", d);
                sb.AppendLine();
                // 顶点标记：点太密就说明拟合没起作用
                foreach (var q in fitPts)
                    sb.AppendFormat(CultureInfo.InvariantCulture,
                        "<circle cx='{0:F2}' cy='{1:F2}' r='1.8' fill='#D60000'/>", Sx(q.X), Sy(q.Y));
                sb.AppendLine();
            }

            sb.AppendLine("</svg>");
            string name = string.Format(CultureInfo.InvariantCulture,
                "VerifyFit_{0}_{1:F0}deg_step{2:F2}.svg", label, turnDeg, stepDeg);
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, name);
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        static int RunTruckPreviewKeyframes()
        {
            Console.WriteLine("\n-- v5.2.2：货车实时预览关键帧与正式出图隔离 --");
            var p = VehicleParams.Defaults();
            p.UnitScale = 1.0;
            p.Articulated = true;
            var frames = TruckKinematics.SimulateFromStateFront(
                p, new Point2d(0, 0), new Vector2d(1, 0), 0.0,
                +1, 170.0, 80.0, +1, 1.0, null,
                TruckKinematics.PreviewToothBudgetM);
            var envelopeFrames = TruckKinematics.PreviewFrames(frames, 320);
            var trackFrames = TruckKinematics.PreviewFrames(frames, 64);
            int fail = 0;

            if (frames.Count <= 320 || envelopeFrames.Count > 320 || trackFrames.Count > 64)
                fail++;
            if (envelopeFrames.Count < 2 ||
                !object.ReferenceEquals(envelopeFrames[0], frames[0]) ||
                !object.ReferenceEquals(envelopeFrames[envelopeFrames.Count - 1], frames[frames.Count - 1]))
                fail++;

            var previewEnvelope = TruckKinematics.EnvelopeTracks(envelopeFrames, p);
            if (TruckKinematics.EnvelopeFallbackUsed || previewEnvelope == null || previewEnvelope.Count < 3)
                fail++;

            double maximumMiss = 0.0;
            if (previewEnvelope != null && previewEnvelope.Count >= 3)
            {
                foreach (var f in frames)
                foreach (var q in TruckKinematics.Corners(f, p))
                {
                    if (GeometryUtil.PointInClosedPolygon(q, previewEnvelope)) continue;
                    double nearest = double.MaxValue;
                    for (int i = 0; i < previewEnvelope.Count; i++)
                    {
                        var a = previewEnvelope[i];
                        var b = previewEnvelope[(i + 1) % previewEnvelope.Count];
                        double d = GeometryUtil.DistancePointToSegment(q, a, b).dist;
                        if (d < nearest) nearest = d;
                    }
                    if (nearest > maximumMiss) maximumMiss = nearest;
                }
                if (maximumMiss > 0.15) fail++;
            }

            // 模拟 TRUCKDRIVE 分两段确认：每段独立缓存 footprint，段间首帧重复。
            // 缓存合并后的精确包络必须与直接由全部帧计算完全等价。
            int middle = frames.Count / 2;
            var first = frames.GetRange(0, middle + 1);
            var second = frames.GetRange(middle, frames.Count - middle);
            var cachedFootprints = TruckKinematics.BuildEnvelopeFootprints(first, p);
            cachedFootprints.AddRange(
                TruckKinematics.BuildEnvelopeFootprints(second, p));
            List<Point2d> cachedEnvelope =
                TruckKinematics.EnvelopeTracksFromFootprints(cachedFootprints);
            bool cachedFallback = TruckKinematics.EnvelopeFallbackUsed;
            List<Point2d> exactEnvelope = TruckKinematics.EnvelopeTracks(frames, p);
            bool exactFallback = TruckKinematics.EnvelopeFallbackUsed;
            double cacheAreaDelta = Math.Abs(
                PolygonArea(cachedEnvelope) - PolygonArea(exactEnvelope));
            if (cachedFallback || exactFallback ||
                cacheAreaDelta > 1e-8 * Math.Max(1.0, PolygonArea(exactEnvelope)))
                fail++;

            Console.WriteLine("  全量 {0} 帧 → 包络 {1} 帧 / 轮迹 {2} 帧；最大临时漏差 {3:F3}m；缓存面积差 {4:E2}  {5}",
                frames.Count, envelopeFrames.Count, trackFrames.Count, maximumMiss,
                cacheAreaDelta, fail == 0 ? "OK" : "FAIL");
            return fail;
        }

        static void RunTruckPreviewPerformance()
        {
            var p = VehicleParams.Defaults();
            p.UnitScale = 1.0;
            p.Articulated = true;
            var frames = TruckKinematics.SimulateFromStateFront(
                p, new Point2d(0, 0), new Vector2d(1, 0), 0.0,
                +1, 170.0, 80.0, +1, 1.0, null,
                TruckKinematics.PreviewToothBudgetM);
            Console.WriteLine("\n-- 货车实时包络性能 --");
            Console.WriteLine("路径帧数：{0}", frames.Count);
            foreach (int cap in new[] { frames.Count, 320, 240, 160, 128 })
            {
                var selected = TruckKinematics.PreviewFrames(frames, cap);
                const int loops = 3;
                var sw = Stopwatch.StartNew();
                int points = 0;
                for (int i = 0; i < loops; i++)
                {
                    var env = TruckKinematics.EnvelopeTracks(selected, p);
                    var simplified = GeometryUtil.SimplifyEnvelopeForSpline(env, 0.05);
                    points = simplified.loop == null ? 0 : simplified.loop.Count;
                }
                sw.Stop();
                Console.WriteLine("  cap={0,4} actual={1,4}  {2,7:F1} ms/次  点={3}",
                    cap, selected.Count, sw.Elapsed.TotalMilliseconds / loops, points);
            }
        }

        static void RunTruckLongEnvelopePerformance()
        {
            var p = VehicleParams.Defaults();
            p.UnitScale = 1.0;
            p.Articulated = true;
            var frames = TruckKinematics.SimulateFromStateFront(
                p, new Point2d(0, 0), new Vector2d(1, 0), 0.0,
                +1, 170.0, 160.0, +1, TruckKinematics.FineStepDeg, null,
                TruckKinematics.StraightToothBudgetM);
            List<Point2d[]> footprints =
                TruckKinematics.BuildEnvelopeFootprints(frames, p);
            Console.WriteLine("\n-- 铰接车长路径最终包络性能 --");
            Console.WriteLine("帧={0}，footprint={1}", frames.Count, footprints.Count);

            var sw = Stopwatch.StartNew();
            List<Point2d> legacy = TruckKinematics.UnionEnvelopeFromConvexPolygons(
                footprints, out bool oldFallback, false);
            sw.Stop();
            double oldMs = sw.Elapsed.TotalMilliseconds;

            sw.Restart();
            List<Point2d> indexed = TruckKinematics.UnionEnvelopeFromConvexPolygons(
                footprints, out bool newFallback, true);
            sw.Stop();
            double indexedMs = sw.Elapsed.TotalMilliseconds;
            double areaOld = PolygonArea(legacy);
            double areaNew = PolygonArea(indexed);
            double areaDelta = Math.Abs(areaOld - areaNew);
            Console.WriteLine(
                "旧扫描 {0:F1} ms；空间索引 {1:F1} ms；加速 {2:F2}x；面积差 {3:E3}",
                oldMs, indexedMs, oldMs / Math.Max(indexedMs, 1e-9), areaDelta);
            AssertTrue(!oldFallback && !newFallback, "长路径包络不得回退凸包");
            AssertTrue(areaDelta <= 1e-6 * Math.Max(1.0, areaOld),
                "空间索引不得改变最终包络面积");
        }

        static void WriteSvg(string label, double turnDeg, double stepDeg,
                             List<Frame> frames, VehicleParams p, List<Point2d> env)
        {
            var all = new List<Point2d>();
            foreach (var f in frames) all.AddRange(BodyCorners(f, p));
            if (all.Count == 0) return;

            double minX = all.Min(q => q.X), maxX = all.Max(q => q.X);
            double minY = all.Min(q => q.Y), maxY = all.Max(q => q.Y);
            double pad = 2.0;
            minX -= pad; maxX += pad; minY -= pad; maxY += pad;
            double w = maxX - minX, h = maxY - minY;
            double scale = 900.0 / Math.Max(w, h);

            Func<double, double> Sx = x => (x - minX) * scale;
            Func<double, double> Sy = y => (maxY - y) * scale;

            var sb = new StringBuilder();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<svg xmlns='http://www.w3.org/2000/svg' width='{0:F0}' height='{1:F0}' viewBox='0 0 {0:F0} {1:F0}'>",
                w * scale, h * scale);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<rect width='{0:F0}' height='{1:F0}' fill='white'/>", w * scale, h * scale);
            sb.AppendLine();
            sb.AppendFormat(CultureInfo.InvariantCulture,
                "<text x='10' y='22' font-size='16' fill='#333'>{0} {1}deg step={2} env_pts={3}</text>",
                label, turnDeg, stepDeg, env.Count);
            sb.AppendLine();

            // 车体 footprint
            foreach (var r in FootprintRects(frames, p))
            {
                var d = string.Join(" ", r.Select(q =>
                    string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
                sb.AppendFormat("<polygon points='{0}' fill='none' stroke='#00AA00' stroke-width='0.6'/>", d);
                sb.AppendLine();
            }

            // 包络（黄，闭合）
            if (env != null && env.Count >= 3)
            {
                var d = string.Join(" ", env.Select(q =>
                    string.Format(CultureInfo.InvariantCulture, "{0:F2},{1:F2}", Sx(q.X), Sy(q.Y))));
                sb.AppendFormat("<polygon points='{0}' fill='rgba(255,193,7,0.25)' stroke='#D6A800' stroke-width='2'/>", d);
                sb.AppendLine();
            }

            sb.AppendLine("</svg>");

            string name = string.Format(CultureInfo.InvariantCulture,
                "Verify_{0}_{1:F0}deg_step{2:F2}.svg", label, turnDeg, stepDeg);
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, name);
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }
    }
}
