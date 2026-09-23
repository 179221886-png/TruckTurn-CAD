using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using GI = Autodesk.AutoCAD.GraphicsInterface;

namespace TruckTurn
{
    /// <summary>WorldDraw 临时图形绘制助手（Jig 预览用，不产生图元）</summary>
    internal static class JigDraw
    {
        public static void Poly(GI.WorldDraw draw, IList<Point2d> pts, bool closed)
        {
            if (pts == null || pts.Count < 2) return;
            for (int i = 0; i + 1 < pts.Count; i++)
                draw.Geometry.WorldLine(P3(pts[i]), P3(pts[i + 1]));
            if (closed && pts.Count > 2)
                draw.Geometry.WorldLine(P3(pts[pts.Count - 1]), P3(pts[0]));
        }

        public static void Rect(GI.WorldDraw draw, Point2d[] r)
        {
            if (r == null || r.Length < 4) return;
            for (int i = 0; i < 4; i++)
                draw.Geometry.WorldLine(P3(r[i]), P3(r[(i + 1) % 4]));
        }

        /// <summary>画出一帧车体（受 p 显示选项控制）</summary>
        public static void Truck(GI.WorldDraw draw, Frame f, VehicleParams p, int steerSign)
        {
            if (p.ShowBody)
            {
                if (p.Articulated)
                {
                    Rect(draw, TruckKinematics.TractorRect(f, p));
                    // v4.4：驾驶室后方的窄车架
                    var chassis = TruckKinematics.ChassisRect(f, p);
                    if (chassis != null) Rect(draw, chassis);
                    Rect(draw, TruckKinematics.TrailerRect(f, p));
                }
                else
                {
                    // v4.9.17：刚性车改画精致轮廓（车头斜面+挡风玻璃+后视镜+车厢）
                    foreach (var seg in TruckKinematics.RigidBodyOutlines(f, p))
                        Poly(draw, seg, false);
                }
                draw.Geometry.WorldLine(P3(f.O1), P3(f.K));   // 后轴到前轴轴线
                if (p.Articulated)
                    draw.Geometry.WorldLine(P3(f.K), P3(f.O2));   // 挂车轴线
            }

            // 轮胎外观（v4.0）
            if (p.ShowWheels)
            {
                double tireLen = 0.5 * p.UnitScale;
                double tireWidth = 0.25 * p.UnitScale;
                foreach (var rect in TruckKinematics.TireRects(f, p, tireLen, tireWidth, steerSign))
                    Rect(draw, rect);
            }
        }

        /// <summary>绘制轮迹/前轴轨迹预览（受 p 显示选项控制）</summary>
        public static void WheelTracks(GI.WorldDraw draw, List<Frame> frames, VehicleParams p)
        {
            DrawWheelTracks(draw, BuildWheelTracks(frames, p), p);
        }

        /// <summary>把轮迹点集与 WorldDraw 分离，供连续驾驶 Jig 按运动帧引用缓存。</summary>
        public static List<Point2d>[] BuildWheelTracks(List<Frame> frames, VehicleParams p)
        {
            if (frames == null || frames.Count < 2 ||
                (!p.ShowWheels && !p.ShowFrontTrack)) return null;

            var tracks = new List<Point2d>[7];
            for (int i = 0; i < tracks.Length; i++) tracks[i] = new List<Point2d>();
            double half = p.WheelTrack / 2.0;
            foreach (var f in frames)
            {
                Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);
                Vector2d n2 = new Vector2d(-f.U2.Y, f.U2.X);
                Point2d frontAxle = f.O1 + p.TractorWheelbase * f.U1;
                tracks[0].Add(frontAxle);
                tracks[1].Add(frontAxle + half * n1);
                tracks[2].Add(frontAxle - half * n1);
                tracks[3].Add(f.O1 + half * n1);
                tracks[4].Add(f.O1 - half * n1);
                if (p.Articulated)
                {
                    tracks[5].Add(f.O2 + half * n2);
                    tracks[6].Add(f.O2 - half * n2);
                }
            }
            return tracks;
        }

        public static void DrawWheelTracks(
            GI.WorldDraw draw, List<Point2d>[] tracks, VehicleParams p)
        {
            if (tracks == null || tracks.Length < 7) return;

            if (p.ShowFrontTrack)
            {
                SetColor(draw, 6);
                Poly(draw, tracks[0], false);
            }

            if (p.ShowWheels)
            {
                SetColor(draw, 1);
                Poly(draw, tracks[1], false); Poly(draw, tracks[2], false);
                Poly(draw, tracks[3], false); Poly(draw, tracks[4], false);
                if (p.Articulated)
                {
                    Poly(draw, tracks[5], false); Poly(draw, tracks[6], false);
                }
            }
        }

        /// <summary>
        /// 绘制扫掠包络边界（黄色闭合线，受 p 显示选项控制）。
        /// v4.9.1：可选缓存参数 —— 传入 _cachedFrames 引用若未变，且 _cachedHash
        ///   对得上，则完全跳过 EnvelopeTracks + FitEnvelopeBulges + SampleBulgePolyline
        ///   整条链。鼠标每动一下，CAD 都调一次 WorldDraw；不缓存时
        ///   91 帧铰接车的 EnvelopeTracks 要跑 ~10ms，60Hz 鼠标 = 600ms/秒 CPU，
        ///   GstarCAD 直接"卡死"（v4.9 阶段 4 报告）。加缓存后只画 1 帧车体，
        ///   缓存命中时 CPU 占比从 100% 降到 < 5%。
        /// 缓存 key 用 (frames, ShowEnvelope) 的复合哈希。
        /// 旧签名（不传缓存参数）保留兼容：所有缓存 ref 都是 null，重算（行为不变）。
        /// </summary>
        public static void EnvelopeTracks(GI.WorldDraw draw, List<Frame> frames, VehicleParams p)
        {
            List<Frame> cf = null; int ch = 0; List<Point2d> cp = null; List<double> cb = null; bool ce = true;
            EnvelopeTracks(draw, frames, p, cf, ref ch, ref cp, ref cb, ref ce);
        }

        public static void EnvelopeTracks(GI.WorldDraw draw, List<Frame> frames, VehicleParams p,
                                          List<Frame> cachedFrames, ref int cachedHash,
                                          ref List<Point2d> cachedPts, ref List<double> cachedBulges,
                                          ref bool cachedShowEnvelope)
        {
            if (!p.ShowEnvelope || frames == null || frames.Count < 2) return;
            int hash = unchecked(frames.Count * 397) ^ (p.ShowEnvelope ? 1 : 0);
            if (cachedFrames == frames && cachedHash == hash && cachedPts != null
                && cachedShowEnvelope == p.ShowEnvelope)
            {
                // 命中：直接画上次算好的简化环
                SetColor(draw, 2);
                Poly(draw, cachedPts, true);
                return;
            }

            // 缓存未命中时才运行最重的多边形并集。旧顺序在命中缓存前已经完成
            // EnvelopeTracks，实际只能省掉后续稀化，无法降低 CAD 重绘开销。
            var envelope = TruckKinematics.EnvelopeTracks(frames, p);
            if (envelope == null || envelope.Count < 3) return;

            // v4.9.7：预览改用与出图相同的「大容差 RDP + 外推」简化管线。
            //   之前 FitEnvelopeBulges 把 RAW envelope 上的「0.1mm/80mm 短边 + 90°」反射伪尖角
            //   （每帧车体角点位置几乎重合 + 「最近角点」在帧间切换留下 0.1mm 短边）
            //   误识别为圆弧段画成小圆凸起 —— 用户实机看到的「车头/车尾包络凸起」。
            //   大 RDP(5cm) 直接把近似共线段压成 2 个端点，35 顶点画 Polyline 已经够光滑，
            //   且与出图 Spline 的几何完全一致。
            var (simpPts, _) = GeometryUtil.SimplifyEnvelopeForSpline(envelope, 0.05);
            if (simpPts == null || simpPts.Count < 3) return;

            SetColor(draw, 2);
            Poly(draw, simpPts, true);

            // 更新缓存（让下一次 WorldDraw 命中）
            cachedPts = simpPts;
            cachedBulges = null;         // 不再用 bulge，置空即可
            cachedHash = hash;
            cachedShowEnvelope = p.ShowEnvelope;
        }

        public static Point3d P3(Point2d p) { return new Point3d(p.X, p.Y, 0.0); }

        public static void SetColor(GI.WorldDraw draw, short aci)
        {
            try { draw.SubEntityTraits.Color = aci; } catch { }
        }
    }

    /// <summary>
    /// 阶段一：货车外形跟随光标平移，左键点击确定起始位置（前轴中心）。
    /// </summary>
    public class TruckPlaceJig : DrawJig
    {
        private readonly VehicleParams _p;
        private readonly Vector2d _heading;
        private Point3d _cur;
        public Point2d Result { get { return new Point2d(_cur.X, _cur.Y); } }

        public TruckPlaceJig(VehicleParams p, Vector2d heading, Point3d seed)
        {
            _p = p;
            _heading = heading.GetNormal();
            _cur = seed;
        }

        // v4.9.3：阶段 1（点初始位置）若 Sampler/WorldDraw 抛异常，浩辰 Jig 状态机
        //   不会返回 → ed.Drag 永久挂起 = "CAD 卡死，必须任务管理器杀"。
        //   v4.9.1 已经给阶段 4 (TruckDriveJig) 加了同样的保护，但阶段 1+2 当时漏了。
        //   现在补上：异常时打印完整堆栈，返回 Cancel/false 让 Jig 正常退出。
        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            try { return SamplerCore(prompts); }
            catch (System.Exception ex)
            {
                try { System.Console.WriteLine("[TRUCKDRIVE 阶段1 Sampler 异常] {0}: {1}\n{2}",
                    ex.GetType().FullName, ex.Message, ex.StackTrace); } catch { }
                return SamplerStatus.Cancel;
            }
        }
        private SamplerStatus SamplerCore(JigPrompts prompts)
        {
            var opt = new JigPromptPointOptions("\n指定货车位置(前轴中心)，鼠标移动预览车形");
            opt.UserInputControls = UserInputControls.Accept3dCoordinates
                                  | UserInputControls.NullResponseAccepted
                                  | UserInputControls.AcceptMouseUpAsPoint;

            var res = prompts.AcquirePoint(opt);
            if (res.Status != PromptStatus.OK) return SamplerStatus.Cancel;
            if (res.Value.DistanceTo(_cur) < 1e-8) return SamplerStatus.NoChange;

            _cur = res.Value;
            return SamplerStatus.OK;
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            try { return WorldDrawCore(draw); }
            catch (System.Exception ex)
            {
                try { System.Console.WriteLine("[TRUCKDRIVE 阶段1 WorldDraw 异常] {0}: {1}\n{2}",
                    ex.GetType().FullName, ex.Message, ex.StackTrace); } catch { }
                return false;
            }
        }
        private bool WorldDrawCore(GI.WorldDraw draw)
        {
            Point2d rear = Result - _p.TractorWheelbase * _heading;
            var frames = TruckKinematics.Simulate(_p, rear, _heading, 1, 0.0, 0.0, 1.0);
            if (frames.Count == 0) return true;

            JigDraw.SetColor(draw, 3);
            JigDraw.Truck(draw, frames[0], _p, 0);

            // 车头方向指示箭头（从前轴中心向前）
            JigDraw.SetColor(draw, 1);
            var f = frames[0];
            Point2d frontAxle = TruckKinematics.FrontAxle(f, _p);
            double len = _p.TractorFrontOverhang;
            Point2d nose = frontAxle + (len + _p.TractorWidth * 0.6) * f.U1;
            Point2d tip = frontAxle + len * f.U1;
            Vector2d n = new Vector2d(-f.U1.Y, f.U1.X);
            draw.Geometry.WorldLine(JigDraw.P3(tip), JigDraw.P3(nose));
            draw.Geometry.WorldLine(JigDraw.P3(nose), JigDraw.P3(tip + (_p.TractorWidth * 0.25) * n));
            draw.Geometry.WorldLine(JigDraw.P3(nose), JigDraw.P3(tip - (_p.TractorWidth * 0.25) * n));
            return true;
        }
    }

    /// <summary>
    /// 阶段二：起点（前轴中心）已定，货车绕前轴中心旋转跟随光标，左键点击确定车头朝向。
    /// </summary>
    public class TruckHeadingJig : DrawJig
    {
        private readonly VehicleParams _p;
        private readonly Point2d _front;
        private Point3d _cur;
        private Vector2d _heading;
        public Vector2d Result { get { return _heading; } }

        public TruckHeadingJig(VehicleParams p, Point2d front, Vector2d seedHeading)
        {
            _p = p;
            _front = front;
            _heading = seedHeading.GetNormal();
            double arrowLen = Math.Max(1.0, _p.TractorFrontOverhang + _p.TractorWheelbase * 0.5);
            _cur = new Point3d(front.X + _heading.X * arrowLen, front.Y + _heading.Y * arrowLen, 0.0);
        }

        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            var opt = new JigPromptPointOptions("\n指定车头朝向(移动鼠标旋转货车)");
            opt.BasePoint = JigDraw.P3(_front);
            opt.UseBasePoint = true;
            opt.UserInputControls = UserInputControls.Accept3dCoordinates
                                  | UserInputControls.AcceptMouseUpAsPoint;

            var res = prompts.AcquirePoint(opt);
            if (res.Status != PromptStatus.OK) return SamplerStatus.Cancel;
            if (res.Value.DistanceTo(_cur) < 1e-8) return SamplerStatus.NoChange;

            _cur = res.Value;
            var v = new Vector2d(_cur.X - _front.X, _cur.Y - _front.Y);
            if (v.Length > 1e-6) _heading = v.GetNormal();
            return SamplerStatus.OK;
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            Point2d rear = _front - _p.TractorWheelbase * _heading;
            var frames = TruckKinematics.Simulate(_p, rear, _heading, 1, 0.0, 0.0, 1.0);
            if (frames.Count == 0) return true;

            JigDraw.SetColor(draw, 3);
            JigDraw.Truck(draw, frames[0], _p, 0);

            JigDraw.SetColor(draw, 1);
            draw.Geometry.WorldLine(JigDraw.P3(_front), _cur);
            return true;
        }
    }

    /// <summary>
    /// 阶段三：起点/朝向/转向已定，拖动光标确定货车运动结束位置（前轴中心）。
    /// 由光标点几何反解（圆弧 + 切线直线），实时预览整条行驶轨迹、扫掠包络与轮迹。
    /// </summary>
    public class TruckPathJig : DrawJig
    {
        private readonly VehicleParams _p;
        private readonly Point2d _startFront;
        private readonly Vector2d _heading;
        private readonly bool _autoSide;

        private int _steerSign;
        private Point3d _cur;
        private double _turnDeg;
        private double _straight;
        private double? _rearRadiusOverride;

        public double TurnAngleDeg { get { return _turnDeg; } }
        public double StraightLen { get { return _straight; } }
        public int SteerSign { get { return _steerSign; } }
        public double? RearRadiusOverride { get { return _rearRadiusOverride; } }

        private const double PreviewStep = 1.0;
        private const int GhostEvery = 8;

        public TruckPathJig(VehicleParams p, Point2d startFront, Vector2d heading, int steerSign, bool autoSide)
        {
            _p = p;
            _startFront = startFront;
            _heading = heading.GetNormal();
            _steerSign = steerSign >= 0 ? 1 : -1;
            _autoSide = autoSide;
            _cur = JigDraw.P3(startFront);
        }

        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            string side = _steerSign > 0 ? "左" : "右";
            string msg = string.Format("\n指定货车结束位置(前轴中心) [{0}转 转弯{1:F1}° 直行{2:F0}]",
                                       side, _turnDeg, _straight);
            var opt = new JigPromptPointOptions(msg);
            opt.BasePoint = JigDraw.P3(_startFront);
            opt.UseBasePoint = true;
            opt.UserInputControls = UserInputControls.Accept3dCoordinates
                                  | UserInputControls.AcceptMouseUpAsPoint;

            var res = prompts.AcquirePoint(opt);
            if (res.Status != PromptStatus.OK) return SamplerStatus.Cancel;
            if (res.Value.DistanceTo(_cur) < 1e-8) return SamplerStatus.NoChange;

            _cur = res.Value;
            var target = new Point2d(_cur.X, _cur.Y);

            if (_autoSide)
            {
                Vector2d d = target - _startFront;
                double cross = _heading.X * d.Y - _heading.Y * d.X;
                if (Math.Abs(cross) > 1e-9) _steerSign = cross > 0 ? 1 : -1;
            }

            // v4.1/v4.2：优先用自适应单圆弧（过起点-目标点的圆），不可行则按最小半径转 90°
            if (!TruckKinematics.TrySolveFromTargetFrontAdaptive(_p, _startFront, _heading, _steerSign,
                                                                 target, out _turnDeg, out _straight, out double rearR))
            {
                _rearRadiusOverride = null;
                // v4.7：同 TruckDriveJig —— 目标太近时不再硬跳 90°，改为沿最小半径圆
                // 把车头尽量转向光标，转角随光标连续变化。
                _turnDeg = TruckKinematics.TurnTowardTargetAtMinRadius(_p, _startFront, _heading, _steerSign, target);
                _straight = 0.0;
            }
            else
            {
                _rearRadiusOverride = rearR;
            }
            return SamplerStatus.OK;
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            // v4.9.14：预览用放宽的齿深预算（PreviewToothBudgetM），帧数 ~5×↓，
            // 避免残留铰接角段精采样导致每次鼠标移动都重算大包络而卡顿；出图仍用 0.02m 精采样。
            var frames = TruckKinematics.SimulateFromFrontAxle(_p, _startFront, _heading, _steerSign,
                                                               _turnDeg, _straight, PreviewStep, _rearRadiusOverride,
                                                               TruckKinematics.PreviewToothBudgetM);
            if (frames.Count == 0) return true;

            // 自适应成功时才让末帧精确对齐光标；最小半径回退时不强移（避免车身平移）
            if (_rearRadiusOverride.HasValue)
            {
                var last = frames[frames.Count - 1];
                frames[frames.Count - 1] = last.WithFrontAt(_p, new Point2d(_cur.X, _cur.Y));
            }

            // 扫掠包络左右边界 + 轮迹
            JigDraw.EnvelopeTracks(draw, frames, _p);
            JigDraw.WheelTracks(draw, frames, _p);

            // 中间姿态（幽灵车）
            JigDraw.SetColor(draw, 8);
            int[] signs = TruckKinematics.ComputeSteerSigns(frames);
            for (int i = GhostEvery; i < frames.Count - 1; i += GhostEvery)
                JigDraw.Truck(draw, frames[i], _p, signs[i]);

            // 起始与结束姿态
            JigDraw.SetColor(draw, 3);
            JigDraw.Truck(draw, frames[0], _p, _steerSign);
            JigDraw.Truck(draw, frames[frames.Count - 1], _p, _steerSign);
            return true;
        }
    }

    /// <summary>
    /// 连续驾驶 Jig：光标处实时显示从上一帧到当前光标点的货车图块、包络与轮迹。
    /// 每一点击确认当前段，上一帧更新为当前段末帧。
    /// </summary>
    public class TruckDriveJig : DrawJig
    {
        private readonly VehicleParams _p;
        private readonly Frame _startFrame;
        private int _steerSign;
        private readonly bool _autoSide;
        private readonly int _dir;               // v4.9：+1 前进 / −1 倒退
        private Point3d _cur;
        private double _turnDeg;
        private double _straight;
        private double? _rearRadiusOverride;
        private int _reverseSide = 1;            // v4.9.9：倒退车尾甩向（+1 左 / −1 右）
        private double _reverseRadius;           // v4.9.9：倒退车尾控制点半径 R2
        private List<Frame> _previewFrames;
        private bool _valid;
        private string _invalidHint = "";   // v4.9.21：倒退不可达原因（在提示行显示）
        // v4.9.19：漂移守卫 —— 记录最后一次 WorldDraw 实际画出的预览终点姿态。
        // 点击瞬间 SamplerCore 会按落点重算解（覆盖 _previewFrames），但 WorldDraw
        // 不会再跑 —— 用户确认的是他从未见过的解。在回退分支陡变区（实测极角 95°
        // dist 12→16m 扫掠角 20°→128°）或对象捕捉跳点时，落点解与所见预览能差出
        // 半圈（「预览小转角、确认绕大半圈」，v4.9.16 之后的新形态）。
        // 主循环确认前调 ConfirmMatchesDrawn 比对，不一致就只更新预览不确认。
        private Point2d _drawnEndFront;
        private Vector2d _drawnEndU1;
        private bool _hasDrawnPreview;
        private string _kwTurn90;
        private string _kw;              // v4.6：U=撤销上一段，X=结束并出图；v4.9：B=切换前进/倒退

        public List<Frame> SegmentFrames { get { return _previewFrames; } }
        public double TurnAngleDeg { get { return _turnDeg; } }
        public double StraightLen { get { return _straight; } }
        public int SteerSign { get { return _steerSign; } }
        public double? RearRadiusOverride { get { return _rearRadiusOverride; } }
        /// <summary>v4.9.9：倒退段的车尾甩向与半径（确认段重算用）。</summary>
        public int ReverseSide { get { return _reverseSide; } }
        public double ReverseRadius { get { return _reverseRadius; } }
        /// <summary>Sampler 内通过关键字 L/R 触发的 90° 转弯请求，主循环读取后清空。</summary>
        public string KwTurn90 { get { return _kwTurn90; } }
        /// <summary>v4.6：Sampler 内通过关键字 U/X 触发的「撤销上一段 / 结束出图」请求；v4.9：B=切换前进/倒退。</summary>
        public string Kw { get { return _kw; } }
        /// <summary>v4.9：当前行驶方向：+1 前进 / −1 倒退。</summary>
        public int Dir { get { return _dir; } }

        /// <summary>
        /// v4.9.19 漂移守卫：当前（落点重算后的）解是否与最后一次实际画出的预览一致。
        /// 终点位置差 &gt;0.5m 或航向差 &gt;10° 判为漂移。无已画预览（首帧/上一位置无解）
        /// 时不拦截（保持原行为）。
        /// </summary>
        public bool ConfirmMatchesDrawn(out string why)
        {
            why = null;
            if (!_hasDrawnPreview || _previewFrames == null || _previewFrames.Count == 0) return true;
            var lf = _previewFrames[_previewFrames.Count - 1];
            Point2d endF = TruckKinematics.FrontAxle(lf, _p);
            double dx = endF.X - _drawnEndFront.X, dy = endF.Y - _drawnEndFront.Y;
            double dist = Math.Sqrt(dx * dx + dy * dy);
            double ang = Math.Abs(Math.Atan2(
                _drawnEndU1.X * lf.U1.Y - _drawnEndU1.Y * lf.U1.X,
                _drawnEndU1.X * lf.U1.X + _drawnEndU1.Y * lf.U1.Y)) * 180.0 / Math.PI;
            if (dist > 0.5 || ang > 10.0)
            {
                why = string.Format("终点差 {0:F1}m / 航向差 {1:F0}°", dist, ang);
                return false;
            }
            return true;
        }

        private const double PreviewStep = 1.0; // v4.3.1：预览步长从 4°降到 1°，包络更顺滑
        // 仅作用于鼠标移动时的临时图形；确认后 Commands 仍以 0.25° 和完整帧正式出图。
        private const int PreviewEnvelopeFrameLimit = 320;
        private const int PreviewTrackFrameLimit = 64;

        // v4.9.1：上一帧 frames 的引用。WorldDraw 在 EnvelopeTracks 上做了缓存：
        //   frames 对象引用没变 + ShowEnvelope 没变 ⇒ 跳过整条 EnvelopeTracks 链
        //   （BuildFootprintRects/ExposedBoundarySegments/ChainBoundaryLoops/FitEnvelopeBulges/SampleBulgePolyline），
        //   鼠标快速移动时只重画 1 帧车体（< 1ms），把 CPU 从 100% 拉回 5% 以下。
        //   旧逻辑每次 Sampler 都全跑一遍：91 帧 × 273 矩形 × 273 矩形 ≈ 30 万次
        //   Cyrus-Beck，单次约 10ms，60Hz 鼠标 = 600ms/秒 CPU，GstarCAD 直接"卡死"。
        private List<Frame> _cachedEnvFrames = null;
        private int _cachedEnvHash = 0;
        private List<Point2d> _cachedEnvPts = null;
        private List<double> _cachedEnvBulges = null;
        private bool _cachedEnvShowEnvelope = true;
        private List<Frame> _cachedPreviewSource = null;
        private List<Frame> _cachedPreviewEnvelopeFrames = null;
        private List<Point2d>[] _cachedPreviewTracks = null;

        public TruckDriveJig(VehicleParams p, Frame startFrame, int steerSign, bool autoSide, int dir = 1)
        {
            _p = p;
            _startFrame = startFrame;
            _steerSign = steerSign >= 0 ? 1 : -1;
            _autoSide = autoSide;
            _dir = dir >= 0 ? 1 : -1;
            _cur = JigDraw.P3(TruckKinematics.FrontAxle(startFrame, p));
            _previewFrames = new List<Frame>();
        }

        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            // v4.9.1：v4.9 起 CAD 报"TRUCKDRIVE 直接卡死，必须任务管理器杀"。
            //   根因之一是 Jig 状态机被坏值卡死：Sampler 或 WorldDraw 抛异常后，
            //   GstarCAD 的 Jig 状态机没能安全返回，整个命令线程死锁。
            //   修法：Sampler/WorldDraw 外层都包 try-catch，任何异常走
            //   Cancel / false 路径，让 Jig 自然退出，主循环的 SafeDrag 兜底接住。
            try
            {
                return SamplerCore(prompts);
            }
            catch (System.Exception ex)
            {
                try { System.Console.WriteLine("[TRUCKDRIVE Sampler 异常] {0}: {1}", ex.GetType().Name, ex.Message); } catch { }
                _valid = false;
                _previewFrames = new List<Frame>();
                return SamplerStatus.Cancel;
            }
        }

        private SamplerStatus SamplerCore(JigPrompts prompts)
        {
            // v4.9.18：撤销只保留 Esc，删除 U 键入口（单一撤销方式）。
            // 右键/回车/其他非 OK 退出仍由主循环按「有段可撤就撤，已回到起点就结束」兜底。
            // 当前预览若走的是「最小半径回退」，光标点是走不到的，把这点写进提示
            // （_rearRadiusOverride 为空正是回退分支的标志；首帧 _valid=false 时不显示）
            string tight = (!_rearRadiusOverride.HasValue && _valid) ? "｜目标太近：按最小半径尽量转向" : "";
            // v4.9.9：倒退时显示「车尾甩向」（用户盯的是车尾），不再显示方向盘方向。
            string side = _dir < 0
                ? (_reverseSide > 0 ? "车尾左甩" : "车尾右甩")
                : (_steerSign > 0 ? "左" : "右");
            // v4.9：把当前行驶方向写进提示 —— 倒退的转向手感与前进相反（左打方向盘车往右后走），
            // 不给提示的话用户会以为预览算错了。
            string dirTxt = _dir > 0 ? "前进" : "倒退·车尾跟随光标";
            // v4.9.20：倒退甩尾角 >90° 时提示行醒目告警 —— 侧后方陡变区的大甩尾
            // 多数是无意点出来的（角度数字没人读），确认前必须让人意识到这是大半圈。
            string bigSwing = (_dir < 0 && _valid && _turnDeg > 90.0) ? "｜⚠ 大角度甩尾，确认前请先看预览" : "";
            string warn = bigSwing + _invalidHint;      // v4.9.21：告警 + 不可达原因
            string msg = _dir < 0
                ? string.Format(
                    "\n连续驾驶[{4}]：左键=确认这一段 [Esc]=撤销上一段 [X]=结束并出图 [B]=切换前进/倒退 [L/R]=车尾甩90°｜{0} {1:F1}° 直{2:F0}{3}{5}",
                    side, _turnDeg, _straight, tight, dirTxt, warn)
                : string.Format(
                    "\n连续驾驶[{4}]：左键=确认这一段 [Esc]=撤销上一段 [X]=结束并出图 [B]=切换前进/倒退 [L]=左转90° [R]=右转90°｜{0}转 转{1:F1}° 直{2:F0}{3}",
                    side, _turnDeg, _straight, tight, dirTxt);
            var opt = new JigPromptPointOptions(msg);
            opt.BasePoint = JigDraw.P3(TruckKinematics.FrontAxle(_startFrame, _p));
            opt.UseBasePoint = true;
            // v4.9.18：撤销只留 Esc，不再注册 U 关键字（也不设默认关键字——
            // 浩辰 2026 回车不落默认关键字，非 OK 退出统一由主循环兜底处理）。
            opt.Keywords.Add("X", "X", "结束并出图(X)");
            opt.Keywords.Add("B", "B", "切换前进/倒退(B)");
            opt.Keywords.Add("L", "L", "左转90°(L)");
            opt.Keywords.Add("R", "R", "右转90°(R)");
            // v4.7：去掉 GovernedByOrthoMode / GovernedByUCSDetect。
            // GovernedByOrthoMode 会让光标受正交(F8)约束：开着正交时，光标一靠近
            // 与车头垂直的方向就被吸到正 90° 上，正是用户说的「自动吸附过去，很不舒服」。
            // TRUCKDRIVE 要的是自由驾驶，光标该在哪就在哪，不受正交/动态 UCS 约束。
            opt.UserInputControls = UserInputControls.Accept3dCoordinates
                                  | UserInputControls.AcceptMouseUpAsPoint
                                  | UserInputControls.UseBasePointElevation;
            // 不开 NoNegativeResponseAccepted：关键字直接结束 Sampler，让主循环拿到后调用 90° 转弯
            opt.AppendKeywordsToMessage = true;

            var res = prompts.AcquirePoint(opt);
            // 用户输入关键字：返回 Cancel 但携带 StringResult，主循环识别后分派动作
            if (res.Status == PromptStatus.Keyword)
            {
                if (res.StringResult == "L" || res.StringResult == "R")
                {
                    _kwTurn90 = res.StringResult;
                    return SamplerStatus.Cancel;
                }
                // X = 结束并出图（v4.9.18 起 U 关键字已删除，撤销只走 Esc/非 OK 兜底）
                if (res.StringResult == "X")
                {
                    _kw = res.StringResult;
                    return SamplerStatus.Cancel;
                }
                // v4.9：B = 切换前进/倒退。方向是整段的属性，Jig 自己改不了（_dir 是只读字段），
                // 交回主循环：切换后新建 Jig 重新预览。
                if (res.StringResult == "B")
                {
                    _kw = res.StringResult;
                    return SamplerStatus.Cancel;
                }
            }
            // 其余非 OK 状态（右键 / 回车 / Esc）统一交回主循环：
            // 主循环按「有段可撤就撤，已回到起点就结束」处理。
            if (res.Status != PromptStatus.OK) return SamplerStatus.Cancel;
            if (res.Value.DistanceTo(_cur) < 1e-8) return SamplerStatus.NoChange;

            _cur = res.Value;
            var target = new Point2d(_cur.X, _cur.Y);

            // v4.9.9：倒退改走「车尾控制」管线 —— 挂车后轴（铰接）/ 后轴（刚性）精确跟随光标，
            // 铰接角按司机修正模型收敛，不再 jackknife；与前进的前轴控制完全分开。
            if (_dir < 0)
            {
                int rside;
                double rR;
                bool okR = TruckKinematics.TrySolveReverseArc(_p, _startFrame, target,
                                                              out rside, out _turnDeg, out rR, out _straight);
                if (okR)
                {
                    _rearRadiusOverride = rR;               // HasValue = 精确解标志
                }
                else
                {
                    _rearRadiusOverride = null;             // 最小半径回退标志
                    okR = TruckKinematics.ReverseTurnTowardAtMinRadius(_p, _startFrame, target,
                                                                       out rside, out _turnDeg, out rR);
                    _straight = 0.0;
                }
                if (!okR)
                {
                    // v4.9.21：不可达原因写进提示行（>90° 封顶 / 目标在车头前方），
                    // 否则预览突然消失用户不知道发生了什么。
                    _invalidHint = "｜该位置一段倒不过去（甩尾>90°或目标在车头前方），请分两段倒";
                    _valid = false;
                    _previewFrames = new List<Frame>();
                    return SamplerStatus.OK;
                }
                _invalidHint = "";
                _reverseSide = rside;
                _reverseRadius = rR;
                _steerSign = -rside;                        // 方向盘打向与车尾甩向相反（仅用于提示语）
                _valid = true;
                _previewFrames = TruckKinematics.SimulateReverseFromState(_p, _startFrame,
                                                                          rside, _turnDeg, _straight, rR, PreviewStep,
                                                                          TruckKinematics.PreviewToothBudgetM);
                return SamplerStatus.OK;
            }

            double startPhi2 = Math.Atan2(_startFrame.U2.Y, _startFrame.U2.X);
            Point2d segStartFront = TruckKinematics.FrontAxle(_startFrame, _p);
            // v4.1/v4.2：优先用自适应单圆弧（过起点-目标点的圆）；
            // v4.7：不可行（目标太近）时不再是固定 90°，而是沿最小半径圆尽量转向光标。
            // v4.9：自动判向时交给 TrySolveAutoSide —— 倒退时两侧都可能可行，需试两侧取转角小者。
            int steer = _steerSign;
            bool ok = _autoSide
                ? TruckKinematics.TrySolveAutoSide(_p, segStartFront, _startFrame.U1, _dir, target,
                                                   out steer, out _turnDeg, out _straight, out double rearR)
                : TruckKinematics.TrySolveFromStateToTargetFrontAdaptive(_p, segStartFront, _startFrame.U1,
                                                                         _steerSign, target, out _turnDeg,
                                                                         out _straight, out rearR, _dir);
            if (!_autoSide) steer = _steerSign;

            if (ok)
            {
                _rearRadiusOverride = rearR;
                _steerSign = steer;
            }
            else
            {
                // v4.7：目标太近（所需半径 < 最小转弯半径）时不再硬跳 90°。
                // 旧逻辑固定写 90°，与光标位置无关 —— 光标扫到车头侧面时预览突然吸住不动
                // （"自动吸附，很不舒服"）。改为沿最小半径圆把车头尽量转向光标：
                // 角度随光标连续变化，且在可行性边界上与自适应解完全衔接。
                _rearRadiusOverride = null;
                _turnDeg = TruckKinematics.TurnTowardTargetAtMinRadius(_p, segStartFront, _startFrame.U1, steer, target, _dir);
                _straight = 0.0;
                ok = _turnDeg > 1e-6;
                // v4.9.15 修复：自动判向 + 自适应解不可行（目标太近）时，预览用局部
                // steer（TrySolveAutoSide 失败也输出朝向光标的猜测侧）走最小半径回退，
                // 但 _steerSign 字段没同步 —— 确认段时 Commands 读 driveJig.SteerSign
                // 拿到残留的旧侧，画出的段与预览左右镜像（刚性车急弯最易踩中）。
                // 非自动判向时 steer == _steerSign，本赋值是 no-op。
                _steerSign = steer;
            }
            if (!ok || _turnDeg < -1e-9 || _straight < -1e-9)
            {
                _valid = false;
                _previewFrames = new List<Frame>();
                return SamplerStatus.OK;
            }
            _valid = true;
            _previewFrames = TruckKinematics.SimulateFromStateFront(_p, segStartFront,
                                                                     _startFrame.U1, startPhi2,
                                                                     steer, _turnDeg, _straight, _dir, PreviewStep, _rearRadiusOverride,
                                                                     TruckKinematics.PreviewToothBudgetM);
            // 自适应成功时才让末帧精确对齐光标；最小半径回退时不强移（避免车身平移）
            if (_previewFrames.Count > 0 && _rearRadiusOverride.HasValue)
            {
                var last = _previewFrames[_previewFrames.Count - 1];
                _previewFrames[_previewFrames.Count - 1] = last.WithFrontAt(_p, target);
            }
            return SamplerStatus.OK;
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            // v4.9.1：与 Sampler 同样的兜底 —— 任何异常返回 false 让 CAD 跳过这帧，
            //   避免坏值把 Jig 状态机卡死。真正要的"抑制 EnvelopeTracks 重算"
            //   见 JigDraw.EnvelopeTracks 的 _lastEnvFrameCount 缓存。
            try
            {
                return WorldDrawCore(draw);
            }
            catch (System.Exception ex)
            {
                try { System.Console.WriteLine("[TRUCKDRIVE WorldDraw 异常] {0}: {1}", ex.GetType().Name, ex.Message); } catch { }
                return false;
            }
        }

        private bool WorldDrawCore(GI.WorldDraw draw)
        {
            if (!_valid || _previewFrames == null || _previewFrames.Count == 0)
            {
                _hasDrawnPreview = false;   // v4.9.19：没有有效预览被画出
                JigDraw.SetColor(draw, 8);
                draw.Geometry.WorldLine(JigDraw.P3(TruckKinematics.FrontAxle(_startFrame, _p)), _cur);
                return true;
            }

            var frames = _previewFrames;
            // v4.9.19：记录本次实际画出的预览终点姿态（漂移守卫比对基准）
            {
                var dl = frames[frames.Count - 1];
                _drawnEndFront = TruckKinematics.FrontAxle(dl, _p);
                _drawnEndU1 = dl.U1;
                _hasDrawnPreview = true;
            }

            // 实时预览采用有限关键帧，并缓存派生包络/轮迹；正式出图仍使用完整 frames。
            // 同一 Sampler 结果被 CAD 重绘多次时，不再重复抽帧、并集和轮迹点集分配。
            if (!object.ReferenceEquals(_cachedPreviewSource, frames))
            {
                _cachedPreviewEnvelopeFrames =
                    TruckKinematics.PreviewFrames(frames, PreviewEnvelopeFrameLimit);
                List<Frame> trackFrames =
                    TruckKinematics.PreviewFrames(frames, PreviewTrackFrameLimit);
                _cachedPreviewTracks = JigDraw.BuildWheelTracks(trackFrames, _p);
                _cachedPreviewSource = frames;
            }
            JigDraw.EnvelopeTracks(draw, _cachedPreviewEnvelopeFrames, _p,
                                    _cachedEnvFrames, ref _cachedEnvHash,
                                    ref _cachedEnvPts, ref _cachedEnvBulges, ref _cachedEnvShowEnvelope);
            _cachedEnvFrames = _cachedPreviewEnvelopeFrames;
            JigDraw.DrawWheelTracks(draw, _cachedPreviewTracks, _p);

            // 光标处完整货车图块
            // v4.9：倒退用青色(4)、前进用绿色(3)，一眼能看出当前是哪一种，
            // 否则两种模式的车形完全一样，只有命令行文字在提示，切换后容易忘记自己在倒退。
            JigDraw.SetColor(draw, _dir > 0 ? (short)3 : (short)4);
            JigDraw.Truck(draw, frames[frames.Count - 1], _p, _steerSign);

            // v4.9：倒退时补一个"行驶方向"箭头 —— 沿 −U1 从车头指到车尾，
            // 倒车的转向手感与前进相反（左打方向盘车往右后走），没有方向指示很难判断预览对不对。
            if (_dir < 0 && frames.Count > 0)
            {
                var last = frames[frames.Count - 1];
                Point2d front = TruckKinematics.FrontAxle(last, _p);
                Vector2d n = new Vector2d(-last.U1.Y, last.U1.X);
                double ah = _p.TractorWidth * 0.35;                       // 箭头大小
                Point2d nose = front + (_p.TractorFrontOverhang + ah) * last.U1;   // 车头前尖
                Point2d tip = nose - (ah * 2.0) * last.U1;                // 箭头指向车尾
                JigDraw.SetColor(draw, 4);
                draw.Geometry.WorldLine(JigDraw.P3(nose), JigDraw.P3(tip));
                draw.Geometry.WorldLine(JigDraw.P3(tip), JigDraw.P3(tip + ah * last.U1 + ah * n));
                draw.Geometry.WorldLine(JigDraw.P3(tip), JigDraw.P3(tip + ah * last.U1 - ah * n));
            }

            // v4.7：走最小半径回退时车头到不了光标点（这是运动的物理限制，不是卡住）。
            // 补一条红线把「车头实际停在哪」和「光标在哪」连起来，
            // 否则预览看着像吸住不动，用户不知道发生了什么。
            if (!_rearRadiusOverride.HasValue)
            {
                JigDraw.SetColor(draw, 1);
                draw.Geometry.WorldLine(JigDraw.P3(TruckKinematics.FrontAxle(frames[frames.Count - 1], _p)), _cur);
            }
            return true;
        }
    }
}
