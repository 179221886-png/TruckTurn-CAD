using System;
using System.Collections.Generic;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using GI = Autodesk.AutoCAD.GraphicsInterface;

namespace TruckTurn
{
    internal static class CarrierJigDraw
    {
        public static void Poly(GI.WorldDraw draw, IList<Point2d> points, bool closed)
        {
            if (points == null || points.Count < 2) return;
            for (int i = 0; i + 1 < points.Count; i++)
                draw.Geometry.WorldLine(P3(points[i]), P3(points[i + 1]));
            if (closed && points.Count > 2)
                draw.Geometry.WorldLine(P3(points[points.Count - 1]), P3(points[0]));
        }

        public static void Frame(GI.WorldDraw draw, CarrierParams p, CarrierFrame frame)
        {
            if (frame == null) return;
            if (p.ShowBody)
            {
                SetColor(draw, 3);
                Poly(draw, CarrierCadGeometry.BodyPolygon(p, frame.Pose), true);
                List<Point2d> container = CarrierCadGeometry.ContainerPolygon(p, frame.Pose);
                if (container.Count > 0)
                {
                    SetColor(draw, 30);
                    Poly(draw, container, true);
                }
                SetColor(draw, 1);
                for (int i = 0; i < frame.Wheels.Length; i++)
                    Poly(draw, CarrierCadGeometry.WheelPolygon(p, frame, i), true);
            }

            Point2d center = CarrierCadGeometry.ToCad(frame.Pose.X, frame.Pose.Y, p);
            Point2d nose = CarrierCadGeometry.ToCad(
                frame.Pose.X + Math.Cos(frame.Pose.HeadingRad) * p.OverallLength * 0.65,
                frame.Pose.Y + Math.Sin(frame.Pose.HeadingRad) * p.OverallLength * 0.65, p);
            SetColor(draw, 4);
            draw.Geometry.WorldLine(P3(center), P3(nose));
        }

        public static void Segment(GI.WorldDraw draw, CarrierParams p, IList<CarrierFrame> frames)
        {
            if (frames == null || frames.Count == 0) return;
            if (p.ShowEquipmentEnvelope)
            {
                SetColor(draw, 2);
                Poly(draw, CarrierCadGeometry.EquipmentEnvelope(p, frames), true);
            }
            if (p.ShowContainerEnvelope && p.ContainerPreset != CarrierContainerPreset.None)
            {
                SetColor(draw, 30);
                Poly(draw, CarrierCadGeometry.ContainerEnvelope(p, frames), true);
            }
            if (p.ShowSafetyEnvelope)
            {
                SetColor(draw, 1);
                Poly(draw, CarrierCadGeometry.ClearanceEnvelope(p, frames), true);
            }
            if (p.ShowReferencePath)
            {
                SetColor(draw, 6);
                Poly(draw, CarrierCadGeometry.ReferencePath(p, frames), false);
            }
            if (p.ShowWheelTracks)
            {
                SetColor(draw, 1);
                foreach (List<Point2d> track in CarrierCadGeometry.WheelTracks(p, frames))
                    Poly(draw, track, false);
            }
            Frame(draw, p, frames[frames.Count - 1]);
        }

        public static Point3d P3(Point2d p) { return new Point3d(p.X, p.Y, 0.0); }
        public static void SetColor(GI.WorldDraw draw, short aci)
        {
            try { draw.SubEntityTraits.Color = aci; } catch { }
        }
    }

    public sealed class CarrierPlaceJig : DrawJig
    {
        readonly CarrierParams p;
        readonly double heading;
        Point3d current;
        public Point3d Result { get { return current; } }

        public CarrierPlaceJig(CarrierParams parameters, double headingRad, Point3d seed)
        {
            p = parameters;
            heading = headingRad;
            current = seed;
        }

        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            try
            {
                var options = new JigPromptPointOptions(
                    "\n指定跨运车车体中心，移动鼠标预览设备与集装箱");
                options.UserInputControls = UserInputControls.Accept3dCoordinates |
                    UserInputControls.AcceptMouseUpAsPoint;
                PromptPointResult result = prompts.AcquirePoint(options);
                if (result.Status != PromptStatus.OK) return SamplerStatus.Cancel;
                if (result.Value.DistanceTo(current) < 1e-8) return SamplerStatus.NoChange;
                current = result.Value;
                return SamplerStatus.OK;
            }
            catch (Exception ex)
            {
                try { Console.WriteLine("[CARRIERDRIVE 放置 Sampler] " + ex); } catch { }
                return SamplerStatus.Cancel;
            }
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            try
            {
                CarrierPose pose = CarrierCadGeometry.ToMeterPose(current, heading, p);
                CarrierFrame frame = CarrierKinematics.Solve(
                    p, pose, CarrierMotionCommand.Longitudinal(0.0));
                CarrierJigDraw.Frame(draw, p, frame);
                return true;
            }
            catch (Exception ex)
            {
                try { Console.WriteLine("[CARRIERDRIVE 放置 WorldDraw] " + ex); } catch { }
                return false;
            }
        }
    }

    public sealed class CarrierHeadingJig : DrawJig
    {
        readonly CarrierParams p;
        readonly Point3d center;
        Point3d current;
        double heading;
        public double HeadingRad { get { return heading; } }

        public CarrierHeadingJig(CarrierParams parameters, Point3d centerCad, double seedHeading)
        {
            p = parameters;
            center = centerCad;
            heading = seedHeading;
            double length = Math.Max(parameters.OverallLength * parameters.UnitScale, 1.0);
            current = new Point3d(
                center.X + Math.Cos(heading) * length,
                center.Y + Math.Sin(heading) * length, center.Z);
        }

        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            try
            {
                var options = new JigPromptPointOptions("\n指定跨运车车头方向");
                options.BasePoint = center;
                options.UseBasePoint = true;
                options.UserInputControls = UserInputControls.Accept3dCoordinates |
                    UserInputControls.AcceptMouseUpAsPoint;
                PromptPointResult result = prompts.AcquirePoint(options);
                if (result.Status != PromptStatus.OK) return SamplerStatus.Cancel;
                if (result.Value.DistanceTo(current) < 1e-8) return SamplerStatus.NoChange;
                current = result.Value;
                double dx = current.X - center.X, dy = current.Y - center.Y;
                if (dx * dx + dy * dy > 1e-12) heading = Math.Atan2(dy, dx);
                return SamplerStatus.OK;
            }
            catch (Exception ex)
            {
                try { Console.WriteLine("[CARRIERDRIVE 朝向 Sampler] " + ex); } catch { }
                return SamplerStatus.Cancel;
            }
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            try
            {
                CarrierPose pose = CarrierCadGeometry.ToMeterPose(center, heading, p);
                CarrierFrame frame = CarrierKinematics.Solve(
                    p, pose, CarrierMotionCommand.Longitudinal(0.0));
                CarrierJigDraw.Frame(draw, p, frame);
                CarrierJigDraw.SetColor(draw, 4);
                draw.Geometry.WorldLine(center, current);
                return true;
            }
            catch (Exception ex)
            {
                try { Console.WriteLine("[CARRIERDRIVE 朝向 WorldDraw] " + ex); } catch { }
                return false;
            }
        }
    }

    public sealed class CarrierDriveJig : DrawJig
    {
        // 仅限制鼠标移动时的临时图形量；点击确认后的正式出图仍使用 plan.Frames 全量帧。
        // 160 帧配合已知瞬心圆弧拟合，可兼顾极端倒车转弯的包络稳定性与实时交互性能。
        const int PreviewEnvelopeFrameLimit = 160;
        const int PreviewTrackFrameLimit = 48;

        readonly CarrierParams p;
        readonly CarrierPose start;
        readonly double[] currentWheelAngles;
        readonly CarrierSteeringMode mode;
        readonly int driveDirection;
        Point3d current;
        CarrierSegmentPlan plan;
        string invalidHint = "";
        string keyword = "";
        bool hasDrawn;
        CarrierPose drawnEnd;
        CarrierSegmentPlan cachedPlan;
        List<Point2d> cachedEquipment;
        List<Point2d> cachedContainer;
        List<Point2d> cachedClearance;
        List<Point2d> cachedPath;
        List<Point2d>[] cachedWheels;

        public string Keyword { get { return keyword; } }
        public CarrierSegmentPlan Plan { get { return plan; } }
        public CarrierSteeringMode Mode { get { return mode; } }

        public CarrierDriveJig(
            CarrierParams parameters, CarrierPose startPose, double[] wheelAngles,
            CarrierSteeringMode steeringMode, int direction)
        {
            p = parameters;
            start = startPose;
            currentWheelAngles = wheelAngles;
            mode = steeringMode;
            driveDirection = direction < 0 ? -1 : 1;
            Point2d seed = CarrierCadGeometry.ToCad(start.X, start.Y, p);
            current = CarrierJigDraw.P3(seed);
        }

        protected override SamplerStatus Sampler(JigPrompts prompts)
        {
            try { return SamplerCore(prompts); }
            catch (Exception ex)
            {
                try { Console.WriteLine("[CARRIERDRIVE 驾驶 Sampler] " + ex); } catch { }
                plan = null;
                return SamplerStatus.Cancel;
            }
        }

        SamplerStatus SamplerCore(JigPrompts prompts)
        {
            string message = string.Format(
                "\n跨运车[{0}｜{1}] 左键=确认 [M]=切换模式 [B]=前进/倒行 [X]=完成 [Esc]=撤销 {2}",
                CarrierCadGeometry.ModeName(mode), driveDirection > 0 ? "前进" : "倒行", invalidHint);
            var options = new JigPromptPointOptions(message);
            options.BasePoint = CarrierJigDraw.P3(CarrierCadGeometry.ToCad(start.X, start.Y, p));
            options.UseBasePoint = true;
            options.Keywords.Add("M", "M", "切换转向模式(M)");
            options.Keywords.Add("B", "B", "切换前进倒行(B)");
            options.Keywords.Add("X", "X", "完成并出图(X)");
            options.AppendKeywordsToMessage = true;
            options.UserInputControls = UserInputControls.Accept3dCoordinates |
                UserInputControls.AcceptMouseUpAsPoint |
                UserInputControls.UseBasePointElevation;

            PromptPointResult result = prompts.AcquirePoint(options);
            if (result.Status == PromptStatus.Keyword)
            {
                keyword = result.StringResult;
                return SamplerStatus.Cancel;
            }
            if (result.Status != PromptStatus.OK) return SamplerStatus.Cancel;
            if (result.Value.DistanceTo(current) < 1e-8) return SamplerStatus.NoChange;
            current = result.Value;
            double targetX = current.X / p.UnitScale;
            double targetY = current.Y / p.UnitScale;
            if (!CarrierPathPlanner.TryPlan(
                p, start, currentWheelAngles, targetX, targetY,
                mode, driveDirection, out plan, out invalidHint))
            {
                plan = null;
                invalidHint = "｜" + invalidHint;
            }
            else invalidHint = "";
            return SamplerStatus.OK;
        }

        protected override bool WorldDraw(GI.WorldDraw draw)
        {
            try
            {
                if (plan == null || plan.Frames == null || plan.Frames.Count == 0)
                {
                    hasDrawn = false;
                    CarrierJigDraw.SetColor(draw, 8);
                    draw.Geometry.WorldLine(
                        CarrierJigDraw.P3(CarrierCadGeometry.ToCad(start.X, start.Y, p)), current);
                    return true;
                }
                DrawCachedSegment(draw);
                drawnEnd = plan.LastFrame.Pose;
                hasDrawn = true;
                return true;
            }
            catch (Exception ex)
            {
                try { Console.WriteLine("[CARRIERDRIVE 驾驶 WorldDraw] " + ex); } catch { }
                hasDrawn = false;
                return false;
            }
        }

        void DrawCachedSegment(GI.WorldDraw draw)
        {
            if (!object.ReferenceEquals(cachedPlan, plan))
            {
                // 实时包络只保留均匀关键帧；中心线与轮迹还可更稀。
                // 圆弧恢复由 SmoothEnvelopePreview 完成，正式出图不走这里的帧数上限。
                List<CarrierFrame> envelopeFrames =
                    CarrierCadGeometry.PreviewFrames(plan.Frames, PreviewEnvelopeFrameLimit);
                List<CarrierFrame> trackFrames =
                    CarrierCadGeometry.PreviewFrames(plan.Frames, PreviewTrackFrameLimit);
                if (p.ShowEquipmentEnvelope)
                    cachedEquipment = CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.EquipmentEnvelope(p, envelopeFrames), envelopeFrames);
                else cachedEquipment = null;
                if (p.ShowContainerEnvelope)
                    cachedContainer = CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.ContainerEnvelope(p, envelopeFrames), envelopeFrames);
                else cachedContainer = null;
                if (p.ShowSafetyEnvelope)
                    cachedClearance = CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.ClearanceEnvelope(p, envelopeFrames), envelopeFrames);
                else cachedClearance = null;
                cachedPath = p.ShowReferencePath
                    ? CarrierCadGeometry.ReferencePath(p, trackFrames) : null;
                cachedWheels = p.ShowWheelTracks
                    ? CarrierCadGeometry.WheelTracks(p, trackFrames) : null;
                cachedPlan = plan;
            }

            if (cachedEquipment != null)
            {
                CarrierJigDraw.SetColor(draw, 2);
                CarrierJigDraw.Poly(draw, cachedEquipment, true);
            }
            if (cachedContainer != null && cachedContainer.Count > 0)
            {
                CarrierJigDraw.SetColor(draw, 30);
                CarrierJigDraw.Poly(draw, cachedContainer, true);
            }
            if (cachedClearance != null)
            {
                CarrierJigDraw.SetColor(draw, 1);
                CarrierJigDraw.Poly(draw, cachedClearance, true);
            }
            if (cachedPath != null)
            {
                CarrierJigDraw.SetColor(draw, 6);
                CarrierJigDraw.Poly(draw, cachedPath, false);
            }
            if (cachedWheels != null)
            {
                CarrierJigDraw.SetColor(draw, 1);
                for (int i = 0; i < cachedWheels.Length; i++)
                    CarrierJigDraw.Poly(draw, cachedWheels[i], false);
            }
            CarrierJigDraw.Frame(draw, p, plan.LastFrame);
        }

        public bool ConfirmMatchesDrawn(out string reason)
        {
            reason = "";
            if (!hasDrawn || plan == null || plan.LastFrame == null)
            {
                reason = "尚无有效预览";
                return false;
            }
            CarrierPose end = plan.LastFrame.Pose;
            double dx = end.X - drawnEnd.X, dy = end.Y - drawnEnd.Y;
            double distance = Math.Sqrt(dx * dx + dy * dy);
            double angle = Math.Abs(CarrierMath.NormalizeAngleRad(
                end.HeadingRad - drawnEnd.HeadingRad)) * 180.0 / Math.PI;
            if (distance > 0.01 || angle > 0.1)
            {
                reason = string.Format("末点差 {0:F3}m，朝向差 {1:F2}°", distance, angle);
                return false;
            }
            return true;
        }
    }
}
