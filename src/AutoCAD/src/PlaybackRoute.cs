using System;
using System.Collections.Generic;

namespace TruckTurn
{
    public enum PlaybackVehicleKind { Truck = 0, Carrier = 1, Fire = 2 }

    /// <summary>独立于当前选型的路线快照。坐标是图形单位；时间是演示时间，不是实车驾驶计时。</summary>
    public sealed class PlaybackRoute
    {
        public int Version = 1;
        public PlaybackVehicleKind Kind;
        public string DisplayName;
        public string SourceSummary = "";
        public double UnitScale;
        public VehicleParams TruckParameters;
        public CarrierParams CarrierParameters;
        public List<PlaybackPose> Poses = new List<PlaybackPose>();
        public double DurationSeconds
        {
            get { return Poses == null || Poses.Count == 0 ? 0.0 : Poses[Poses.Count - 1].TimeSeconds; }
        }
    }

    public sealed class PlaybackPose
    {
        public double TimeSeconds, X, Y, HeadingRad, TrailerHeadingRad, FrontSteerRad;
        public double[] WheelAnglesRad;
        public int SteeringMode, TravelDirection;
    }

    public static class PlaybackRouteFactory
    {
        public const double DemonstrationSpeedMetersPerSecond = 1.4;
        const double YawSpeed = 20.0 * Math.PI / 180.0;
        const double WheelSpeed = 15.0 * Math.PI / 180.0;

        public static PlaybackRoute FromTruck(VehicleParams parameters, IList<Frame> frames,
            PlaybackVehicleKind kind = PlaybackVehicleKind.Truck, string displayName = null)
        {
            if (parameters == null) throw new ArgumentNullException("parameters");
            CheckCount(frames == null ? 0 : frames.Count);
            if (kind != PlaybackVehicleKind.Truck && kind != PlaybackVehicleKind.Fire)
                throw new ArgumentException("货车回放类别无效。", "kind");
            var p = PlaybackRouteCodec.CloneTruckParameters(parameters);
            var route = new PlaybackRoute
            {
                Kind = kind, DisplayName = displayName ?? (kind == PlaybackVehicleKind.Fire ? "消防车轨迹" : "货车轨迹"),
                UnitScale = p.UnitScale, TruckParameters = p
            };
            for (int i = 0; i < frames.Count; i++)
            {
                Frame f = frames[i];
                if (f == null || !ValidVector(f.U1.X, f.U1.Y) ||
                    (p.Articulated && !ValidVector(f.U2.X, f.U2.Y)))
                    throw new ArgumentException("车辆轨迹包含无效姿态。");
                double h = Math.Atan2(f.U1.Y, f.U1.X);
                route.Poses.Add(new PlaybackPose
                {
                    X = f.O1.X, Y = f.O1.Y, HeadingRad = h,
                    TrailerHeadingRad = p.Articulated ? Math.Atan2(f.U2.Y, f.U2.X) : h,
                    WheelAnglesRad = new double[0], TravelDirection = 1
                });
            }

            // 由实际位移和航向变化估计自行车转角，而不是复用分段的左/右标签。
            // 反向驾驶时 signedDistance 为负，前轮显示方向仍与真实曲率一致。
            int lastDirection = 1;
            for (int i = 0; i < route.Poses.Count; i++)
            {
                int a = i, b = i + 1;
                if (b >= route.Poses.Count) { a = i - 1; b = i; }
                if (a < 0) continue;
                PlaybackPose x = route.Poses[a], y = route.Poses[b];
                double dh = PlaybackTimeline.AngleDifference(x.HeadingRad, y.HeadingRad);
                double mid = x.HeadingRad + dh * 0.5;
                double dx = y.X - x.X, dy = y.Y - x.Y;
                double distance = Math.Sqrt(dx * dx + dy * dy);
                double sign = dx * Math.Cos(mid) + dy * Math.Sin(mid);
                if (distance > p.UnitScale * 1e-9)
                {
                    lastDirection = sign < 0 ? -1 : 1;
                    // Exact circular arc: chord = 2 R sin(dHeading/2).
                    double curvature = 2.0 * Math.Sin(dh * 0.5) / (lastDirection * distance);
                    route.Poses[i].FrontSteerRad = Math.Atan(p.TractorWheelbase * curvature);
                }
                route.Poses[i].TravelDirection = lastDirection;
            }
            AssignTimes(route, WheelSpeed);
            PlaybackRouteCodec.Validate(route);
            return route;
        }

        public static PlaybackRoute FromCarrier(CarrierParams parameters, IList<CarrierFrame> frames,
            string displayName = null)
        {
            if (parameters == null) throw new ArgumentNullException("parameters");
            CheckCount(frames == null ? 0 : frames.Count);
            var p = PlaybackRouteCodec.CloneCarrierParameters(parameters);
            var route = new PlaybackRoute
            {
                Kind = PlaybackVehicleKind.Carrier, DisplayName = displayName ?? p.PresetName,
                UnitScale = p.UnitScale, CarrierParameters = p
            };
            int direction = 1;
            foreach (CarrierFrame f in frames)
            {
                if (f == null || f.Wheels == null || f.Wheels.Length != 4)
                    throw new ArgumentException("搬运车轨迹必须包含四轮完整姿态。");
                double vx = f.Twist.Vx, vy = f.Twist.Vy;
                if (!PlaybackTimeline.Finite(vx) || !PlaybackTimeline.Finite(vy) ||
                    !PlaybackTimeline.Finite(f.Twist.YawRate))
                    throw new ArgumentException("搬运车轨迹速度无效。");
                if (Math.Abs(vx) > 1e-9) direction = vx < 0 ? -1 : 1;
                else if (Math.Abs(vy) > 1e-9) direction = vy < 0 ? -1 : 1;
                var wheels = new double[4];
                var used = new bool[4];
                foreach (CarrierWheelState w in f.Wheels)
                {
                    int id = (int)w.Id;
                    if (id < 0 || id >= 4 || used[id])
                        throw new ArgumentException("搬运车轮位无效或重复。");
                    used[id] = true;
                    wheels[id] = w.SteerAngleRad;
                }
                route.Poses.Add(new PlaybackPose
                {
                    X = f.Pose.X * p.UnitScale, Y = f.Pose.Y * p.UnitScale,
                    HeadingRad = f.Pose.HeadingRad, TrailerHeadingRad = f.Pose.HeadingRad,
                    FrontSteerRad = wheels[0], WheelAnglesRad = wheels,
                    SteeringMode = (int)f.Mode, TravelDirection = direction
                });
            }
            AssignTimes(route, p.MaxSteerRateDegPerSecond * Math.PI / 180.0);
            PlaybackRouteCodec.Validate(route);
            return route;
        }

        static void CheckCount(int count)
        {
            if (count < 1) throw new ArgumentException("不能保存空轨迹。");
            if (count > PlaybackRouteCodec.MaximumPoseCount)
                throw new ArgumentException("轨迹超过回放存储上限（200000 帧），请分段录制；不会自动降低轨迹精度。");
        }

        static bool ValidVector(double x, double y)
        {
            return PlaybackTimeline.Finite(x) && PlaybackTimeline.Finite(y) && x * x + y * y > 1e-12;
        }

        static void AssignTimes(PlaybackRoute route, double steerSpeed)
        {
            route.Poses[0].TimeSeconds = 0.0;
            steerSpeed = Math.Min(WheelSpeed, steerSpeed);
            if (!PlaybackTimeline.Finite(steerSpeed) || steerSpeed <= 0) throw new ArgumentException("转向角速度无效。");
            for (int i = 1; i < route.Poses.Count; i++)
            {
                PlaybackPose a = route.Poses[i - 1], b = route.Poses[i];
                double dx = (b.X - a.X) / route.UnitScale, dy = (b.Y - a.Y) / route.UnitScale;
                double dt = Math.Sqrt(dx * dx + dy * dy) / DemonstrationSpeedMetersPerSecond;
                dt = Math.Max(dt, Math.Abs(PlaybackTimeline.AngleDifference(a.HeadingRad, b.HeadingRad)) / YawSpeed);
                dt = Math.Max(dt, Math.Abs(PlaybackTimeline.AngleDifference(a.TrailerHeadingRad, b.TrailerHeadingRad)) / YawSpeed);
                dt = Math.Max(dt, Math.Abs(PlaybackTimeline.AngleDifference(a.FrontSteerRad, b.FrontSteerRad)) / steerSpeed);
                if (a.WheelAnglesRad != null && b.WheelAnglesRad != null)
                    for (int j = 0; j < Math.Min(a.WheelAnglesRad.Length, b.WheelAnglesRad.Length); j++)
                        dt = Math.Max(dt, Math.Abs(PlaybackTimeline.AngleDifference(a.WheelAnglesRad[j], b.WheelAnglesRad[j])) / steerSpeed);
                // 重复接缝也保留；停车摆轮由真实转角变化计时，不按采样索引强加时长。
                b.TimeSeconds = a.TimeSeconds + Math.Max(0.001, dt);
            }
        }
    }

    public static class PlaybackTimeline
    {
        public static PlaybackPose Sample(PlaybackRoute route, double seconds)
        {
            if (route == null || route.Poses == null || route.Poses.Count == 0)
                throw new ArgumentException("回放轨迹为空。");
            if (!Finite(seconds)) throw new ArgumentOutOfRangeException("seconds");
            if (seconds <= 0 || route.Poses.Count == 1) return Interpolate(route.Poses[0], route.Poses[0], 0);
            if (seconds >= route.DurationSeconds)
                return Interpolate(route.Poses[route.Poses.Count - 1], route.Poses[route.Poses.Count - 1], 0);
            int lo = 0, hi = route.Poses.Count - 1;
            while (hi - lo > 1)
            {
                int mid = lo + (hi - lo) / 2;
                if (route.Poses[mid].TimeSeconds <= seconds) lo = mid; else hi = mid;
            }
            PlaybackPose a = route.Poses[lo], b = route.Poses[hi];
            if (!(b.TimeSeconds > a.TimeSeconds)) throw new ArgumentException("回放时间非递增。");
            return Interpolate(a, b, (seconds - a.TimeSeconds) / (b.TimeSeconds - a.TimeSeconds));
        }

        static PlaybackPose Interpolate(PlaybackPose a, PlaybackPose b, double t)
        {
            var result = new PlaybackPose
            {
                TimeSeconds = a.TimeSeconds + (b.TimeSeconds - a.TimeSeconds) * t,
                X = a.X + (b.X - a.X) * t, Y = a.Y + (b.Y - a.Y) * t,
                HeadingRad = AngleLerp(a.HeadingRad, b.HeadingRad, t),
                TrailerHeadingRad = AngleLerp(a.TrailerHeadingRad, b.TrailerHeadingRad, t),
                FrontSteerRad = AngleLerp(a.FrontSteerRad, b.FrontSteerRad, t),
                SteeringMode = t >= 1 ? b.SteeringMode : a.SteeringMode,
                TravelDirection = t >= 1 ? b.TravelDirection : a.TravelDirection
            };
            int count = a.WheelAnglesRad == null ? 0 : a.WheelAnglesRad.Length;
            if ((b.WheelAnglesRad == null ? 0 : b.WheelAnglesRad.Length) != count)
                throw new ArgumentException("回放车轮数量不一致。");
            result.WheelAnglesRad = new double[count];
            for (int i = 0; i < count; i++) result.WheelAnglesRad[i] = AngleLerp(a.WheelAnglesRad[i], b.WheelAnglesRad[i], t);
            return result;
        }

        public static double AngleDifference(double from, double to)
        {
            double delta = (to - from) % (Math.PI * 2.0);
            if (delta > Math.PI) delta -= Math.PI * 2.0;
            if (delta <= -Math.PI) delta += Math.PI * 2.0;
            return delta;
        }
        public static double AngleLerp(double from, double to, double t)
        {
            double angle = (from + AngleDifference(from, to) * t) % (2.0 * Math.PI);
            if (angle > Math.PI) angle -= Math.PI * 2.0;
            if (angle <= -Math.PI) angle += Math.PI * 2.0;
            return angle;
        }
        internal static bool Finite(double value) { return !double.IsNaN(value) && !double.IsInfinity(value); }
    }
}
