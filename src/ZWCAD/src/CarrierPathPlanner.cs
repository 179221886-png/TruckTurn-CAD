using System;
using System.Collections.Generic;

namespace TruckTurn
{
    /// <summary>鼠标目标点反解得到的一段可执行跨运车运动。</summary>
    public sealed class CarrierSegmentPlan
    {
        public CarrierSteeringMode Mode;
        public CarrierMotionCommand Command;
        public List<CarrierFrame> Frames = new List<CarrierFrame>();
        public double DurationSeconds;
        public double SteeringTransitionSeconds;
        public double CursorMissDistance;

        public CarrierFrame LastFrame
        {
            get { return Frames == null || Frames.Count == 0 ? null : Frames[Frames.Count - 1]; }
        }
    }

    /// <summary>
    /// CAD 无关的交互路径反解器。把光标点解释为当前转向模式下的目标位置，
    /// 再统一调用 CarrierKinematics 生成运动帧；预览和确认必须使用同一结果。
    /// </summary>
    public static class CarrierPathPlanner
    {
        public const double NominalSpeedMetersPerSecond = 1.0;
        public const double PivotRateDegPerSecond = 20.0;
        public const double SteeringStepSeconds = 0.10;
        public const double MaximumInteractiveSegmentMeters = 500.0;
        const double MinimumSegmentMeters = 0.05;

        public static bool TryPlan(
            CarrierParams p, CarrierPose start, double[] currentWheelAnglesRad,
            double targetX, double targetY, CarrierSteeringMode mode, int driveDirection,
            out CarrierSegmentPlan plan, out string error)
        {
            plan = null;
            error = "";
            try
            {
                if (p == null) throw new ArgumentNullException("p");
                if (!p.Supports(mode))
                    throw new CarrierKinematicException("当前车型不支持 " + mode + " 模式。 ");
                if (!CarrierMath.Finite(targetX) || !CarrierMath.Finite(targetY))
                    throw new CarrierKinematicException("目标点坐标无效。 ");

                int dir = driveDirection < 0 ? -1 : 1;
                double dx = targetX - start.X;
                double dy = targetY - start.Y;
                double c = Math.Cos(start.HeadingRad);
                double s = Math.Sin(start.HeadingRad);
                double a = c * dx + s * dy;
                double b = -s * dx + c * dy;
                double cursorDistance = Math.Sqrt(dx * dx + dy * dy);
                if (cursorDistance > MaximumInteractiveSegmentMeters)
                    throw new CarrierKinematicException(
                        "单段距离超过 500m，请检查图纸单位或缩短单段距离。 ");

                CarrierMotionCommand command;
                double duration;
                bool exactCursor;

                switch (mode)
                {
                    case CarrierSteeringMode.Longitudinal:
                        if (dir * a < MinimumSegmentMeters)
                            throw new CarrierKinematicException("目标点不在当前行驶方向，请按 B 切换前进/倒行。 ");
                        command = CarrierMotionCommand.Longitudinal(dir * NominalSpeedMetersPerSecond);
                        duration = Math.Abs(a) / NominalSpeedMetersPerSecond;
                        exactCursor = Math.Abs(b) < 1e-6;
                        break;

                    case CarrierSteeringMode.Crab:
                        double length = cursorDistance;
                        if (length < MinimumSegmentMeters)
                            throw new CarrierKinematicException("目标点距离太近。 ");
                        double crab = Math.Atan2(b, a) * 180.0 / Math.PI;
                        double crabSpeed = NominalSpeedMetersPerSecond;
                        if (crab > 90.0) { crab -= 180.0; crabSpeed = -crabSpeed; }
                        else if (crab < -90.0) { crab += 180.0; crabSpeed = -crabSpeed; }
                        if (Math.Abs(a) > MinimumSegmentMeters && Math.Sign(crabSpeed) != dir)
                            throw new CarrierKinematicException("目标点不在当前行驶方向，请按 B 切换前进/倒行。 ");
                        command = CarrierMotionCommand.Crab(crabSpeed, crab);
                        duration = length / NominalSpeedMetersPerSecond;
                        exactCursor = true;
                        break;

                    case CarrierSteeringMode.Lateral:
                        if (Math.Abs(b) < MinimumSegmentMeters)
                            throw new CarrierKinematicException("横移目标必须位于车体左侧或右侧。 ");
                        command = CarrierMotionCommand.Lateral(
                            Math.Sign(b) * NominalSpeedMetersPerSecond);
                        duration = Math.Abs(b) / NominalSpeedMetersPerSecond;
                        exactCursor = Math.Abs(a) < 1e-6;
                        break;

                    case CarrierSteeringMode.Pivot:
                        if (Math.Sqrt(dx * dx + dy * dy) < MinimumSegmentMeters)
                            throw new CarrierKinematicException("请用光标指定新的车头方向。 ");
                        double desired = Math.Atan2(dy, dx);
                        double delta = CarrierMath.NormalizeAngleRad(desired - start.HeadingRad);
                        if (Math.Abs(delta) < 0.25 * Math.PI / 180.0)
                            throw new CarrierKinematicException("回转角度太小。 ");
                        double rate = Math.Sign(delta) * PivotRateDegPerSecond;
                        command = CarrierMotionCommand.Pivot(rate);
                        duration = Math.Abs(delta) / (PivotRateDegPerSecond * Math.PI / 180.0);
                        exactCursor = false;
                        break;

                    case CarrierSteeringMode.CounterPhaseFourWheel:
                    case CarrierSteeringMode.FrontOnly:
                    case CarrierSteeringMode.RearOnly:
                        if (Math.Abs(b) < 1e-5)
                        {
                            if (dir * a < MinimumSegmentMeters)
                                throw new CarrierKinematicException("目标点不在当前行驶方向，请按 B 切换。 ");
                            command = CarrierMotionCommand.Longitudinal(
                                dir * NominalSpeedMetersPerSecond);
                            duration = Math.Abs(a) / NominalSpeedMetersPerSecond;
                            exactCursor = true;
                            break;
                        }

                        double icrX = 0.0;
                        if (mode == CarrierSteeringMode.FrontOnly) icrX = -p.Wheelbase * 0.5;
                        else if (mode == CarrierSteeringMode.RearOnly) icrX = p.Wheelbase * 0.5;

                        double phi = CarrierMath.NormalizeAngleRad(
                            2.0 * Math.Atan2(b, a - 2.0 * icrX));
                        if (Math.Abs(phi) < 1e-7)
                            throw new CarrierKinematicException("无法从该目标点形成稳定圆弧。 ");

                        double oneMinusCos = 1.0 - Math.Cos(phi);
                        if (oneMinusCos < 1e-10)
                            throw new CarrierKinematicException("目标点过于接近直线奇异位置。 ");
                        double icrY = (b * oneMinusCos + a * Math.Sin(phi)) /
                                      (2.0 * oneMinusCos);
                        double radius = Math.Sqrt(icrX * icrX + icrY * icrY);
                        int turnDirection = icrY < 0.0 ? -1 : 1;
                        int expectedPhiSign = turnDirection * dir;
                        if (Math.Sign(phi) != expectedPhiSign)
                            throw new CarrierKinematicException("该目标点需要绕行超过 180°，请分段操作或按 B 切换方向。 ");

                        command = CarrierMotionCommand.Turn(
                            mode, dir * NominalSpeedMetersPerSecond, radius, turnDirection);
                        duration = Math.Abs(phi) * radius / NominalSpeedMetersPerSecond;
                        exactCursor = true;
                        break;

                    default:
                        throw new CarrierKinematicException("未知转向模式。 ");
                }

                double step = SimulationStep(p, command);
                List<CarrierFrame> motion = CarrierKinematics.Simulate(
                    p, start, command, duration, step);
                if (motion.Count == 0)
                    throw new CarrierKinematicException("运动学未生成有效帧。 ");

                var result = new CarrierSegmentPlan
                {
                    Mode = mode,
                    Command = command,
                    DurationSeconds = duration
                };
                AppendSteeringTransition(
                    p, start, currentWheelAnglesRad, motion[0].Wheels,
                    mode, result.Frames, out double transitionSeconds);
                result.SteeringTransitionSeconds = transitionSeconds;
                result.Frames.AddRange(motion);

                CarrierPose end = result.LastFrame.Pose;
                result.CursorMissDistance = exactCursor
                    ? Math.Sqrt((end.X - targetX) * (end.X - targetX) +
                                (end.Y - targetY) * (end.Y - targetY))
                    : Math.Sqrt((end.X - targetX) * (end.X - targetX) +
                                (end.Y - targetY) * (end.Y - targetY));
                plan = result;
                return true;
            }
            catch (Exception ex)
            {
                error = ex.Message;
                return false;
            }
        }

        static double SimulationStep(CarrierParams p, CarrierMotionCommand command)
        {
            CarrierTwist twist = CarrierKinematics.CommandToTwist(p, command);
            double byDistance = 0.20 / Math.Max(
                Math.Sqrt(twist.Vx * twist.Vx + twist.Vy * twist.Vy), 1e-9);
            double byAngle = (1.0 * Math.PI / 180.0) /
                             Math.Max(Math.Abs(twist.YawRate), 1e-9);
            return CarrierMath.Clamp(Math.Min(byDistance, byAngle), 0.01, 0.25);
        }

        static void AppendSteeringTransition(
            CarrierParams p, CarrierPose pose, double[] currentAngles,
            CarrierWheelState[] targetWheels, CarrierSteeringMode mode,
            List<CarrierFrame> output, out double seconds)
        {
            int n = targetWheels.Length;
            double[] cur = new double[n];
            for (int i = 0; i < n; i++)
                cur[i] = currentAngles != null && i < currentAngles.Length
                    ? currentAngles[i] : 0.0;

            double maxDiff = MaxAngleDifference(cur, targetWheels);
            seconds = maxDiff / (p.MaxSteerRateDegPerSecond * Math.PI / 180.0);
            if (seconds < 1e-9) return;

            output.Add(StoppedFrame(pose, mode, targetWheels, cur));
            double elapsed = 0.0;
            while (elapsed < seconds - 1e-10)
            {
                double dt = Math.Min(SteeringStepSeconds, seconds - elapsed);
                cur = CarrierKinematics.RateLimitSteering(
                    cur, targetWheels, p.MaxSteerRateDegPerSecond, dt);
                output.Add(StoppedFrame(pose, mode, targetWheels, cur));
                elapsed += dt;
                if (output.Count > 10000)
                    throw new CarrierKinematicException("停车转向过渡帧数异常。 ");
            }
        }

        static double MaxAngleDifference(double[] current, CarrierWheelState[] target)
        {
            double max = 0.0;
            for (int i = 0; i < target.Length; i++)
            {
                double d = Math.Abs(CarrierMath.NormalizeAngleRad(
                    target[i].SteerAngleRad - current[i]));
                if (d > max) max = d;
            }
            return max;
        }

        static CarrierFrame StoppedFrame(
            CarrierPose pose, CarrierSteeringMode mode,
            CarrierWheelState[] target, double[] angles)
        {
            var wheels = new CarrierWheelState[target.Length];
            for (int i = 0; i < target.Length; i++)
            {
                wheels[i] = target[i];
                wheels[i].SteerAngleRad = angles[i];
                wheels[i].RollingSpeed = 0.0;
            }
            return new CarrierFrame(pose, new CarrierTwist(0.0, 0.0, 0.0), mode, wheels);
        }
    }
}
