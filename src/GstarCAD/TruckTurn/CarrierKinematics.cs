using System;
using System.Collections.Generic;

namespace TruckTurn
{
    public sealed class CarrierKinematicException : InvalidOperationException
    {
        public CarrierKinematicException(string message) : base(message) { }
    }

    /// <summary>一次恒定运动指令。Speed 为车体参考点速度，正值前进，负值倒行。</summary>
    public struct CarrierMotionCommand
    {
        public CarrierSteeringMode Mode;
        public double Speed;
        public double ReferenceRadius;
        public int TurnDirection;
        public double CrabAngleDeg;
        public double PivotYawRateDegPerSecond;

        public static CarrierMotionCommand Longitudinal(double speed)
        {
            return new CarrierMotionCommand { Mode = CarrierSteeringMode.Longitudinal, Speed = speed };
        }

        public static CarrierMotionCommand Turn(
            CarrierSteeringMode mode, double speed, double referenceRadius, int turnDirection)
        {
            return new CarrierMotionCommand
            {
                Mode = mode,
                Speed = speed,
                ReferenceRadius = referenceRadius,
                TurnDirection = turnDirection < 0 ? -1 : 1
            };
        }

        public static CarrierMotionCommand Crab(double speed, double angleDeg)
        {
            return new CarrierMotionCommand
            {
                Mode = CarrierSteeringMode.Crab,
                Speed = speed,
                CrabAngleDeg = angleDeg
            };
        }

        public static CarrierMotionCommand Lateral(double speed)
        {
            return new CarrierMotionCommand { Mode = CarrierSteeringMode.Lateral, Speed = speed };
        }

        public static CarrierMotionCommand Pivot(double yawRateDegPerSecond)
        {
            return new CarrierMotionCommand
            {
                Mode = CarrierSteeringMode.Pivot,
                PivotYawRateDegPerSecond = yawRateDegPerSecond
            };
        }
    }

    /// <summary>
    /// 四轮跨运车二维准静态运动学。
    /// 基本关系：车轮 i 位于 r=(x,y) 时，轮心速度为
    ///     v_i = (Vx - w*y, Vy + w*x)。
    /// 非纯平移时所有轮心速度线都与同一个瞬时转动中心一致。
    /// </summary>
    public static class CarrierKinematics
    {
        public const double DefaultSimulationStepSeconds = 0.05;
        const double AngleToleranceRad = 1e-9;

        public static CarrierFrame Solve(
            CarrierParams p, CarrierPose pose, CarrierMotionCommand command)
        {
            EnsureParams(p);
            CarrierTwist twist = CommandToTwist(p, command);
            EnsureFinite(twist);
            CarrierWheelState[] wheels = SolveWheelStates(p, pose, twist);
            return new CarrierFrame(pose, twist, command.Mode, wheels);
        }

        public static CarrierTwist CommandToTwist(CarrierParams p, CarrierMotionCommand command)
        {
            EnsureParams(p);
            if (!p.Supports(command.Mode))
                throw new CarrierKinematicException("当前车型未启用转向模式 " + command.Mode + "。 ");

            if (!CarrierMath.Finite(command.Speed) ||
                !CarrierMath.Finite(command.ReferenceRadius) ||
                !CarrierMath.Finite(command.CrabAngleDeg) ||
                !CarrierMath.Finite(command.PivotYawRateDegPerSecond))
                throw new CarrierKinematicException("运动指令包含非有限数值。 ");

            switch (command.Mode)
            {
                case CarrierSteeringMode.Longitudinal:
                    return new CarrierTwist(command.Speed, 0.0, 0.0);

                case CarrierSteeringMode.Crab:
                    return PureTranslation(command.Speed, command.CrabAngleDeg * Math.PI / 180.0);

                case CarrierSteeringMode.Lateral:
                    return PureTranslation(command.Speed, Math.PI * 0.5);

                case CarrierSteeringMode.Pivot:
                    if (p.Architecture != CarrierArchitecture.FourWheelIndependent)
                        throw new CarrierKinematicException("原地回转只允许四轮独立转向架构。 ");
                    return new CarrierTwist(
                        0.0, 0.0, command.PivotYawRateDegPerSecond * Math.PI / 180.0);

                case CarrierSteeringMode.CounterPhaseFourWheel:
                    ValidateTurnCommand(p, command);
                    return ValidateMinimumTurningRadius(p, TwistAboutIcr(
                            command.Speed, command.ReferenceRadius,
                            0.0, SignedLateralIcr(command.ReferenceRadius, command.TurnDirection),
                            command.TurnDirection));

                case CarrierSteeringMode.FrontOnly:
                    ValidateTurnCommand(p, command);
                    return ValidateMinimumTurningRadius(
                        p, SingleAxleSteerTwist(p, command, rearAxleFixed: true));

                case CarrierSteeringMode.RearOnly:
                    ValidateTurnCommand(p, command);
                    return ValidateMinimumTurningRadius(
                        p, SingleAxleSteerTwist(p, command, rearAxleFixed: false));

                default:
                    throw new CarrierKinematicException("未知转向模式。 ");
            }
        }

        public static CarrierWheelState[] SolveWheelStates(
            CarrierParams p, CarrierPose pose, CarrierTwist twist)
        {
            EnsureParams(p);
            EnsureFinite(twist);
            CarrierWheelLocation[] locations = p.ResolvedWheelLocations();
            var result = new CarrierWheelState[locations.Length];
            double c = Math.Cos(pose.HeadingRad);
            double s = Math.Sin(pose.HeadingRad);

            for (int i = 0; i < locations.Length; i++)
            {
                CarrierWheelLocation loc = locations[i];
                double vx = twist.Vx - twist.YawRate * loc.Y;
                double vy = twist.Vy + twist.YawRate * loc.X;
                double rollingSpeed;
                double angle;
                CanonicalWheelDirection(vx, vy, out angle, out rollingSpeed);

                double maxAngle = loc.MaxSteerAngleDeg * Math.PI / 180.0;
                if (Math.Abs(angle) > maxAngle + AngleToleranceRad)
                {
                    throw new CarrierKinematicException(string.Format(
                        "{0} 需要转角 {1:F3}°，超过车型上限 {2:F3}°。",
                        loc.Id, angle * 180.0 / Math.PI, loc.MaxSteerAngleDeg));
                }

                result[i] = new CarrierWheelState
                {
                    Id = loc.Id,
                    LocalX = loc.X,
                    LocalY = loc.Y,
                    WorldX = pose.X + c * loc.X - s * loc.Y,
                    WorldY = pose.Y + s * loc.X + c * loc.Y,
                    SteerAngleRad = angle,
                    RollingSpeed = rollingSpeed
                };
            }
            return result;
        }

        /// <summary>在给定车体速度旋量恒定的条件下，用 SE(2) 精确积分一个时间步。</summary>
        public static CarrierPose Integrate(CarrierPose pose, CarrierTwist twist, double dtSeconds)
        {
            if (!(dtSeconds >= 0.0) || !CarrierMath.Finite(dtSeconds))
                throw new ArgumentOutOfRangeException("dtSeconds");
            EnsureFinite(twist);

            double dxBody;
            double dyBody;
            double dtheta = twist.YawRate * dtSeconds;
            if (Math.Abs(twist.YawRate) < CarrierMath.Epsilon)
            {
                dxBody = twist.Vx * dtSeconds;
                dyBody = twist.Vy * dtSeconds;
            }
            else
            {
                double sw = Math.Sin(dtheta);
                double cw = Math.Cos(dtheta);
                dxBody = (sw * twist.Vx - (1.0 - cw) * twist.Vy) / twist.YawRate;
                dyBody = ((1.0 - cw) * twist.Vx + sw * twist.Vy) / twist.YawRate;
            }

            double c = Math.Cos(pose.HeadingRad);
            double s = Math.Sin(pose.HeadingRad);
            return new CarrierPose(
                pose.X + c * dxBody - s * dyBody,
                pose.Y + s * dxBody + c * dyBody,
                pose.HeadingRad + dtheta);
        }

        public static List<CarrierFrame> Simulate(
            CarrierParams p, CarrierPose start, CarrierMotionCommand command,
            double durationSeconds, double stepSeconds)
        {
            if (!(durationSeconds >= 0.0) || !CarrierMath.Finite(durationSeconds))
                throw new ArgumentOutOfRangeException("durationSeconds");
            if (!(stepSeconds > 0.0) || !CarrierMath.Finite(stepSeconds))
                throw new ArgumentOutOfRangeException("stepSeconds");

            var frames = new List<CarrierFrame>();
            CarrierFrame current = Solve(p, start, command);
            frames.Add(current);
            double elapsed = 0.0;
            while (elapsed < durationSeconds - 1e-12)
            {
                double dt = Math.Min(stepSeconds, durationSeconds - elapsed);
                CarrierPose nextPose = Integrate(current.Pose, current.Twist, dt);
                current = Solve(p, nextPose, command);
                frames.Add(current);
                elapsed += dt;
                if (frames.Count > 1000000)
                    throw new CarrierKinematicException("仿真帧数异常，已中止。 ");
            }
            return frames;
        }

        /// <summary>用所有轮心速度做最小二乘，反算车体速度旋量。</summary>
        public static CarrierTwist EstimateTwist(CarrierWheelState[] wheels)
        {
            if (wheels == null || wheels.Length < 2)
                throw new ArgumentException("至少需要两个轮位。", "wheels");

            double n = wheels.Length;
            double sumX = 0.0, sumY = 0.0, sumR2 = 0.0;
            double bx = 0.0, by = 0.0, bw = 0.0;
            for (int i = 0; i < wheels.Length; i++)
            {
                CarrierWheelState wheel = wheels[i];
                double sx = wheel.RollingSpeed * Math.Cos(wheel.SteerAngleRad);
                double sy = wheel.RollingSpeed * Math.Sin(wheel.SteerAngleRad);
                sumX += wheel.LocalX;
                sumY += wheel.LocalY;
                sumR2 += wheel.LocalX * wheel.LocalX + wheel.LocalY * wheel.LocalY;
                bx += sx;
                by += sy;
                bw += -wheel.LocalY * sx + wheel.LocalX * sy;
            }

            double[,] augmented =
            {
                { n, 0.0, -sumY, bx },
                { 0.0, n, sumX, by },
                { -sumY, sumX, sumR2, bw }
            };
            double[] x = Solve3x3(augmented);
            return new CarrierTwist(x[0], x[1], x[2]);
        }

        public static double MaxWheelVelocityResidual(CarrierWheelState[] wheels, CarrierTwist twist)
        {
            double max = 0.0;
            for (int i = 0; i < wheels.Length; i++)
            {
                CarrierWheelState wheel = wheels[i];
                double expectedX = twist.Vx - twist.YawRate * wheel.LocalY;
                double expectedY = twist.Vy + twist.YawRate * wheel.LocalX;
                double actualX = wheel.RollingSpeed * Math.Cos(wheel.SteerAngleRad);
                double actualY = wheel.RollingSpeed * Math.Sin(wheel.SteerAngleRad);
                double dx = actualX - expectedX;
                double dy = actualY - expectedY;
                max = Math.Max(max, Math.Sqrt(dx * dx + dy * dy));
            }
            return max;
        }

        public static CarrierTurnRadii TurnRadii(CarrierParams p, CarrierFrame frame)
        {
            EnsureParams(p);
            if (frame == null) throw new ArgumentNullException("frame");
            return CalculateTurnRadii(p, frame.Twist);
        }

        static CarrierTurnRadii CalculateTurnRadii(CarrierParams p, CarrierTwist twist)
        {
            if (Math.Abs(twist.YawRate) < CarrierMath.Epsilon)
            {
                return new CarrierTurnRadii
                {
                    IsStraight = true,
                    ReferencePoint = double.PositiveInfinity,
                    InnerWheel = double.PositiveInfinity,
                    OuterWheel = double.PositiveInfinity,
                    InnerBody = double.PositiveInfinity,
                    OuterBody = double.PositiveInfinity
                };
            }

            double cx = -twist.Vy / twist.YawRate;
            double cy = twist.Vx / twist.YawRate;
            double innerWheel = double.PositiveInfinity;
            double outerWheel = 0.0;
            CarrierWheelLocation[] locations = p.ResolvedWheelLocations();
            for (int i = 0; i < locations.Length; i++)
            {
                double dx = locations[i].X - cx;
                double dy = locations[i].Y - cy;
                double r = Math.Sqrt(dx * dx + dy * dy);
                innerWheel = Math.Min(innerWheel, r);
                outerWheel = Math.Max(outerWheel, r);
            }

            double hx = p.OverallLength * 0.5;
            double hy = p.OverallWidth * 0.5;
            double innerDx = Math.Max(Math.Abs(cx) - hx, 0.0);
            double innerDy = Math.Max(Math.Abs(cy) - hy, 0.0);
            double innerBody = Math.Sqrt(innerDx * innerDx + innerDy * innerDy);
            double outerBody = 0.0;
            for (int ix = -1; ix <= 1; ix += 2)
            {
                for (int iy = -1; iy <= 1; iy += 2)
                {
                    double dx = ix * hx - cx;
                    double dy = iy * hy - cy;
                    outerBody = Math.Max(outerBody, Math.Sqrt(dx * dx + dy * dy));
                }
            }

            return new CarrierTurnRadii
            {
                IsStraight = false,
                ReferencePoint = Math.Sqrt(cx * cx + cy * cy),
                InnerWheel = innerWheel,
                OuterWheel = outerWheel,
                InnerBody = innerBody,
                OuterBody = outerBody
            };
        }

        /// <summary>停车转向时按最大转角速度逼近目标轮角，不改变车体位姿。</summary>
        public static double[] RateLimitSteering(
            double[] currentAnglesRad, CarrierWheelState[] targetWheels,
            double maxRateDegPerSecond, double dtSeconds)
        {
            if (currentAnglesRad == null || targetWheels == null ||
                currentAnglesRad.Length != targetWheels.Length)
                throw new ArgumentException("当前轮角和目标轮角数量必须一致。 ");
            if (!(maxRateDegPerSecond > 0.0) || !(dtSeconds >= 0.0) ||
                !CarrierMath.Finite(maxRateDegPerSecond) || !CarrierMath.Finite(dtSeconds))
                throw new ArgumentOutOfRangeException("转向速率或时间步无效。 ");

            double maxStep = maxRateDegPerSecond * Math.PI / 180.0 * dtSeconds;
            var result = new double[currentAnglesRad.Length];
            for (int i = 0; i < result.Length; i++)
            {
                double delta = CarrierMath.NormalizeAngleRad(
                    targetWheels[i].SteerAngleRad - currentAnglesRad[i]);
                delta = CarrierMath.Clamp(delta, -maxStep, +maxStep);
                result[i] = CarrierMath.NormalizeAngleRad(currentAnglesRad[i] + delta);
            }
            return result;
        }

        public static double[,] BodyCorners(CarrierParams p, CarrierPose pose)
        {
            return RectangleCorners(pose, p.OverallLength, p.OverallWidth, 0.0, 0.0, 0.0);
        }

        public static double[,] ContainerCorners(CarrierParams p, CarrierPose pose)
        {
            CarrierContainerDimensions dims = p.ContainerDimensions();
            if (dims.Length <= 0.0 || dims.Width <= 0.0) return new double[0, 2];
            return RectangleCorners(
                pose, dims.Length, dims.Width,
                p.ContainerOffsetX, p.ContainerOffsetY,
                p.ContainerSlewAngleDeg * Math.PI / 180.0);
        }

        static CarrierTwist PureTranslation(double speed, double directionRad)
        {
            return new CarrierTwist(
                speed * Math.Cos(directionRad), speed * Math.Sin(directionRad), 0.0);
        }

        static CarrierTwist SingleAxleSteerTwist(
            CarrierParams p, CarrierMotionCommand command, bool rearAxleFixed)
        {
            double halfWheelbase = p.Wheelbase * 0.5;
            if (command.ReferenceRadius <= halfWheelbase + CarrierMath.Epsilon)
                throw new CarrierKinematicException(
                    "单桥转向的车体参考点半径必须大于半轴距。 ");
            double lateral = Math.Sqrt(
                command.ReferenceRadius * command.ReferenceRadius - halfWheelbase * halfWheelbase);
            lateral *= command.TurnDirection < 0 ? -1.0 : 1.0;
            double icrX = rearAxleFixed ? -halfWheelbase : +halfWheelbase;
            return TwistAboutIcr(
                command.Speed, command.ReferenceRadius, icrX, lateral, command.TurnDirection);
        }

        static CarrierTwist TwistAboutIcr(
            double referenceSpeed, double referenceRadius,
            double icrX, double icrY, int turnDirection)
        {
            double yawRate = (turnDirection < 0 ? -1.0 : 1.0) *
                             referenceSpeed / referenceRadius;
            return new CarrierTwist(yawRate * icrY, -yawRate * icrX, yawRate);
        }

        static double SignedLateralIcr(double radius, int direction)
        {
            return (direction < 0 ? -1.0 : 1.0) * radius;
        }

        static void ValidateTurnCommand(CarrierParams p, CarrierMotionCommand command)
        {
            if (!(command.ReferenceRadius > 0.0))
                throw new CarrierKinematicException("转弯半径必须大于 0。 ");
            if (command.TurnDirection != -1 && command.TurnDirection != 1)
                throw new CarrierKinematicException("转弯方向必须为 -1 或 +1。 ");
        }

        static CarrierTwist ValidateMinimumTurningRadius(CarrierParams p, CarrierTwist twist)
        {
            CarrierTurnRadii radii = CalculateTurnRadii(p, twist);
            double actual;
            string label;
            switch (p.TurningRadiusReference)
            {
                case CarrierTurningRadiusReference.BodyReferencePoint:
                    actual = radii.ReferencePoint;
                    label = "车体参考点";
                    break;
                case CarrierTurningRadiusReference.InnerWheel:
                    actual = radii.InnerWheel;
                    label = "内轮";
                    break;
                case CarrierTurningRadiusReference.OuterWheel:
                    actual = radii.OuterWheel;
                    label = "外轮";
                    break;
                case CarrierTurningRadiusReference.OuterBody:
                    actual = radii.OuterBody;
                    label = "车体外缘";
                    break;
                default:
                    throw new CarrierKinematicException("未知的最小转弯半径测量基准。 ");
            }

            if (actual + 1e-9 < p.MinimumReferenceRadius)
                throw new CarrierKinematicException(string.Format(
                    "{0}半径 {1:F3}m 小于车型下限 {2:F3}m。",
                    label, actual, p.MinimumReferenceRadius));
            return twist;
        }

        static void CanonicalWheelDirection(
            double vx, double vy, out double angle, out double rollingSpeed)
        {
            double speed = Math.Sqrt(vx * vx + vy * vy);
            if (speed < CarrierMath.Epsilon)
            {
                angle = 0.0;
                rollingSpeed = 0.0;
                return;
            }

            angle = Math.Atan2(vy, vx);
            rollingSpeed = speed;
            if (angle > Math.PI * 0.5)
            {
                angle -= Math.PI;
                rollingSpeed = -rollingSpeed;
            }
            else if (angle < -Math.PI * 0.5)
            {
                angle += Math.PI;
                rollingSpeed = -rollingSpeed;
            }
        }

        static double[] Solve3x3(double[,] a)
        {
            for (int col = 0; col < 3; col++)
            {
                int pivot = col;
                double pivotAbs = Math.Abs(a[pivot, col]);
                for (int row = col + 1; row < 3; row++)
                {
                    double candidate = Math.Abs(a[row, col]);
                    if (candidate > pivotAbs) { pivot = row; pivotAbs = candidate; }
                }
                if (pivotAbs < 1e-12)
                    throw new CarrierKinematicException("轮位几何退化，无法反算车体速度。 ");
                if (pivot != col)
                {
                    for (int k = col; k < 4; k++)
                    {
                        double tmp = a[col, k]; a[col, k] = a[pivot, k]; a[pivot, k] = tmp;
                    }
                }
                double divisor = a[col, col];
                for (int k = col; k < 4; k++) a[col, k] /= divisor;
                for (int row = 0; row < 3; row++)
                {
                    if (row == col) continue;
                    double factor = a[row, col];
                    for (int k = col; k < 4; k++) a[row, k] -= factor * a[col, k];
                }
            }
            return new[] { a[0, 3], a[1, 3], a[2, 3] };
        }

        static double[,] RectangleCorners(
            CarrierPose pose, double length, double width,
            double offsetX, double offsetY, double localRotation)
        {
            double hx = length * 0.5;
            double hy = width * 0.5;
            double cl = Math.Cos(localRotation);
            double sl = Math.Sin(localRotation);
            double cw = Math.Cos(pose.HeadingRad);
            double sw = Math.Sin(pose.HeadingRad);
            var output = new double[4, 2];
            int index = 0;
            for (int ix = -1; ix <= 1; ix += 2)
            {
                for (int iy = -1; iy <= 1; iy += 2)
                {
                    double x = ix * hx;
                    double y = iy * hy;
                    double lx = offsetX + cl * x - sl * y;
                    double ly = offsetY + sl * x + cl * y;
                    output[index, 0] = pose.X + cw * lx - sw * ly;
                    output[index, 1] = pose.Y + sw * lx + cw * ly;
                    index++;
                }
            }
            return output;
        }

        static void EnsureParams(CarrierParams p)
        {
            if (p == null) throw new ArgumentNullException("p");
            List<string> errors = p.Validate();
            if (errors.Count > 0)
                throw new CarrierKinematicException("跨运车参数无效：" + string.Join("；", errors));
        }

        static void EnsureFinite(CarrierTwist twist)
        {
            if (!CarrierMath.Finite(twist.Vx) || !CarrierMath.Finite(twist.Vy) ||
                !CarrierMath.Finite(twist.YawRate))
                throw new CarrierKinematicException("车体速度旋量包含非有限数值。 ");
        }
    }
}
