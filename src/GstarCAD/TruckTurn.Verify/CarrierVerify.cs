using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace TruckTurn
{
    static partial class VerifyProgram
    {
        const double CarrierTol = 1e-8;

        static int RunCarrierPhase1()
        {
            Console.WriteLine("\n-- 跨运车 Phase 1：参数、四轮运动学与数值不变量 --");
            int fail = 0;
            fail += CarrierCase("参数与箱型预设", VerifyCarrierParams);
            fail += CarrierCase("纵向直行与倒行", VerifyCarrierLongitudinal);
            fail += CarrierCase("蟹行与纯横移", VerifyCarrierTranslations);
            fail += CarrierCase("反相四轮 90° 转弯", VerifyCarrierCounterPhaseTurn);
            fail += CarrierCase("反相四轮倒行", VerifyCarrierReverseTurn);
            fail += CarrierCase("前桥/后桥单独转向", VerifyCarrierSingleAxleModes);
            fail += CarrierCase("原地回转能力开关", VerifyCarrierPivot);
            fail += CarrierCase("轮角上限与最小半径", VerifyCarrierConstraints);
            fail += CarrierCase("停车转向速率限制", VerifyCarrierSteerRate);
            fail += CarrierCase("轮速正解反算一致性", VerifyCarrierForwardInverse);
            fail += CarrierCase("外廓与集装箱角点", VerifyCarrierFootprints);
            fail += CarrierCase("参数网格数值扫描", VerifyCarrierParameterSweep);
            fail += CarrierCase("交互目标点路径反解", VerifyCarrierPathPlanner);
            fail += CarrierCase("CAD 单位与分层包络", VerifyCarrierCadGeometry);
            fail += CarrierCase("蟹行解析直线包络与预览降采样", VerifyCarrierCrabEnvelope);
            fail += CarrierCase("多段蟹行累计包络保持直线", VerifyCarrierSegmentedCrabEnvelope);
            fail += CarrierCase("反相四轮转弯圆弧平滑", VerifyCarrierTurnEnvelopeSmoothing);
            fail += CarrierCase("全部转弯模式包络圆弧矩阵", VerifyAllCarrierTurnModes);
            fail += CarrierCase("多角度多半径尖刺扫描", VerifyCarrierEnvelopeSpikeSweep);
            fail += CarrierCase("160帧实时预览与全量包络一致性", VerifyCarrierPreviewQuality);
            fail += CarrierCase("累计预览自适应采样与安全圆弧", VerifyCarrierAdaptiveCumulativePreview);
            Console.WriteLine(fail == 0
                ? "跨运车 Phase 1：全部通过。"
                : "跨运车 Phase 1：存在 " + fail + " 个失败场景。");
            return fail;
        }

        static void RunCarrierPreviewPerformance()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), new[] { 0.0, 0.0, 0.0, 0.0 },
                35.0, 22.0, CarrierSteeringMode.CounterPhaseFourWheel, +1,
                out CarrierSegmentPlan plan, out string error),
                "性能测试路径失败：" + error);

            Console.WriteLine("\n-- 跨运车实时预览性能 --");
            Console.WriteLine("全量帧：" + plan.Frames.Count);
            foreach (int cap in new[] { 256, 160, 128, 96, 64 })
            {
                List<CarrierFrame> frames = CarrierCadGeometry.PreviewFrames(plan.Frames, cap);
                MeasurePreview("设备", cap, 5, delegate
                {
                    return CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.EquipmentEnvelope(p, frames), frames).Count;
                });
                MeasurePreview("安全", cap, 5, delegate
                {
                    return CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.ClearanceEnvelope(p, frames), frames).Count;
                });
                MeasurePreview("安全原", cap, 5, delegate
                {
                    return CarrierCadGeometry.ClearanceEnvelope(p, frames).Count;
                });
            }
        }

        static void MeasurePreview(string name, int cap, int iterations, Func<int> action)
        {
            action();
            var stopwatch = Stopwatch.StartNew();
            int points = 0;
            for (int i = 0; i < iterations; i++) points = action();
            stopwatch.Stop();
            Console.WriteLine("{0,-4} cap={1,3}  {2,7:F1} ms/次  点={3}",
                name, cap, stopwatch.Elapsed.TotalMilliseconds / iterations, points);
        }

        static int CarrierCase(string name, Action test)
        {
            try
            {
                test();
                Console.WriteLine("  [OK] " + name);
                return 0;
            }
            catch (Exception ex)
            {
                Console.WriteLine("  [FAIL] " + name + "：" + ex.Message);
                return 1;
            }
        }

        static void VerifyCarrierParams()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            AssertTrue(p.Validate().Count == 0, "通用四轮参数应合法");
            AssertTrue(p.Supports(CarrierSteeringMode.Crab), "独立转向默认应支持蟹行");
            AssertTrue(!p.Supports(CarrierSteeringMode.Pivot), "原地回转必须默认关闭");
            AssertTrue(!p.ShowReferencePath && p.ShowEquipmentEnvelope &&
                       !p.ShowContainerEnvelope && p.ShowSafetyEnvelope &&
                       !p.ShowWheelTracks && p.ShowBody &&
                       !p.ShowAllNodeVehicles,
                "CAD 输出默认只应勾选设备包络、安全包络、车体与车轮，车辆姿态默认仅首尾");

            AssertDimensions(p, CarrierContainerPreset.Iso20Gp, 6.058, 2.438, 2.591);
            AssertDimensions(p, CarrierContainerPreset.Iso40Gp, 12.192, 2.438, 2.591);
            AssertDimensions(p, CarrierContainerPreset.Iso40Hc, 12.192, 2.438, 2.896);
            AssertDimensions(p, CarrierContainerPreset.Iso45Hc, 13.716, 2.438, 2.896);

            p.ContainerPreset = CarrierContainerPreset.Custom;
            p.CustomContainerLength = 7.1;
            p.CustomContainerWidth = 2.7;
            p.CustomContainerHeight = 3.0;
            CarrierContainerDimensions custom = p.ContainerDimensions();
            AssertNear(custom.Length, 7.1, CarrierTol, "自定义箱长");

            p.Architecture = CarrierArchitecture.MultiAxle;
            AssertTrue(p.Validate().Count > 0, "V1 必须拒绝多轴运动学");
        }

        static void VerifyCarrierLongitudinal()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            CarrierPose start = new CarrierPose(1.0, -2.0, 0.0);
            List<CarrierFrame> forward = CarrierKinematics.Simulate(
                p, start, CarrierMotionCommand.Longitudinal(2.0), 5.0, 0.13);
            CarrierFrame f = forward[forward.Count - 1];
            AssertNear(f.Pose.X, 11.0, CarrierTol, "直行 X");
            AssertNear(f.Pose.Y, -2.0, CarrierTol, "直行 Y");
            AssertNear(f.Pose.HeadingRad, 0.0, CarrierTol, "直行朝向");
            AssertWheelResidual(f);

            List<CarrierFrame> reverse = CarrierKinematics.Simulate(
                p, start, CarrierMotionCommand.Longitudinal(-1.5), 4.0, 0.07);
            f = reverse[reverse.Count - 1];
            AssertNear(f.Pose.X, -5.0, CarrierTol, "倒行 X");
            for (int i = 0; i < f.Wheels.Length; i++)
            {
                AssertNear(f.Wheels[i].SteerAngleRad, 0.0, CarrierTol, "倒行轮角");
                AssertTrue(f.Wheels[i].RollingSpeed < 0.0, "倒行轮速必须为负");
            }
        }

        static void VerifyCarrierTranslations()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            CarrierPose start = new CarrierPose(0.0, 0.0, 0.0);
            CarrierFrame crab = Last(CarrierKinematics.Simulate(
                p, start, CarrierMotionCommand.Crab(2.0, 30.0), 5.0, 0.11));
            AssertNear(crab.Pose.X, 10.0 * Math.Cos(Math.PI / 6.0), CarrierTol, "蟹行 X");
            AssertNear(crab.Pose.Y, 5.0, CarrierTol, "蟹行 Y");
            AssertNear(crab.Pose.HeadingRad, 0.0, CarrierTol, "蟹行朝向不变");
            for (int i = 0; i < crab.Wheels.Length; i++)
                AssertNear(crab.Wheels[i].SteerAngleDeg, 30.0, 1e-9, "蟹行轮角");
            AssertWheelResidual(crab);

            CarrierFrame lateral = Last(CarrierKinematics.Simulate(
                p, start, CarrierMotionCommand.Lateral(1.5), 3.0, 0.08));
            AssertNear(lateral.Pose.X, 0.0, CarrierTol, "横移 X");
            AssertNear(lateral.Pose.Y, 4.5, CarrierTol, "横移 Y");
            AssertNear(lateral.Pose.HeadingRad, 0.0, CarrierTol, "横移朝向不变");
            for (int i = 0; i < lateral.Wheels.Length; i++)
                AssertNear(lateral.Wheels[i].SteerAngleDeg, 90.0, 1e-9, "横移轮角");
        }

        static void VerifyCarrierCounterPhaseTurn()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            double speed = 2.0;
            double radius = 10.0;
            double duration = (Math.PI * 0.5) / (speed / radius);
            CarrierFrame end = Last(CarrierKinematics.Simulate(
                p, new CarrierPose(0.0, 0.0, 0.0),
                CarrierMotionCommand.Turn(
                    CarrierSteeringMode.CounterPhaseFourWheel, speed, radius, +1),
                duration, 0.031));
            AssertNear(end.Pose.X, radius, 1e-7, "左转 90° X");
            AssertNear(end.Pose.Y, radius, 1e-7, "左转 90° Y");
            AssertNear(end.Pose.HeadingRad, Math.PI * 0.5, 1e-8, "左转 90° 朝向");
            AssertTrue(end.Wheels[0].SteerAngleRad > 0.0, "前轮应向左");
            AssertTrue(end.Wheels[2].SteerAngleRad < 0.0, "后轮应反相");
            AssertWheelResidual(end);

            CarrierTurnRadii radii = CarrierKinematics.TurnRadii(p, end);
            AssertTrue(!radii.IsStraight, "转弯不应标为直线");
            AssertNear(radii.ReferencePoint, radius, CarrierTol, "参考点半径");
            AssertTrue(radii.InnerWheel < radius && radii.OuterWheel > radius,
                "内外轮半径应分列参考点两侧");
            AssertTrue(radii.OuterBody > radii.OuterWheel, "外廓半径应大于外轮半径");
        }

        static void VerifyCarrierReverseTurn()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            double speed = -2.0;
            double radius = 10.0;
            double duration = (Math.PI * 0.5) / (Math.Abs(speed) / radius);
            CarrierFrame end = Last(CarrierKinematics.Simulate(
                p, new CarrierPose(0.0, 0.0, 0.0),
                CarrierMotionCommand.Turn(
                    CarrierSteeringMode.CounterPhaseFourWheel, speed, radius, +1),
                duration, 0.029));
            AssertNear(end.Pose.X, -radius, 1e-7, "倒行左舵 X");
            AssertNear(end.Pose.Y, radius, 1e-7, "倒行左舵 Y");
            AssertNear(end.Pose.HeadingRad, -Math.PI * 0.5, 1e-8, "倒行左舵朝向");
            for (int i = 0; i < end.Wheels.Length; i++)
                AssertTrue(end.Wheels[i].RollingSpeed < 0.0, "倒行转弯轮速必须为负");
            AssertWheelResidual(end);
        }

        static void VerifyCarrierSingleAxleModes()
        {
            CarrierParams p = CarrierParams.GenericFourWheelLinked();
            p.MinimumReferenceRadius = 7.0;
            CarrierPose pose = new CarrierPose(0.0, 0.0, 0.0);
            CarrierFrame front = CarrierKinematics.Solve(
                p, pose, CarrierMotionCommand.Turn(CarrierSteeringMode.FrontOnly, 2.0, 12.0, +1));
            AssertNear(front.Wheels[2].SteerAngleRad, 0.0, CarrierTol, "前轮模式左后轮");
            AssertNear(front.Wheels[3].SteerAngleRad, 0.0, CarrierTol, "前轮模式右后轮");
            AssertWheelResidual(front);

            CarrierFrame rear = CarrierKinematics.Solve(
                p, pose, CarrierMotionCommand.Turn(CarrierSteeringMode.RearOnly, 2.0, 12.0, -1));
            AssertNear(rear.Wheels[0].SteerAngleRad, 0.0, CarrierTol, "后轮模式左前轮");
            AssertNear(rear.Wheels[1].SteerAngleRad, 0.0, CarrierTol, "后轮模式右前轮");
            AssertWheelResidual(rear);
        }

        static void VerifyCarrierPivot()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            ExpectCarrierFailure(delegate
            {
                CarrierKinematics.Solve(p, new CarrierPose(), CarrierMotionCommand.Pivot(30.0));
            }, "默认必须禁止原地回转");

            p.SteeringCapabilities |= CarrierSteeringCapabilities.Pivot;
            CarrierFrame end = Last(CarrierKinematics.Simulate(
                p, new CarrierPose(3.0, -4.0, 0.0), CarrierMotionCommand.Pivot(30.0), 3.0, 0.04));
            AssertNear(end.Pose.X, 3.0, CarrierTol, "原地回转 X");
            AssertNear(end.Pose.Y, -4.0, CarrierTol, "原地回转 Y");
            AssertNear(end.Pose.HeadingRad, Math.PI * 0.5, 1e-8, "原地回转朝向");
            AssertWheelResidual(end);
        }

        static void VerifyCarrierConstraints()
        {
            CarrierParams p = CarrierParams.GenericFourWheelLinked();
            p.SteeringCapabilities |= CarrierSteeringCapabilities.Lateral;
            ExpectCarrierFailure(delegate
            {
                CarrierKinematics.Solve(p, new CarrierPose(), CarrierMotionCommand.Lateral(1.0));
            }, "45° 转角上限必须拒绝纯横移");

            p = CarrierParams.GenericFourWheelIndependent();
            ExpectCarrierFailure(delegate
            {
                CarrierKinematics.Solve(
                    p, new CarrierPose(),
                    CarrierMotionCommand.Turn(
                        CarrierSteeringMode.CounterPhaseFourWheel, 1.0, 8.99, +1));
            }, "必须拒绝小于最小参考点半径的转弯");

            p.MinimumReferenceRadius = 1.0;
            CarrierMotionCommand atTen = CarrierMotionCommand.Turn(
                CarrierSteeringMode.CounterPhaseFourWheel, 1.0, 10.0, +1);
            CarrierFrame frame = CarrierKinematics.Solve(p, new CarrierPose(), atTen);
            CarrierTurnRadii radii = CarrierKinematics.TurnRadii(p, frame);

            p.TurningRadiusReference = CarrierTurningRadiusReference.OuterWheel;
            p.MinimumReferenceRadius = radii.OuterWheel + 0.01;
            ExpectCarrierFailure(delegate
            {
                CarrierKinematics.Solve(p, new CarrierPose(), atTen);
            }, "外轮半径基准必须独立执行下限校验");

            p.TurningRadiusReference = CarrierTurningRadiusReference.OuterBody;
            p.MinimumReferenceRadius = radii.OuterBody - 0.01;
            CarrierKinematics.Solve(p, new CarrierPose(), atTen);
        }

        static void VerifyCarrierSteerRate()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            CarrierFrame target = CarrierKinematics.Solve(
                p, new CarrierPose(), CarrierMotionCommand.Lateral(1.0));
            double[] current = { 0.0, 0.0, 0.0, 0.0 };
            double[] next = CarrierKinematics.RateLimitSteering(current, target.Wheels, 15.0, 2.0);
            for (int i = 0; i < next.Length; i++)
                AssertNear(next[i] * 180.0 / Math.PI, 30.0, 1e-9, "两秒转角限速");

            double[] reached = CarrierKinematics.RateLimitSteering(next, target.Wheels, 60.0, 2.0);
            for (int i = 0; i < reached.Length; i++)
                AssertNear(reached[i] * 180.0 / Math.PI, 90.0, 1e-9, "目标轮角");
        }

        static void VerifyCarrierForwardInverse()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            CarrierFrame frame = CarrierKinematics.Solve(
                p, new CarrierPose(2.0, 4.0, 0.7),
                CarrierMotionCommand.Turn(
                    CarrierSteeringMode.CounterPhaseFourWheel, 1.7, 13.0, -1));
            CarrierTwist estimated = CarrierKinematics.EstimateTwist(frame.Wheels);
            AssertNear(estimated.Vx, frame.Twist.Vx, 1e-10, "反算 Vx");
            AssertNear(estimated.Vy, frame.Twist.Vy, 1e-10, "反算 Vy");
            AssertNear(estimated.YawRate, frame.Twist.YawRate, 1e-10, "反算角速度");
            AssertWheelResidual(frame);
        }

        static void VerifyCarrierFootprints()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            p.ContainerOffsetX = 0.5;
            p.ContainerOffsetY = -0.25;
            CarrierPose pose = new CarrierPose(10.0, 20.0, Math.PI * 0.5);
            double[,] body = CarrierKinematics.BodyCorners(p, pose);
            double[,] container = CarrierKinematics.ContainerCorners(p, pose);
            AssertTrue(body.GetLength(0) == 4 && container.GetLength(0) == 4,
                "设备与集装箱必须各有四个角点");
            AssertNear(Distance(body, 0, 2), p.OverallLength, 1e-10, "设备长边");
            AssertNear(Distance(body, 0, 1), p.OverallWidth, 1e-10, "设备短边");
            CarrierContainerDimensions dims = p.ContainerDimensions();
            AssertNear(Distance(container, 0, 2), dims.Length, 1e-10, "集装箱长边");
            AssertNear(Distance(container, 0, 1), dims.Width, 1e-10, "集装箱短边");
        }

        static void VerifyCarrierParameterSweep()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            CarrierPose pose = new CarrierPose(1.2, -3.4, 0.37);
            double[] radii = { 9.0, 12.0, 30.0 };
            double[] speeds = { -2.0, -0.5, 0.5, 2.0 };
            int[] directions = { -1, 1 };
            int cases = 0;

            for (int r = 0; r < radii.Length; r++)
            for (int s = 0; s < speeds.Length; s++)
            for (int d = 0; d < directions.Length; d++)
            {
                CarrierMotionCommand command = CarrierMotionCommand.Turn(
                    CarrierSteeringMode.CounterPhaseFourWheel,
                    speeds[s], radii[r], directions[d]);
                CarrierFrame frame = CarrierKinematics.Solve(p, pose, command);
                AssertWheelResidual(frame);
                CarrierTwist estimated = CarrierKinematics.EstimateTwist(frame.Wheels);
                AssertNear(estimated.Vx, frame.Twist.Vx, 1e-10, "扫描反算 Vx");
                AssertNear(estimated.Vy, frame.Twist.Vy, 1e-10, "扫描反算 Vy");
                AssertNear(estimated.YawRate, frame.Twist.YawRate, 1e-10, "扫描反算角速度");

                CarrierFrame integrated = CarrierKinematics.Solve(
                    p, CarrierKinematics.Integrate(pose, frame.Twist, 0.17), command);
                AssertTrue(CarrierMath.Finite(integrated.Pose.X) &&
                           CarrierMath.Finite(integrated.Pose.Y) &&
                           CarrierMath.Finite(integrated.Pose.HeadingRad),
                    "扫描积分不得出现 NaN/Infinity");
                cases++;
            }

            double[] crabAngles = { -90.0, -45.0, 0.0, 45.0, 90.0 };
            for (int i = 0; i < crabAngles.Length; i++)
            {
                CarrierFrame frame = CarrierKinematics.Solve(
                    p, pose, CarrierMotionCommand.Crab(1.3, crabAngles[i]));
                AssertWheelResidual(frame);
                cases++;
            }

            AssertTrue(cases == 29, "参数扫描场景数不完整");

            CarrierParams linked = CarrierParams.GenericFourWheelLinked();
            ExpectCarrierFailure(delegate
            {
                CarrierKinematics.CommandToTwist(linked, CarrierMotionCommand.Crab(1.0, 20.0));
            }, "底层指令转换也必须执行能力开关校验");
        }

        static void VerifyCarrierPathPlanner()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            CarrierPose start = new CarrierPose(2.0, -1.0, 0.0);
            double[] wheelAngles = { 0.0, 0.0, 0.0, 0.0 };
            CarrierSegmentPlan plan;
            string error;

            AssertTrue(CarrierPathPlanner.TryPlan(
                p, start, wheelAngles, 10.0, 3.0,
                CarrierSteeringMode.CounterPhaseFourWheel, +1, out plan, out error),
                "反相四轮目标点反解失败：" + error);
            AssertNear(plan.LastFrame.Pose.X, 10.0, 1e-7, "圆弧目标 X");
            AssertNear(plan.LastFrame.Pose.Y, 3.0, 1e-7, "圆弧目标 Y");
            AssertTrue(plan.SteeringTransitionSeconds > 0.0, "首段应包含停车摆轮过渡");

            CarrierPose next = plan.LastFrame.Pose;
            double[] current = WheelAngles(plan.LastFrame);
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, next, current,
                next.X - 3.0 * Math.Sin(next.HeadingRad),
                next.Y + 3.0 * Math.Cos(next.HeadingRad),
                CarrierSteeringMode.Lateral, +1, out plan, out error),
                "横移目标点反解失败：" + error);
            AssertNear(plan.CursorMissDistance, 0.0, 1e-7, "横移目标误差");

            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), wheelAngles, -5.0, 1.0,
                CarrierSteeringMode.Crab, -1, out plan, out error),
                "倒行蟹行反解失败：" + error);
            AssertNear(plan.LastFrame.Pose.X, -5.0, 1e-7, "倒蟹行 X");
            AssertNear(plan.LastFrame.Pose.Y, 1.0, 1e-7, "倒蟹行 Y");

            CarrierParams linked = CarrierParams.GenericFourWheelLinked();
            linked.MinimumReferenceRadius = 7.0;
            AssertTrue(CarrierPathPlanner.TryPlan(
                linked, new CarrierPose(), wheelAngles, 10.0, 2.0,
                CarrierSteeringMode.FrontOnly, +1, out plan, out error),
                "前桥转向目标点反解失败：" + error);
            AssertNear(plan.LastFrame.Pose.X, 10.0, 1e-7, "前桥模式目标 X");
            AssertNear(plan.LastFrame.Pose.Y, 2.0, 1e-7, "前桥模式目标 Y");

            AssertTrue(CarrierPathPlanner.TryPlan(
                linked, new CarrierPose(), wheelAngles, 10.0, -2.0,
                CarrierSteeringMode.RearOnly, +1, out plan, out error),
                "后桥转向目标点反解失败：" + error);
            AssertNear(plan.LastFrame.Pose.X, 10.0, 1e-7, "后桥模式目标 X");
            AssertNear(plan.LastFrame.Pose.Y, -2.0, 1e-7, "后桥模式目标 Y");

            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), wheelAngles, -8.0, 3.0,
                CarrierSteeringMode.CounterPhaseFourWheel, -1, out plan, out error),
                "倒行圆弧目标点反解失败：" + error);
            AssertNear(plan.LastFrame.Pose.X, -8.0, 1e-7, "倒行圆弧目标 X");
            AssertNear(plan.LastFrame.Pose.Y, 3.0, 1e-7, "倒行圆弧目标 Y");

            p.SteeringCapabilities |= CarrierSteeringCapabilities.Pivot;
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(4.0, 5.0, 0.0), wheelAngles, 4.0, 10.0,
                CarrierSteeringMode.Pivot, +1, out plan, out error),
                "原地回转目标方向反解失败：" + error);
            AssertNear(plan.LastFrame.Pose.X, 4.0, CarrierTol, "回转中心 X");
            AssertNear(plan.LastFrame.Pose.Y, 5.0, CarrierTol, "回转中心 Y");
            AssertNear(plan.LastFrame.Pose.HeadingRad, Math.PI * 0.5, 1e-8, "回转目标朝向");

            AssertTrue(!CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), wheelAngles, 501.0, 0.0,
                CarrierSteeringMode.Longitudinal, +1, out plan, out error),
                "交互单段必须限制在 500m 内以避免 Jig 卡死");
        }

        static double[] WheelAngles(CarrierFrame frame)
        {
            var result = new double[frame.Wheels.Length];
            for (int i = 0; i < result.Length; i++)
                result[i] = frame.Wheels[i].SteerAngleRad;
            return result;
        }

        static void VerifyCarrierCadGeometry()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.UnitScale = 1000.0;
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            CarrierSegmentPlan plan;
            string error;
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), new[] { 0.0, 0.0, 0.0, 0.0 },
                10.0, 4.0, CarrierSteeringMode.CounterPhaseFourWheel, +1,
                out plan, out error), "包络测试路径失败：" + error);

            var equipment = CarrierCadGeometry.EquipmentEnvelope(p, plan.Frames);
            AssertTrue(!CarrierCadGeometry.EnvelopeFallbackUsed,
                "设备包络不应退回凸包保底");
            var container = CarrierCadGeometry.ContainerEnvelope(p, plan.Frames);
            AssertTrue(!CarrierCadGeometry.EnvelopeFallbackUsed,
                "箱体包络不应退回凸包保底");
            var clearance = CarrierCadGeometry.ClearanceEnvelope(p, plan.Frames);
            AssertTrue(!CarrierCadGeometry.EnvelopeFallbackUsed,
                "安全包络不应退回凸包保底");
            AssertTrue(equipment.Count >= 3 && container.Count >= 3 && clearance.Count >= 3,
                "三类包络都必须形成闭合多边形");

            // 弯道真实扫掠区应小于自身凸包；旧版把所有角点直接做凸包，
            // 会在弯道内侧生成截图中的长直弦，此断言专门防止该问题回归。
            List<Gssoft.Gscad.Geometry.Point2d> equipmentHull =
                GeometryUtil.ConvexHull(equipment);
            double area = Math.Abs(PolygonArea(equipment));
            double hullArea = Math.Abs(PolygonArea(equipmentHull));
            AssertTrue(area < hullArea - 1e-3 * p.UnitScale * p.UnitScale,
                "转弯设备包络不应退化为凸包长弦");

            CarrierCadGeometry.FitEnvelopeBulges(
                p, equipment, out var fitted, out var bulges);
            bool hasArc = false;
            for (int i = 0; i < bulges.Count; i++)
                if (Math.Abs(bulges[i]) > 1e-6) { hasArc = true; break; }
            AssertTrue(hasArc, "转弯包络出图必须包含真实圆弧段");
            AssertTrue(fitted.Count < equipment.Count,
                "圆弧拟合应减少离散包络顶点");

            for (int i = 0; i < plan.Frames.Count; i++)
            {
                foreach (var point in CarrierCadGeometry.BodyPolygon(p, plan.Frames[i].Pose))
                    AssertInsideOrBoundary(point, equipment, 1e-5, "设备包络漏包车体角点");
                foreach (var point in CarrierCadGeometry.ContainerPolygon(p, plan.Frames[i].Pose))
                    AssertInsideOrBoundary(point, container, 1e-5, "箱体包络漏包角点");
            }

            foreach (var point in equipment)
                AssertInsideOrBoundary(point, clearance, 1e-5, "安全包络漏包设备包络");
            foreach (var point in container)
                AssertInsideOrBoundary(point, clearance, 1e-5, "安全包络漏包箱体包络");

            var body = CarrierCadGeometry.BodyPolygon(p, new CarrierPose());
            double bodyDx = body[0].X - body[3].X;
            double bodyDy = body[0].Y - body[3].Y;
            AssertNear(Math.Sqrt(bodyDx * bodyDx + bodyDy * bodyDy),
                p.OverallLength * p.UnitScale, 1e-7,
                "CAD 单位换算后的设备长度");
        }

        static void VerifyCarrierCrabEnvelope()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            CarrierSegmentPlan plan;
            string error;
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), new[] { 0.0, 0.0, 0.0, 0.0 },
                40.0, 16.0, CarrierSteeringMode.Crab, +1,
                out plan, out error), "蟹行包络测试路径失败：" + error);

            List<Gssoft.Gscad.Geometry.Point2d> equipment =
                CarrierCadGeometry.EquipmentEnvelope(p, plan.Frames);
            List<Gssoft.Gscad.Geometry.Point2d> container =
                CarrierCadGeometry.ContainerEnvelope(p, plan.Frames);
            AssertTrue(!CarrierCadGeometry.EnvelopeFallbackUsed,
                "蟹行包络不应触发凸包失败回退");
            AssertTrue(equipment.Count >= 4 && equipment.Count <= 24,
                "蟹行设备扫掠应为少量直边组成的解析多边形，而不是逐帧锯齿；实际顶点=" +
                equipment.Count);
            AssertTrue(container.Count >= 4 && container.Count <= 8,
                "蟹行箱体扫掠应为少量直边组成的解析多边形");

            CarrierCadGeometry.FitEnvelopeBulges(
                p, equipment, out var fitted, out var bulges);
            for (int i = 0; i < bulges.Count; i++)
                AssertTrue(Math.Abs(bulges[i]) < 1e-9,
                    "蟹行直线包络不应被拟合成圆弧");
            AssertTrue(fitted.Count == equipment.Count,
                "蟹行解析包络不应再做破坏直边的简化");

            List<CarrierFrame> preview = CarrierCadGeometry.PreviewFrames(plan.Frames, 64);
            AssertTrue(preview.Count <= 64, "实时预览帧数必须受上限控制");
            AssertTrue(object.ReferenceEquals(preview[0], plan.Frames[0]) &&
                       object.ReferenceEquals(preview[preview.Count - 1],
                                              plan.Frames[plan.Frames.Count - 1]),
                "预览降采样必须保留首末帧");
        }

        static void VerifyCarrierSegmentedCrabEnvelope()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;

            CarrierSegmentPlan first;
            CarrierSegmentPlan second;
            string error;
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), new[] { 0.0, 0.0, 0.0, 0.0 },
                28.0, 10.0, CarrierSteeringMode.Crab, +1,
                out first, out error), "第一段蟹行路径失败：" + error);
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, first.LastFrame.Pose, WheelAngles(first.LastFrame),
                52.0, -6.0, CarrierSteeringMode.Crab, +1,
                out second, out error), "第二段蟹行路径失败：" + error);

            var segments = new List<List<CarrierFrame>>
            {
                first.Frames,
                second.Frames
            };
            List<Gssoft.Gscad.Geometry.Point2d> equipment =
                CarrierCadGeometry.EquipmentEnvelopeBySegments(p, segments);
            List<Gssoft.Gscad.Geometry.Point2d> container =
                CarrierCadGeometry.ContainerEnvelopeBySegments(p, segments);

            AssertTrue(!CarrierCadGeometry.EnvelopeFallbackUsed,
                "多段蟹行累计包络不应触发凸包失败回退");
            AssertTrue(equipment.Count >= 4 && equipment.Count <= 32,
                "多段蟹行累计设备包络应保持少量直边，不能退化为逐帧锯齿；实际顶点=" +
                equipment.Count);
            AssertTrue(container.Count >= 4 && container.Count <= 20,
                "多段蟹行累计箱体包络应保持少量直边；实际顶点=" +
                container.Count);

            CarrierCadGeometry.FitEnvelopeBulges(
                p, equipment, out var fitted, out var bulges);
            for (int i = 0; i < bulges.Count; i++)
                AssertTrue(Math.Abs(bulges[i]) < 1e-9,
                    "多段蟹行直线包络不应被错误拟合成圆弧");
            AssertTrue(fitted.Count == equipment.Count,
                "多段蟹行解析包络不应被二次简化为锯齿");
        }

        static void VerifyCarrierTurnEnvelopeSmoothing()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            CarrierSegmentPlan plan;
            string error;
            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), new[] { 0.0, 0.0, 0.0, 0.0 },
                10.0, 10.0, CarrierSteeringMode.CounterPhaseFourWheel, +1,
                out plan, out error), "反相四轮平滑测试路径失败：" + error);

            VerifySmoothedEnvelope(
                p, plan.Frames, CarrierCadGeometry.EquipmentEnvelope(p, plan.Frames),
                "设备", 2);
            VerifySmoothedEnvelope(
                p, plan.Frames, CarrierCadGeometry.ContainerEnvelope(p, plan.Frames),
                "箱体", 2);
            VerifySmoothedEnvelope(
                p, plan.Frames, CarrierCadGeometry.ClearanceEnvelope(p, plan.Frames),
                "安全", 2, false);

            AssertTrue(CarrierPathPlanner.TryPlan(
                p, new CarrierPose(), new[] { 0.0, 0.0, 0.0, 0.0 },
                -10.0, 10.0, CarrierSteeringMode.CounterPhaseFourWheel, -1,
                out plan, out error), "倒行反相四轮平滑测试路径失败：" + error);
            VerifySmoothedEnvelope(
                p, plan.Frames, CarrierCadGeometry.EquipmentEnvelope(p, plan.Frames),
                "倒行设备", 2);
            VerifySmoothedEnvelope(
                p, plan.Frames, CarrierCadGeometry.ContainerEnvelope(p, plan.Frames),
                "倒行箱体", 2);
        }

        static void VerifySmoothedEnvelope(
            CarrierParams p, IList<CarrierFrame> frames,
            List<Gssoft.Gscad.Geometry.Point2d> raw, string name, int minimumArcs,
            bool requireZigzagReduction = true)
        {
            CarrierCadGeometry.FitEnvelopeBulges(
                p, raw, frames, out var fitted, out var bulges);
            int arcs = 0;
            for (int i = 0; i < bulges.Count; i++)
                if (Math.Abs(bulges[i]) > 1e-6) arcs++;
            AssertTrue(arcs >= minimumArcs,
                name + "转弯包络圆弧数量不足；实际=" + arcs);
            AssertTrue(fitted.Count < raw.Count,
                name + "圆弧拟合应显著替代离散台阶顶点");

            List<Gssoft.Gscad.Geometry.Point2d> sampled =
                GeometryUtil.SampleBulgePolyline(fitted, bulges, 48);
            double maxTurn = GeometryUtil.MaxTurnAngleDeg(sampled);
            double spikeDepth = MaxReflexSpikeDepth(sampled, 140.0);
            AssertTrue(spikeDepth <= 8e-2 * p.UnitScale,
                name + "包络出现疑似独立尖刺；深度=" +
                (spikeDepth / p.UnitScale).ToString("F3") + "m，最大折返角=" +
                maxTurn.ToString("F2") + "°");
            for (int i = 0; i < raw.Count; i++)
                AssertInsideOrBoundary(
                    raw[i], sampled, 3e-2 * p.UnitScale,
                    name + "平滑圆弧不得向内漏掉原始扫掠边界");
            if (requireZigzagReduction)
            {
                int rawZigzag = CountZigzag(raw);
                int smoothZigzag = CountZigzag(sampled);
                AssertTrue(rawZigzag == 0 ? smoothZigzag == 0 : smoothZigzag < rawZigzag,
                    name + "圆弧拟合后锯齿数必须下降；" + rawZigzag + "→" + smoothZigzag);
            }
        }

        static void VerifyAllCarrierTurnModes()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            p.SteeringCapabilities |= CarrierSteeringCapabilities.FrontOnly |
                                      CarrierSteeringCapabilities.RearOnly |
                                      CarrierSteeringCapabilities.Pivot;
            var modes = new[]
            {
                CarrierSteeringMode.CounterPhaseFourWheel,
                CarrierSteeringMode.FrontOnly,
                CarrierSteeringMode.RearOnly
            };
            int scenarios = 0;
            for (int m = 0; m < modes.Length; m++)
            for (int direction = -1; direction <= 1; direction += 2)
            for (int drive = -1; drive <= 1; drive += 2)
            {
                CarrierMotionCommand command = CarrierMotionCommand.Turn(
                    modes[m], drive, 12.0, direction);
                CarrierTwist twist = CarrierKinematics.CommandToTwist(p, command);
                double duration = (Math.PI * 0.5) / Math.Abs(twist.YawRate);
                double productionStep = (Math.PI / 180.0) / Math.Abs(twist.YawRate);
                List<CarrierFrame> frames = CarrierKinematics.Simulate(
                    p, new CarrierPose(), command, duration, productionStep);
                string label = modes[m] + "｜" +
                    (direction > 0 ? "左" : "右") + "｜" +
                    (drive > 0 ? "前进" : "倒行");
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.EquipmentEnvelope(p, frames),
                    label + "设备", 2);
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.ContainerEnvelope(p, frames),
                    label + "箱体", 2);
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.ClearanceEnvelope(p, frames),
                    label + "安全", 2, false);
                scenarios++;
            }

            for (int direction = -1; direction <= 1; direction += 2)
            {
                CarrierMotionCommand command = CarrierMotionCommand.Pivot(direction * 20.0);
                CarrierTwist twist = CarrierKinematics.CommandToTwist(p, command);
                double productionStep = (Math.PI / 180.0) / Math.Abs(twist.YawRate);
                List<CarrierFrame> frames = CarrierKinematics.Simulate(
                    p, new CarrierPose(), command, 4.5, productionStep);
                string label = "原地回转｜" + (direction > 0 ? "左" : "右");
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.EquipmentEnvelope(p, frames),
                    label + "设备", 2, false);
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.ContainerEnvelope(p, frames),
                    label + "箱体", 2, false);
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.ClearanceEnvelope(p, frames),
                    label + "安全", 2, false);
                scenarios++;
            }
            AssertTrue(scenarios == 14, "转弯平滑矩阵场景数不完整");
        }

        static void VerifyCarrierEnvelopeSpikeSweep()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            p.SteeringCapabilities |= CarrierSteeringCapabilities.FrontOnly |
                                      CarrierSteeringCapabilities.RearOnly;
            var modes = new[]
            {
                CarrierSteeringMode.CounterPhaseFourWheel,
                CarrierSteeringMode.FrontOnly,
                CarrierSteeringMode.RearOnly
            };
            var anglesDeg = new[] { 37.0, 83.0 };
            var radii = new[] { 9.0, 23.0 };
            int scenarios = 0;

            for (int m = 0; m < modes.Length; m++)
            for (int a = 0; a < anglesDeg.Length; a++)
            for (int direction = -1; direction <= 1; direction += 2)
            {
                int drive = ((m + a + (direction > 0 ? 1 : 0)) & 1) == 0 ? 1 : -1;
                double radius = radii[(m + a) % radii.Length];
                CarrierMotionCommand command = CarrierMotionCommand.Turn(
                    modes[m], drive, radius, direction);
                CarrierTwist twist = CarrierKinematics.CommandToTwist(p, command);
                double duration = anglesDeg[a] * Math.PI / 180.0 /
                                  Math.Abs(twist.YawRate);
                double productionStep = (Math.PI / 180.0) /
                                        Math.Abs(twist.YawRate);
                List<CarrierFrame> frames = CarrierKinematics.Simulate(
                    p, new CarrierPose(), command, duration, productionStep);
                string label = modes[m] + "|" + anglesDeg[a].ToString("F0") +
                    "°|R" + radius.ToString("F0") + "|" +
                    (direction > 0 ? "左" : "右") + "|" +
                    (drive > 0 ? "前" : "倒");

                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.EquipmentEnvelope(p, frames),
                    label + "设备", 1);
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.ContainerEnvelope(p, frames),
                    label + "箱体", 1);
                VerifySmoothedEnvelope(
                    p, frames, CarrierCadGeometry.ClearanceEnvelope(p, frames),
                    label + "安全", 1, false);
                scenarios++;
            }
            AssertTrue(scenarios == 12, "尖刺扫描场景数不完整");
        }

        static void VerifyCarrierPreviewQuality()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;
            p.SteeringCapabilities |= CarrierSteeringCapabilities.FrontOnly |
                                      CarrierSteeringCapabilities.RearOnly;
            var modes = new[]
            {
                CarrierSteeringMode.CounterPhaseFourWheel,
                CarrierSteeringMode.FrontOnly,
                CarrierSteeringMode.RearOnly
            };

            int scenarios = 0;
            for (int m = 0; m < modes.Length; m++)
            for (int direction = -1; direction <= 1; direction += 2)
            for (int drive = -1; drive <= 1; drive += 2)
            {
                double radius = m == 0 ? 9.0 : 17.0;
                CarrierMotionCommand command = CarrierMotionCommand.Turn(
                    modes[m], drive, radius, direction);
                CarrierTwist twist = CarrierKinematics.CommandToTwist(p, command);
                double duration = 170.0 * Math.PI / 180.0 /
                                  Math.Abs(twist.YawRate);
                double fullStep = (0.25 * Math.PI / 180.0) /
                                  Math.Abs(twist.YawRate);
                List<CarrierFrame> fullFrames = CarrierKinematics.Simulate(
                    p, new CarrierPose(), command, duration, fullStep);
                List<CarrierFrame> previewFrames =
                    CarrierCadGeometry.PreviewFrames(fullFrames, 160);
                AssertTrue(previewFrames.Count <= 160,
                    "实时预览关键帧不得超过160帧");

                string label = modes[m] + "|" +
                    (direction > 0 ? "左" : "右") + "|" +
                    (drive > 0 ? "前" : "倒");
                AssertPreviewMatchesFull(
                    CarrierCadGeometry.EquipmentEnvelope(p, fullFrames),
                    CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.EquipmentEnvelope(p, previewFrames),
                        previewFrames),
                    p, label + "设备");
                AssertPreviewMatchesFull(
                    CarrierCadGeometry.ClearanceEnvelope(p, fullFrames),
                    CarrierCadGeometry.SmoothEnvelopePreview(
                        p, CarrierCadGeometry.ClearanceEnvelope(p, previewFrames),
                        previewFrames),
                    p, label + "安全");
                scenarios++;
            }
            AssertTrue(scenarios == 12, "实时预览质量扫描场景数不完整");
        }

        static void VerifyCarrierAdaptiveCumulativePreview()
        {
            CarrierParams p = CarrierParams.GenericFourWheelIndependent();
            p.ContainerPreset = CarrierContainerPreset.Iso40Gp;

            CarrierPose pose = new CarrierPose();
            var segments = new List<List<CarrierFrame>>();
            List<CarrierFrame> straight1 = CarrierKinematics.Simulate(
                p, pose, CarrierMotionCommand.Longitudinal(1.0), 18.0, 0.05);
            segments.Add(straight1);
            pose = Last(straight1).Pose;

            CarrierMotionCommand left = CarrierMotionCommand.Turn(
                CarrierSteeringMode.CounterPhaseFourWheel, 1.0, 12.0, +1);
            CarrierTwist leftTwist = CarrierKinematics.CommandToTwist(p, left);
            List<CarrierFrame> turn1 = CarrierKinematics.Simulate(
                p, pose, left, (Math.PI * 0.5) / Math.Abs(leftTwist.YawRate),
                (0.25 * Math.PI / 180.0) / Math.Abs(leftTwist.YawRate));
            segments.Add(turn1);
            pose = Last(turn1).Pose;

            List<CarrierFrame> straight2 = CarrierKinematics.Simulate(
                p, pose, CarrierMotionCommand.Longitudinal(1.0), 12.0, 0.05);
            segments.Add(straight2);
            pose = Last(straight2).Pose;

            CarrierMotionCommand right = CarrierMotionCommand.Turn(
                CarrierSteeringMode.CounterPhaseFourWheel, 1.0, 12.0, -1);
            CarrierTwist rightTwist = CarrierKinematics.CommandToTwist(p, right);
            List<CarrierFrame> turn2 = CarrierKinematics.Simulate(
                p, pose, right, (Math.PI * 0.5) / Math.Abs(rightTwist.YawRate),
                (0.25 * Math.PI / 180.0) / Math.Abs(rightTwist.YawRate));
            segments.Add(turn2);

            List<List<CarrierFrame>> preview =
                CarrierCadGeometry.PreviewSegmentsAdaptive(segments, 192);
            int total = 0;
            for (int i = 0; i < preview.Count; i++) total += preview[i].Count;
            AssertTrue(total <= 192, "累计预览不得超过192帧；实际=" + total);
            AssertTrue(preview[0].Count == 2 && preview[2].Count == 2,
                "解析直行段只应占用首末两帧");
            AssertTrue(preview[1].Count >= 80 && preview[3].Count >= 80,
                "剩余关键帧应优先分配给两个转弯段");

            var fitFrames = new List<CarrierFrame>();
            for (int i = 0; i < preview.Count; i++)
                for (int j = 0; j < preview[i].Count; j++)
                    if (fitFrames.Count == 0 || j > 0) fitFrames.Add(preview[i][j]);

            List<Gssoft.Gscad.Geometry.Point2d> raw =
                CarrierCadGeometry.EquipmentEnvelopeBySegments(p, preview);
            CarrierCadGeometry.FitEnvelopeBulgesInteractive(
                p, raw, fitFrames, out var fitted, out var bulges);
            int arcs = 0;
            for (int i = 0; i < bulges.Count; i++)
                if (Math.Abs(bulges[i]) > 1e-6) arcs++;
            AssertTrue(arcs >= 2, "累计交互预览应恢复内外侧安全圆弧");

            List<Gssoft.Gscad.Geometry.Point2d> sampled =
                GeometryUtil.SampleBulgePolyline(fitted, bulges, 48);
            for (int i = 0; i < raw.Count; i++)
                AssertInsideOrBoundary(raw[i], sampled, 2e-2 * p.UnitScale,
                    "交互支撑圆弧不得漏包降采样边界");
            AssertTrue(CountZigzag(sampled) < CountZigzag(raw),
                "交互安全圆弧应减少累计包络锯齿");
        }

        static void AssertPreviewMatchesFull(
            List<Gssoft.Gscad.Geometry.Point2d> full,
            List<Gssoft.Gscad.Geometry.Point2d> preview,
            CarrierParams p, string name)
        {
            AssertTrue(preview != null && preview.Count >= 3,
                name + "实时预览不得为空");
            for (int i = 0; i < full.Count; i++)
                AssertInsideOrBoundary(
                    full[i], preview, 15e-2 * p.UnitScale,
                    name + "160帧临时预览与全量包络偏差超过15cm");
        }

        static double MaxReflexSpikeDepth(
            List<Gssoft.Gscad.Geometry.Point2d> points, double minimumTurnDeg)
        {
            if (points == null || points.Count < 3) return 0.0;
            double maximum = 0.0;
            for (int i = 0; i < points.Count; i++)
            {
                var a = points[(i - 1 + points.Count) % points.Count];
                var b = points[i];
                var c = points[(i + 1) % points.Count];
                double e1x = b.X - a.X, e1y = b.Y - a.Y;
                double e2x = c.X - b.X, e2y = c.Y - b.Y;
                double turn = Math.Abs(Math.Atan2(
                    e1x * e2y - e1y * e2x,
                    e1x * e2x + e1y * e2y)) * 180.0 / Math.PI;
                if (turn < minimumTurnDeg) continue;
                double depth = GeometryUtil.DistancePointToSegment(b, a, c).dist;
                if (depth > maximum) maximum = depth;
            }
            return maximum;
        }

        static void AssertInsideOrBoundary(
            Gssoft.Gscad.Geometry.Point2d point,
            List<Gssoft.Gscad.Geometry.Point2d> polygon,
            double tolerance, string message)
        {
            if (GeometryUtil.PointInClosedPolygon(point, polygon)) return;
            for (int i = 0; i < polygon.Count; i++)
            {
                var a = polygon[i];
                var b = polygon[(i + 1) % polygon.Count];
                double vx = b.X - a.X, vy = b.Y - a.Y;
                double len2 = vx * vx + vy * vy;
                double t = len2 < 1e-20 ? 0.0 :
                    ((point.X - a.X) * vx + (point.Y - a.Y) * vy) / len2;
                t = Math.Max(0.0, Math.Min(1.0, t));
                double dx = point.X - (a.X + t * vx);
                double dy = point.Y - (a.Y + t * vy);
                if (Math.Sqrt(dx * dx + dy * dy) <= tolerance) return;
            }
            throw new InvalidOperationException(message);
        }

        static CarrierFrame Last(List<CarrierFrame> frames)
        {
            AssertTrue(frames != null && frames.Count > 0, "仿真不得返回空帧");
            return frames[frames.Count - 1];
        }

        static void AssertWheelResidual(CarrierFrame frame)
        {
            double residual = CarrierKinematics.MaxWheelVelocityResidual(frame.Wheels, frame.Twist);
            AssertTrue(residual < 1e-10, "轮心速度残差过大：" + residual.ToString("G17"));
        }

        static void AssertDimensions(
            CarrierParams p, CarrierContainerPreset preset,
            double length, double width, double height)
        {
            p.ContainerPreset = preset;
            CarrierContainerDimensions dims = p.ContainerDimensions();
            AssertNear(dims.Length, length, CarrierTol, preset + " 长度");
            AssertNear(dims.Width, width, CarrierTol, preset + " 宽度");
            AssertNear(dims.Height, height, CarrierTol, preset + " 高度");
        }

        static void ExpectCarrierFailure(Action action, string message)
        {
            try
            {
                action();
            }
            catch (CarrierKinematicException)
            {
                return;
            }
            throw new InvalidOperationException(message);
        }

        static double Distance(double[,] points, int a, int b)
        {
            double dx = points[a, 0] - points[b, 0];
            double dy = points[a, 1] - points[b, 1];
            return Math.Sqrt(dx * dx + dy * dy);
        }

        static void AssertNear(double actual, double expected, double tolerance, string message)
        {
            if (Math.Abs(actual - expected) > tolerance)
                throw new InvalidOperationException(string.Format(
                    "{0}：actual={1:G17}, expected={2:G17}, tol={3:G3}",
                    message, actual, expected, tolerance));
        }

        static void AssertTrue(bool value, string message)
        {
            if (!value) throw new InvalidOperationException(message);
        }
    }
}
