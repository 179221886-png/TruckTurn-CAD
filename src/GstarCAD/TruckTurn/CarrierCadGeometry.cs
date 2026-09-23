using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>跨运车米制运动帧与 CAD 图形单位之间的唯一转换层。</summary>
    internal static class CarrierCadGeometry
    {
        public static bool EnvelopeFallbackUsed { get; private set; }

        public static Point2d ToCad(double xMeters, double yMeters, CarrierParams p)
        {
            return new Point2d(xMeters * p.UnitScale, yMeters * p.UnitScale);
        }

        public static CarrierPose ToMeterPose(Point3d pointCad, double headingRad, CarrierParams p)
        {
            return new CarrierPose(
                pointCad.X / p.UnitScale, pointCad.Y / p.UnitScale, headingRad);
        }

        public static List<Point2d> BodyPolygon(CarrierParams p, CarrierPose pose)
        {
            return ToCadPoints(BodyPolygonMeters(p, pose), p);
        }

        public static List<Point2d> ContainerPolygon(CarrierParams p, CarrierPose pose)
        {
            return ToCadPoints(ContainerPolygonMeters(p, pose), p);
        }

        public static List<Point2d> WheelPolygon(CarrierParams p, CarrierFrame frame, int wheelIndex)
        {
            return ToCadPoints(WheelPolygonMeters(p, frame, wheelIndex), p);
        }

        static Point2d[] WheelPolygonMeters(CarrierParams p, CarrierFrame frame, int wheelIndex)
        {
            CarrierWheelState wheel = frame.Wheels[wheelIndex];
            double angle = frame.Pose.HeadingRad + wheel.SteerAngleRad;
            double ux = Math.Cos(angle), uy = Math.Sin(angle);
            double nx = -uy, ny = ux;
            double hl = p.WheelLength * 0.5;
            double hw = p.WheelWidth * 0.5;
            return new[]
            {
                new Point2d(wheel.WorldX - hl * ux - hw * nx,
                    wheel.WorldY - hl * uy - hw * ny),
                new Point2d(wheel.WorldX - hl * ux + hw * nx,
                    wheel.WorldY - hl * uy + hw * ny),
                new Point2d(wheel.WorldX + hl * ux + hw * nx,
                    wheel.WorldY + hl * uy + hw * ny),
                new Point2d(wheel.WorldX + hl * ux - hw * nx,
                    wheel.WorldY + hl * uy - hw * ny)
            };
        }

        public static List<Point2d> ReferencePath(CarrierParams p, IList<CarrierFrame> frames)
        {
            var result = new List<Point2d>();
            if (frames == null) return result;
            for (int i = 0; i < frames.Count; i++)
            {
                Point2d pt = ToCad(frames[i].Pose.X, frames[i].Pose.Y, p);
                if (result.Count == 0 || Distance(result[result.Count - 1], pt) > 1e-7)
                    result.Add(pt);
            }
            return result;
        }

        public static List<Point2d>[] WheelTracks(CarrierParams p, IList<CarrierFrame> frames)
        {
            var tracks = new[]
            {
                new List<Point2d>(), new List<Point2d>(),
                new List<Point2d>(), new List<Point2d>()
            };
            if (frames == null) return tracks;
            for (int f = 0; f < frames.Count; f++)
            {
                for (int w = 0; w < Math.Min(4, frames[f].Wheels.Length); w++)
                {
                    Point2d pt = ToCad(
                        frames[f].Wheels[w].WorldX, frames[f].Wheels[w].WorldY, p);
                    if (tracks[w].Count == 0 || Distance(tracks[w][tracks[w].Count - 1], pt) > 1e-7)
                        tracks[w].Add(pt);
                }
            }
            return tracks;
        }

        public static List<Point2d> EquipmentEnvelope(CarrierParams p, IList<CarrierFrame> frames)
        {
            return UnionToCad(BuildEquipmentFootprintsMeters(p, frames), p);
        }

        /// <summary>
        /// 按已确认运动段构造累计包络。分段保留后，直行/横移/蟹行可继续使用
        /// 首末姿态解析扫掠凸包，避免把多段拼成一列帧后退化为逐帧锯齿。
        /// </summary>
        public static List<Point2d> EquipmentEnvelopeBySegments(
            CarrierParams p, IList<List<CarrierFrame>> segments)
        {
            return UnionToCad(BuildEquipmentFootprintsForSegments(p, segments), p);
        }

        public static List<Point2d> ContainerEnvelope(CarrierParams p, IList<CarrierFrame> frames)
        {
            if (frames == null || p.ContainerPreset == CarrierContainerPreset.None)
                return new List<Point2d>();
            return UnionToCad(BuildContainerFootprintsMeters(p, frames), p);
        }

        public static List<Point2d> ContainerEnvelopeBySegments(
            CarrierParams p, IList<List<CarrierFrame>> segments)
        {
            if (segments == null || p.ContainerPreset == CarrierContainerPreset.None)
                return new List<Point2d>();
            return UnionToCad(BuildContainerFootprintsForSegments(p, segments), p);
        }

        public static List<Point2d> ClearanceEnvelope(CarrierParams p, IList<CarrierFrame> frames)
        {
            var polygons = BuildEquipmentFootprintsMeters(p, frames);
            if (frames != null && p.ContainerPreset != CarrierContainerPreset.None)
                polygons.AddRange(BuildContainerFootprintsMeters(p, frames));
            return ClearanceEnvelopeFromPolygons(p, polygons, frames);
        }

        public static List<Point2d> ClearanceEnvelopeBySegments(
            CarrierParams p, IList<List<CarrierFrame>> segments,
            IList<CarrierFrame> fitFrames)
        {
            var polygons = BuildEquipmentFootprintsForSegments(p, segments);
            if (segments != null && p.ContainerPreset != CarrierContainerPreset.None)
                polygons.AddRange(BuildContainerFootprintsForSegments(p, segments));
            return ClearanceEnvelopeFromPolygons(p, polygons, fitFrames);
        }

        static List<Point2d> ClearanceEnvelopeFromPolygons(
            CarrierParams p, List<Point2d[]> polygons, IList<CarrierFrame> frames)
        {
            List<Point2d> envelopeMeters = UnionMeters(polygons);
            if (envelopeMeters.Count >= 3 && p.SafetyMargin > 1e-12)
            {
                // 安全线不能直接偏移离散并集的每个台阶，否则每个 1° 采样齿都会被
                // 原样放大到红线中。先复用货车指令的已知瞬心圆弧拟合，把真实扫掠
                // 边界恢复为圆弧，再对圆弧采样环做等距外扩；出图阶段还会再次拟合
                // 为 CAD bulge 圆弧。直行/蟹行没有瞬心，原始解析包络本来就是直线。
                List<Point2d> centers = TurnCentersMeters(frames);
                if (centers.Count > 0)
                {
                    double fitTol = EnvelopeFitToleranceMeters(p, frames);
                    FitTurningEnvelopeMeters(
                        envelopeMeters, centers, fitTol, 2e-2, 2e-2,
                        out List<Point2d> fitted, out List<double> bulges);
                    bool hasArc = false;
                    for (int i = 0; bulges != null && i < bulges.Count; i++)
                        if (Math.Abs(bulges[i]) > 1e-9) { hasArc = true; break; }
                    if (hasArc)
                        envelopeMeters = GeometryUtil.SampleBulgePolyline(
                            fitted, bulges, 32);
                }
                envelopeMeters = GeometryUtil.OffsetClosedLoop(
                    envelopeMeters, p.SafetyMargin);
            }
            return ToCadPoints(envelopeMeters, p);
        }

        /// <summary>
        /// 把离散扫掠边界中连续的小弦段拟合为 CAD bulge 圆弧。
        /// 所有容差先在米制坐标下计算，保证米图和毫米图得到相同结果。
        /// </summary>
        public static void FitEnvelopeBulges(
            CarrierParams p, List<Point2d> envelopeCad,
            out List<Point2d> fittedCad, out List<double> bulges)
        {
            FitEnvelopeBulges(p, envelopeCad, null, out fittedCad, out bulges);
        }

        /// <summary>
        /// 使用运动帧反算真实瞬时转动中心，再按该中心拟合扫掠圆弧。
        /// 反相四轮转弯的每个车体/箱体角点都绕同一世界坐标瞬心运动，
        /// 因而已知圆心拟合比通用 Kasa 拟合更稳定，也能消除内外侧台阶。
        /// </summary>
        public static void FitEnvelopeBulges(
            CarrierParams p, List<Point2d> envelopeCad, IList<CarrierFrame> frames,
            out List<Point2d> fittedCad, out List<double> bulges)
        {
            FitEnvelopeBulgesCore(
                p, envelopeCad, frames, false, out fittedCad, out bulges);
        }

        /// <summary>
        /// 交互累计包络专用拟合。预览帧经过降采样后，离散台阶的径向振幅会比
        /// 最终全量帧更大；这里仍使用 rMax/rMin 安全支撑圆弧，但允许弧端点在
        /// 安全方向作与预览步长相称的径向补偿，从而避免内侧弧因 2cm 端点限制
        /// 回退成锯齿折线。最终 X 出图仍调用常规严格拟合。
        /// </summary>
        public static void FitEnvelopeBulgesInteractive(
            CarrierParams p, List<Point2d> envelopeCad, IList<CarrierFrame> frames,
            out List<Point2d> fittedCad, out List<double> bulges)
        {
            FitEnvelopeBulgesCore(
                p, envelopeCad, frames, true, out fittedCad, out bulges);
        }

        static void FitEnvelopeBulgesCore(
            CarrierParams p, List<Point2d> envelopeCad, IList<CarrierFrame> frames,
            bool interactivePreview,
            out List<Point2d> fittedCad, out List<double> bulges)
        {
            var meters = new List<Point2d>();
            if (envelopeCad != null)
                for (int i = 0; i < envelopeCad.Count; i++)
                    meters.Add(new Point2d(
                        envelopeCad[i].X / p.UnitScale,
                        envelopeCad[i].Y / p.UnitScale));

            List<Point2d> centers = TurnCentersMeters(frames);
            double fitTol = EnvelopeFitToleranceMeters(p, frames);
            double maxPush = 2e-2;
            double maxSag = 2e-2;
            if (interactivePreview)
            {
                // fitTol 已按「外廓半对角线 × 预览帧间转角」估算台阶幅度。
                // 支撑圆弧本身按材料侧选择 rMax/rMin，不会向内漏包；这里仅放宽
                // 接缝端点和弦矢限制，并设置上限，防止把真实长直边误判成圆弧。
                maxPush = CarrierMath.Clamp(fitTol, 2e-2, 0.16);
                maxSag = CarrierMath.Clamp(fitTol * 0.60, 2e-2, 0.10);
            }
            FitTurningEnvelopeMeters(
                meters, centers, fitTol, maxPush, maxSag,
                out List<Point2d> fittedMeters, out bulges);
            fittedCad = ToCadPoints(fittedMeters, p);
        }

        /// <summary>
        /// 优先用货车包络同等的 5cm 直线段稀化消除短齿，但每次都重新验证
        /// 输出仍外包原始扫掠。个别小半径/单桥转向在 5cm 容差下可能略微内缩，
        /// 此时自动退到 2cm，再不满足则关闭额外稀化。
        /// </summary>
        static void FitTurningEnvelopeMeters(
            List<Point2d> meters, List<Point2d> centers, double fitTol,
            double maxPush, double maxSag,
            out List<Point2d> fittedMeters, out List<double> bulges)
        {
            double[] collapseTolerances = { 5e-2, 2e-2, 1e-4 };
            fittedMeters = meters == null ? new List<Point2d>() : new List<Point2d>(meters);
            bulges = new List<double>();

            for (int attempt = 0; attempt < collapseTolerances.Length; attempt++)
            {
                GeometryUtil.FitEnvelopeBulges(
                    meters, centers, out fittedMeters, out bulges,
                    fitTol, 4.0, 4, maxPush, maxSag, 1500, 8.0,
                    collapseTolerances[attempt]);
                if (fittedMeters == null || fittedMeters.Count < 3) continue;
                if (EnvelopeContains(
                    meters, GeometryUtil.SampleBulgePolyline(fittedMeters, bulges, 48),
                    2e-2))
                    return;
            }
        }

        static bool EnvelopeContains(
            List<Point2d> source, List<Point2d> candidate, double tolerance)
        {
            if (source == null || source.Count == 0) return true;
            if (candidate == null || candidate.Count < 3) return false;
            for (int i = 0; i < source.Count; i++)
            {
                Point2d point = source[i];
                if (GeometryUtil.PointInClosedPolygon(point, candidate)) continue;
                if (GeometryUtil.DistancePointToPolyline(point, candidate).dist <= tolerance)
                    continue;
                return false;
            }
            return true;
        }

        /// <summary>
        /// WorldDraw 不支持 bulge，故把拟合后的圆弧适度采样为光滑预览折线。
        /// 正式 CAD 图元仍保存为真正的 bulge 圆弧，不是这些预览小线段。
        /// </summary>
        public static List<Point2d> SmoothEnvelopePreview(
            CarrierParams p, List<Point2d> envelopeCad, IList<CarrierFrame> frames)
        {
            FitEnvelopeBulgesInteractive(
                p, envelopeCad, frames,
                out List<Point2d> fitted, out List<double> bulges);
            if (fitted == null || fitted.Count < 3) return envelopeCad;
            bool hasArc = false;
            for (int i = 0; bulges != null && i < bulges.Count; i++)
                if (Math.Abs(bulges[i]) > 1e-9) { hasArc = true; break; }
            return hasArc
                ? GeometryUtil.SampleBulgePolyline(fitted, bulges, 24)
                : fitted;
        }

        static List<Point2d> TurnCentersMeters(IList<CarrierFrame> frames)
        {
            var result = new List<Point2d>();
            if (frames == null) return result;
            for (int i = 0; i < frames.Count; i++)
            {
                CarrierFrame frame = frames[i];
                double yaw = frame.Twist.YawRate;
                if (Math.Abs(yaw) < 1e-9) continue;

                double localX = -frame.Twist.Vy / yaw;
                double localY = frame.Twist.Vx / yaw;
                double c = Math.Cos(frame.Pose.HeadingRad);
                double s = Math.Sin(frame.Pose.HeadingRad);
                var center = new Point2d(
                    frame.Pose.X + c * localX - s * localY,
                    frame.Pose.Y + s * localX + c * localY);

                bool duplicate = false;
                for (int j = 0; j < result.Count; j++)
                    if (Distance(result[j], center) < 1e-4)
                    {
                        duplicate = true;
                        break;
                    }
                if (!duplicate) result.Add(center);
            }
            return result;
        }

        static double EnvelopeFitToleranceMeters(
            CarrierParams p, IList<CarrierFrame> frames)
        {
            double maxHeadingStep = 0.0;
            if (frames != null)
                for (int i = 1; i < frames.Count; i++)
                {
                    double step = Math.Abs(CarrierMath.NormalizeAngleRad(
                        frames[i].Pose.HeadingRad - frames[i - 1].Pose.HeadingRad));
                    if (step > maxHeadingStep) maxHeadingStep = step;
                }

            double length = p.OverallLength;
            double width = p.OverallWidth;
            CarrierContainerDimensions box = p.ContainerDimensions();
            if (box.Length > length) length = box.Length;
            if (box.Width > width) width = box.Width;
            double halfDiagonal = 0.5 * Math.Sqrt(length * length + width * width);

            // 离散旋转一帧形成的台阶振幅与「外廓半对角线 × 帧间转角」同阶。
            // 货车默认 3cm 对应 0.25° 细采样；跨运车交互通常为 1°，预览降采样
            // 后还可能略大，因此按实际角步放宽，但封顶 18cm 防止吞掉真实端盖折角。
            double adaptive = halfDiagonal * Math.Sin(maxHeadingStep) + 0.01;
            return CarrierMath.Clamp(adaptive, 3e-2, 0.18);
        }

        /// <summary>
        /// 实时预览只保留均匀分布的关键帧；最终确认出图仍使用全部帧。
        /// 首末帧始终保留，避免视觉终点与实际确认终点不一致。
        /// </summary>
        public static List<CarrierFrame> PreviewFrames(
            IList<CarrierFrame> frames, int maximumFrames = 64)
        {
            var result = new List<CarrierFrame>();
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
                if (index != previous)
                {
                    result.Add(frames[index]);
                    previous = index;
                }
            }
            return result;
        }

        /// <summary>
        /// 为累计包络按运动难度分配固定总预算。直行、横移、蟹行的扫掠由首末姿态
        /// 解析得到，只保留两帧；剩余预算按各转弯段的累计车身转角动态分配，避免
        /// 旧的“每段平均分配”在直线段浪费帧、让转弯内侧出现大台阶。
        /// </summary>
        public static List<List<CarrierFrame>> PreviewSegmentsAdaptive(
            IList<List<CarrierFrame>> segments, int maximumTotalFrames)
        {
            var result = new List<List<CarrierFrame>>();
            if (segments == null || segments.Count == 0) return result;

            int count = segments.Count;
            var allocations = new int[count];
            var difficulties = new double[count];
            int allocated = 0;
            for (int i = 0; i < count; i++)
            {
                IList<CarrierFrame> segment = segments[i];
                int frameCount = segment == null ? 0 : segment.Count;
                if (frameCount == 0) continue;

                bool straight = IsStraightTranslation(segment);
                int minimum = Math.Min(frameCount, straight ? 2 : 4);
                allocations[i] = minimum;
                allocated += minimum;
                difficulties[i] = straight ? 0.0 : SegmentTurnDifficulty(segment);
            }

            int budget = Math.Max(allocated, maximumTotalFrames);
            while (allocated < budget)
            {
                int best = -1;
                double bestScore = double.MinValue;
                for (int i = 0; i < count; i++)
                {
                    IList<CarrierFrame> segment = segments[i];
                    int frameCount = segment == null ? 0 : segment.Count;
                    if (allocations[i] >= frameCount) continue;

                    // 每增加一帧就重新计算当前“每个采样间隔承担的转角”，
                    // 总是优先补给最稀疏的转弯段。全直线的特殊情况按长度兜底。
                    double score = difficulties[i] > 1e-12
                        ? difficulties[i] / Math.Max(1, allocations[i] - 1)
                        : 1e-12 / Math.Max(1, allocations[i]);
                    if (score > bestScore)
                    {
                        bestScore = score;
                        best = i;
                    }
                }
                if (best < 0) break;
                allocations[best]++;
                allocated++;
            }

            for (int i = 0; i < count; i++)
                result.Add(PreviewFrames(segments[i], allocations[i]));
            return result;
        }

        static double SegmentTurnDifficulty(IList<CarrierFrame> frames)
        {
            if (frames == null || frames.Count < 2) return 0.0;
            double headingTravel = 0.0;
            double steeringTravel = 0.0;
            for (int i = 1; i < frames.Count; i++)
            {
                headingTravel += Math.Abs(CarrierMath.NormalizeAngleRad(
                    frames[i].Pose.HeadingRad - frames[i - 1].Pose.HeadingRad));
                int wheels = Math.Min(frames[i - 1].Wheels.Length, frames[i].Wheels.Length);
                double frameSteeringChange = 0.0;
                for (int w = 0; w < wheels; w++)
                    frameSteeringChange = Math.Max(frameSteeringChange,
                        Math.Abs(CarrierMath.NormalizeAngleRad(
                            frames[i].Wheels[w].SteerAngleRad -
                            frames[i - 1].Wheels[w].SteerAngleRad)));
                steeringTravel += frameSteeringChange;
            }
            // 车身转角直接决定扫掠曲线密度；少量加入轮角变化，确保前/后桥
            // 摆轮且车轮可能伸出外廓时仍保留必要关键帧。
            return Math.Max(headingTravel + 0.20 * steeringTravel, 1e-9);
        }

        public static string ModeName(CarrierSteeringMode mode)
        {
            switch (mode)
            {
                case CarrierSteeringMode.Longitudinal: return "纵向";
                case CarrierSteeringMode.FrontOnly: return "前桥转向";
                case CarrierSteeringMode.RearOnly: return "后桥转向";
                case CarrierSteeringMode.CounterPhaseFourWheel: return "反相四轮";
                case CarrierSteeringMode.Crab: return "蟹行";
                case CarrierSteeringMode.Lateral: return "横移";
                case CarrierSteeringMode.Pivot: return "原地回转";
                default: return mode.ToString();
            }
        }

        static Point2d[] BodyPolygonMeters(CarrierParams p, CarrierPose pose)
        {
            return OrderedRectangleMeters(CarrierKinematics.BodyCorners(p, pose));
        }

        static Point2d[] ContainerPolygonMeters(CarrierParams p, CarrierPose pose)
        {
            double[,] points = CarrierKinematics.ContainerCorners(p, pose);
            return points.GetLength(0) == 4
                ? OrderedRectangleMeters(points)
                : new Point2d[0];
        }

        static Point2d[] OrderedRectangleMeters(double[,] points)
        {
            // CarrierKinematics 的角点生成顺序为 0,1,2,3；周界顺序应为 0,1,3,2。
            int[] order = { 0, 1, 3, 2 };
            var result = new Point2d[4];
            for (int i = 0; i < order.Length; i++)
                result[i] = new Point2d(points[order[i], 0], points[order[i], 1]);
            return result;
        }

        static List<Point2d[]> BuildEquipmentFootprintsForSegments(
            CarrierParams p, IList<List<CarrierFrame>> segments)
        {
            var polygons = new List<Point2d[]>();
            if (segments == null) return polygons;
            for (int i = 0; i < segments.Count; i++)
                polygons.AddRange(BuildEquipmentFootprintsMeters(p, segments[i]));
            return polygons;
        }

        static List<Point2d[]> BuildContainerFootprintsForSegments(
            CarrierParams p, IList<List<CarrierFrame>> segments)
        {
            var polygons = new List<Point2d[]>();
            if (segments == null) return polygons;
            for (int i = 0; i < segments.Count; i++)
                polygons.AddRange(BuildContainerFootprintsMeters(p, segments[i]));
            return polygons;
        }

        static List<Point2d[]> BuildEquipmentFootprintsMeters(
            CarrierParams p, IList<CarrierFrame> frames)
        {
            var polygons = new List<Point2d[]>();
            if (frames == null || frames.Count == 0) return polygons;

            // 直行、横移和蟹行都属于固定姿态沿直线平移。对每个固定部件取
            // 首末位置（以及停车摆轮过程）的凸包，就是解析扫掠结果：侧边严格为直线，
            // 且只需处理少量多边形，不会产生逐帧并集的锯齿和卡顿。
            if (IsStraightTranslation(frames))
            {
                polygons.Add(EndpointSweptHull(frames, delegate(CarrierFrame frame)
                {
                    return BodyPolygonMeters(p, frame.Pose);
                }));

                int wheelCount = Math.Min(4, frames[0].Wheels.Length);
                for (int w = 0; w < wheelCount; w++)
                {
                    int wheelIndex = w;
                    bool protrudes = false;
                    for (int i = 0; i < frames.Count && !protrudes; i++)
                    {
                        Point2d[] wheel = WheelPolygonMeters(p, frames[i], wheelIndex);
                        protrudes = !InsideBody(p, frames[i].Pose, wheel);
                    }
                    if (protrudes)
                        polygons.Add(SweptHull(frames, delegate(CarrierFrame frame)
                        {
                            return WheelPolygonMeters(p, frame, wheelIndex);
                        }));
                }
                return polygons;
            }

            Point2d[] previousBody = null;
            var previousWheels = new Point2d[4][];
            for (int i = 0; i < frames.Count; i++)
            {
                CarrierFrame frame = frames[i];
                Point2d[] body = BodyPolygonMeters(p, frame.Pose);
                AddIfChanged(polygons, body, ref previousBody);
                for (int w = 0; w < Math.Min(4, frame.Wheels.Length); w++)
                {
                    Point2d[] wheel = WheelPolygonMeters(p, frame, w);
                    // OverallLength/OverallWidth 表示整机外廓；默认车轮完全位于该外廓内，
                    // 无需把同一帧从 1 个多边形膨胀成 5 个。仅当用户参数让车轮伸出
                    // 整机外廓时，才把该轮纳入扫掠并集。
                    if (!InsideBody(p, frame.Pose, wheel))
                        AddIfChanged(polygons, wheel, ref previousWheels[w]);
                }
            }
            return polygons;
        }

        static List<Point2d[]> BuildContainerFootprintsMeters(
            CarrierParams p, IList<CarrierFrame> frames)
        {
            var polygons = new List<Point2d[]>();
            if (frames == null || frames.Count == 0 ||
                p.ContainerPreset == CarrierContainerPreset.None)
                return polygons;

            if (IsStraightTranslation(frames))
            {
                polygons.Add(EndpointSweptHull(frames, delegate(CarrierFrame frame)
                {
                    return ContainerPolygonMeters(p, frame.Pose);
                }));
                return polygons;
            }

            Point2d[] previous = null;
            for (int i = 0; i < frames.Count; i++)
            {
                Point2d[] polygon = ContainerPolygonMeters(p, frames[i].Pose);
                AddIfChanged(polygons, polygon, ref previous);
            }
            return polygons;
        }

        static bool IsStraightTranslation(IList<CarrierFrame> frames)
        {
            if (frames == null || frames.Count < 2) return false;
            CarrierPose first = frames[0].Pose;
            CarrierPose last = frames[frames.Count - 1].Pose;
            double vx = last.X - first.X;
            double vy = last.Y - first.Y;
            double len = Math.Sqrt(vx * vx + vy * vy);
            if (len < 1e-9) return false;

            for (int i = 0; i < frames.Count; i++)
            {
                CarrierPose pose = frames[i].Pose;
                if (Math.Abs(CarrierMath.NormalizeAngleRad(
                    pose.HeadingRad - first.HeadingRad)) > 1e-9)
                    return false;
                double px = pose.X - first.X;
                double py = pose.Y - first.Y;
                if (Math.Abs(px * vy - py * vx) > 1e-8 * Math.Max(1.0, len))
                    return false;
            }
            return true;
        }

        static Point2d[] SweptHull(
            IList<CarrierFrame> frames, Func<CarrierFrame, Point2d[]> polygonAt)
        {
            var points = new List<Point2d>();
            for (int i = 0; i < frames.Count; i++)
            {
                Point2d[] polygon = polygonAt(frames[i]);
                if (polygon != null) points.AddRange(polygon);
            }
            List<Point2d> hull = GeometryUtil.ConvexHull(points);
            return hull.ToArray();
        }

        static Point2d[] EndpointSweptHull(
            IList<CarrierFrame> frames, Func<CarrierFrame, Point2d[]> polygonAt)
        {
            var points = new List<Point2d>();
            points.AddRange(polygonAt(frames[0]));
            points.AddRange(polygonAt(frames[frames.Count - 1]));
            return GeometryUtil.ConvexHull(points).ToArray();
        }

        static bool InsideBody(CarrierParams p, CarrierPose pose, Point2d[] polygon)
        {
            double c = Math.Cos(pose.HeadingRad);
            double s = Math.Sin(pose.HeadingRad);
            double halfLength = p.OverallLength * 0.5 + 1e-9;
            double halfWidth = p.OverallWidth * 0.5 + 1e-9;
            for (int i = 0; i < polygon.Length; i++)
            {
                double dx = polygon[i].X - pose.X;
                double dy = polygon[i].Y - pose.Y;
                double localX = c * dx + s * dy;
                double localY = -s * dx + c * dy;
                if (Math.Abs(localX) > halfLength || Math.Abs(localY) > halfWidth)
                    return false;
            }
            return true;
        }

        static void AddIfChanged(
            List<Point2d[]> polygons, Point2d[] polygon, ref Point2d[] previous)
        {
            if (polygon == null || polygon.Length < 3) return;
            if (previous == null || !SamePolygon(previous, polygon))
            {
                polygons.Add(polygon);
                previous = polygon;
            }
        }

        static bool SamePolygon(Point2d[] a, Point2d[] b)
        {
            if (a == null || b == null || a.Length != b.Length) return false;
            const double tol2 = 1e-20;
            for (int i = 0; i < a.Length; i++)
            {
                double dx = a[i].X - b[i].X;
                double dy = a[i].Y - b[i].Y;
                if (dx * dx + dy * dy > tol2) return false;
            }
            return true;
        }

        static List<Point2d> UnionMeters(List<Point2d[]> polygons)
        {
            if (polygons == null || polygons.Count == 0)
            {
                EnvelopeFallbackUsed = false;
                return new List<Point2d>();
            }
            if (polygons.Count == 1)
            {
                EnvelopeFallbackUsed = false;
                return GeometryUtil.ConvexHull(new List<Point2d>(polygons[0]));
            }
            List<Point2d> result = TruckKinematics.UnionEnvelopeFromConvexPolygons(
                polygons, out bool fallback);
            EnvelopeFallbackUsed = fallback;
            return result;
        }

        static List<Point2d> UnionToCad(List<Point2d[]> polygons, CarrierParams p)
        {
            return ToCadPoints(UnionMeters(polygons), p);
        }

        static List<Point2d> ToCadPoints(IEnumerable<Point2d> points, CarrierParams p)
        {
            var result = new List<Point2d>();
            if (points == null) return result;
            foreach (Point2d point in points)
                result.Add(ToCad(point.X, point.Y, p));
            return result;
        }

        static double Distance(Point2d a, Point2d b)
        {
            double dx = a.X - b.X, dy = a.Y - b.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }
}
