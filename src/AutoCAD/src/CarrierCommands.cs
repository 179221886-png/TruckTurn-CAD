using System;
using System.Collections.Generic;
using System.Text;
#if NET8_0_OR_GREATER
using System.Text.Json;
using System.Text.Json.Serialization;
#else
using System.Web.Script.Serialization;
#endif
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;

[assembly: CommandClass(typeof(TruckTurn.CarrierCommands))]

namespace TruckTurn
{
    /// <summary>集装箱跨运车 CAD 交互入口。与旧货车命令和参数完全隔离。</summary>
    public sealed class CarrierCommands
    {
        const string ParamsDictionary = "TruckTurn.CarrierParams.v1";
        const string ParamsRecord = "ParamsJson";
        const string LayerPath = "TT_CARRIER_PATH";
        const string LayerEquipmentEnvelope = "TT_CARRIER_ENVELOPE";
        const string LayerContainerEnvelope = "TT_CONTAINER_ENVELOPE";
        const string LayerClearance = "TT_CARRIER_CLEARANCE";
        const string LayerWheel = "TT_CARRIER_WHEEL";
        const string LayerBody = "TT_CARRIER_BODY";
        const int CumulativeEnvelopePreviewFrameLimit = 192;

        static CarrierParams parameters = CarrierParams.GenericFourWheelIndependent();
        static double lastHeadingRad = 0.0;

        [CommandMethod("CARRIERDRIVE")]
        public void CarrierDrive()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            string stage = "初始化";
            try
            {
                CarrierDriveBody(doc, doc.Editor, doc.Database, ref stage);
            }
            catch (System.Exception ex)
            {
                doc.Editor.WriteMessage(
                    "\n[CARRIERDRIVE 异常 @ {0}] {1}: {2}\n{3}",
                    stage, ex.GetType().FullName, ex.Message, ex.StackTrace);
            }
        }

        [CommandMethod("CAR")]
        public void CarrierDriveAlias()
        {
            CarrierDrive();
        }

        void CarrierDriveBody(Document doc, Editor ed, Database db, ref string stage)
        {
            stage = "读取跨运车参数";
            parameters = LoadFromDrawing(db, ed) ?? parameters ??
                CarrierParams.GenericFourWheelIndependent();
            double drawingScale = DetectDrawingUnitScale();

            stage = "跨运车选型窗口";
            using (var form = new CarrierSizeForm(parameters, drawingScale))
            {
                if (Application.ShowModalDialog(form) != System.Windows.Forms.DialogResult.OK || form.Result == null)
                {
                    ed.WriteMessage("\n已取消跨运车选型。");
                    return;
                }
                parameters = form.Result;
            }
            SaveToDrawing(db, ed, parameters);
            ReportParameters(ed, drawingScale);

            stage = "放置车体中心";
            var place = new CarrierPlaceJig(parameters, lastHeadingRad, Point3d.Origin);
            PromptResult placeResult = SafeDrag(ed, place, stage);
            if (placeResult == null || placeResult.Status != PromptStatus.OK) return;

            stage = "指定车头方向";
            var heading = new CarrierHeadingJig(parameters, place.Result, lastHeadingRad);
            PromptResult headingResult = SafeDrag(ed, heading, stage);
            if (headingResult == null || headingResult.Status != PromptStatus.OK) return;
            lastHeadingRad = heading.HeadingRad;

            CarrierPose currentPose = CarrierCadGeometry.ToMeterPose(
                place.Result, lastHeadingRad, parameters);
            CarrierFrame initialFrame = CarrierKinematics.Solve(
                parameters, currentPose, CarrierMotionCommand.Longitudinal(0.0));
            double[] currentAngles = WheelAngles(initialFrame);
            List<CarrierSteeringMode> modes = EnabledModes(parameters);
            int modeIndex = DefaultModeIndex(modes);
            int driveDirection = 1;

            var allIds = new List<ObjectId>();
            var initialIds = new List<ObjectId>();
            stage = "绘制初始车体";
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                EnsureLayers(db, tr);
                DrawFrame(db, tr, parameters, initialFrame, initialIds);
                tr.Commit();
            }
            allIds.AddRange(initialIds);

            var allFrames = new List<CarrierFrame> { initialFrame };
            var frameAdds = new List<int>();
            var segmentIds = new List<List<ObjectId>>();
            var poseHistory = new List<CarrierPose>();
            var angleHistory = new List<double[]>();
            var cumulativeEnvelopeIds = new List<ObjectId>();
            var confirmedEnvelopeSegments = new List<List<CarrierFrame>>();
            int segmentNumber = 0;

            ed.WriteMessage(
                "\nCARRIERDRIVE 已启动：左键确认；M 切换模式；B 切换前进/倒行；Esc 撤销；X 完成并打块。"
                + "\n模式切换按停车摆轮处理，轮角变化受参数中的最大转向速度限制。");

            while (true)
            {
                CarrierSteeringMode mode = modes[modeIndex];
                stage = "第" + (segmentNumber + 1) + "段预览";
                var jig = new CarrierDriveJig(
                    parameters, currentPose, currentAngles, mode, driveDirection);
                PromptResult result = SafeDrag(ed, jig, stage);
                if (result == null) break;

                if (result.Status != PromptStatus.OK)
                {
                    if (jig.Keyword == "M")
                    {
                        modeIndex = (modeIndex + 1) % modes.Count;
                        ed.WriteMessage("\n转向模式切换为【{0}】。",
                            CarrierCadGeometry.ModeName(modes[modeIndex]));
                        continue;
                    }
                    if (jig.Keyword == "B")
                    {
                        driveDirection = -driveDirection;
                        ed.WriteMessage("\n行驶方向切换为【{0}】。",
                            driveDirection > 0 ? "前进" : "倒行");
                        continue;
                    }
                    if (jig.Keyword == "X") break;

                    if (segmentIds.Count == 0)
                    {
                        EraseEntities(db, initialIds);
                        ed.WriteMessage("\n已取消 CARRIERDRIVE，未保留图元。");
                        return;
                    }

                    int last = segmentIds.Count - 1;
                    stage = "撤销第" + (last + 1) + "段";
                    EraseEntities(db, segmentIds[last]);
                    int remove = frameAdds[last];
                    if (remove > 0 && remove <= allFrames.Count - 1)
                        allFrames.RemoveRange(allFrames.Count - remove, remove);
                    currentPose = poseHistory[last];
                    currentAngles = angleHistory[last];
                    segmentIds.RemoveAt(last);
                    frameAdds.RemoveAt(last);
                    poseHistory.RemoveAt(last);
                    angleHistory.RemoveAt(last);
                    if (confirmedEnvelopeSegments.Count > last)
                        confirmedEnvelopeSegments.RemoveAt(last);
                    segmentNumber--;
                    stage = "撤销后更新累计包络";
                    ReplaceCumulativeEnvelopes(
                        db, parameters, allFrames, confirmedEnvelopeSegments,
                        cumulativeEnvelopeIds, fullQuality: false);
                    ed.WriteMessage("\n已撤销上一段，当前剩余 {0} 段。", segmentNumber);
                    continue;
                }

                CarrierSegmentPlan plan = jig.Plan;
                if (plan == null || plan.Frames == null || plan.Frames.Count < 2)
                {
                    ed.WriteMessage("\n该目标在当前模式下不可行，请移动光标、按 M 换模式或按 B 换方向。");
                    continue;
                }
                if (!jig.ConfirmMatchesDrawn(out string driftReason))
                {
                    ed.WriteMessage("\n落点解与最后显示的预览不一致（{0}），请确认新预览后再点一次。", driftReason);
                    continue;
                }

                var ids = new List<ObjectId>();
                stage = "第" + (segmentNumber + 1) + "段出图";
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    EnsureLayers(db, tr);
                    // 分段事务只落参考轨迹/轮迹/节点车辆；确认后用 allFrames
                    // 重算并替换累计包络，避免内部段边界残留。
                    DrawSegment(db, tr, parameters, plan.Frames, ids,
                        drawEnvelopes: false);
                    tr.Commit();
                }

                poseHistory.Add(currentPose);
                angleHistory.Add((double[])currentAngles.Clone());
                segmentIds.Add(ids);
                frameAdds.Add(plan.Frames.Count - 1);
                confirmedEnvelopeSegments.Add(new List<CarrierFrame>(plan.Frames));
                for (int i = 1; i < plan.Frames.Count; i++) allFrames.Add(plan.Frames[i]);
                currentPose = plan.LastFrame.Pose;
                currentAngles = WheelAngles(plan.LastFrame);
                segmentNumber++;
                allIds.AddRange(ids);
                lastHeadingRad = currentPose.HeadingRad;
                stage = "第" + segmentNumber + "段更新累计包络";
                ReplaceCumulativeEnvelopes(
                    db, parameters, allFrames, confirmedEnvelopeSegments,
                    cumulativeEnvelopeIds, fullQuality: false);
                ed.WriteMessage(
                    "\n[段{0}] {1}，运动 {2:F2}s，停车摆轮 {3:F2}s，目标偏差 {4:F3}m，{5}。",
                    segmentNumber, CarrierCadGeometry.ModeName(mode),
                    plan.DurationSeconds, plan.SteeringTransitionSeconds,
                    plan.CursorMissDistance, driveDirection > 0 ? "前进" : "倒行");
            }

            // 交互阶段使用有界关键帧；结束时用全部分段帧替换为精确包络。
            if (segmentNumber > 0)
            {
                stage = "生成最终精确累计包络";
                ReplaceCumulativeEnvelopes(
                    db, parameters, allFrames, confirmedEnvelopeSegments,
                    cumulativeEnvelopeIds, fullQuality: true);
            }

            // 累计包络已经精化；这里只补“仅首尾”的末车。
            if (segmentNumber > 0 && !parameters.ShowAllNodeVehicles)
            {
                var finalIds = new List<ObjectId>();
                stage = "绘制最终车辆姿态";
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    EnsureLayers(db, tr);
                    DrawFrame(db, tr, parameters,
                        allFrames[allFrames.Count - 1], finalIds);
                    tr.Commit();
                }
                allIds.AddRange(finalIds);
            }

            allIds.AddRange(cumulativeEnvelopeIds);

            stage = "跨运车路径打块";
            GroupIntoBlock(db, ed, allIds);
            ed.Regen();
            ed.WriteMessage(
                "\nCARRIERDRIVE 完成：{0} 段、{1} 帧。结果已按设备/箱体/安全/轮迹分层并整体成块。",
                segmentNumber, allFrames.Count);
        }

        static void DrawSegment(
            Database db, Transaction tr, CarrierParams p,
            IList<CarrierFrame> frames, List<ObjectId> ids,
            bool drawEnvelopes = true)
        {
            if (drawEnvelopes)
                DrawEnvelopes(db, tr, p, frames, ids);
            if (p.ShowReferencePath)
                AddOpen(db, tr, CarrierCadGeometry.ReferencePath(p, frames), LayerPath, ids);
            if (p.ShowWheelTracks)
            {
                foreach (List<Point2d> track in CarrierCadGeometry.WheelTracks(p, frames))
                    AddOpen(db, tr, track, LayerWheel, ids);
            }
            if (p.ShowAllNodeVehicles)
                DrawFrame(db, tr, p, frames[frames.Count - 1], ids);
        }

        static void DrawEnvelopes(
            Database db, Transaction tr, CarrierParams p,
            IList<CarrierFrame> frames, List<ObjectId> ids)
        {
            if (p.ShowEquipmentEnvelope)
                AddEnvelopeClosed(db, tr, p, frames,
                    CarrierCadGeometry.EquipmentEnvelope(p, frames),
                    LayerEquipmentEnvelope, ids);
            if (p.ShowContainerEnvelope && p.ContainerPreset != CarrierContainerPreset.None)
                AddEnvelopeClosed(db, tr, p, frames,
                    CarrierCadGeometry.ContainerEnvelope(p, frames),
                    LayerContainerEnvelope, ids);
            if (p.ShowSafetyEnvelope)
                AddEnvelopeClosed(db, tr, p, frames,
                    CarrierCadGeometry.ClearanceEnvelope(p, frames),
                    LayerClearance, ids);
        }

        static void ReplaceCumulativeEnvelopes(
            Database db, CarrierParams p, IList<CarrierFrame> allFrames,
            IList<List<CarrierFrame>> confirmedSegments,
            List<ObjectId> currentIds, bool fullQuality)
        {
            List<List<CarrierFrame>> envelopeSegments = fullQuality
                ? CopySegments(confirmedSegments)
                : CarrierCadGeometry.PreviewSegmentsAdaptive(
                    confirmedSegments, CumulativeEnvelopePreviewFrameLimit);
            List<CarrierFrame> fitFrames = fullQuality
                ? new List<CarrierFrame>(allFrames)
                : FlattenSegments(envelopeSegments);
            var replacementIds = new List<ObjectId>();
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                for (int i = 0; i < currentIds.Count; i++)
                {
                    ObjectId id = currentIds[i];
                    if (id.IsNull || !id.IsValid || id.IsErased) continue;
                    Entity entity = tr.GetObject(id, OpenMode.ForWrite, false) as Entity;
                    if (entity != null && !entity.IsErased) entity.Erase();
                }

                if (envelopeSegments.Count > 0 && fitFrames.Count >= 2)
                {
                    EnsureLayers(db, tr);
                    DrawCumulativeEnvelopes(
                        db, tr, p, envelopeSegments, fitFrames,
                        !fullQuality, replacementIds);
                }
                tr.Commit();
            }

            currentIds.Clear();
            currentIds.AddRange(replacementIds);
        }

        static void DrawCumulativeEnvelopes(
            Database db, Transaction tr, CarrierParams p,
            IList<List<CarrierFrame>> segments, IList<CarrierFrame> fitFrames,
            bool interactivePreview, List<ObjectId> ids)
        {
            if (p.ShowEquipmentEnvelope)
                AddEnvelopeClosed(db, tr, p, fitFrames,
                    CarrierCadGeometry.EquipmentEnvelopeBySegments(p, segments),
                    LayerEquipmentEnvelope, ids, interactivePreview);
            if (p.ShowContainerEnvelope && p.ContainerPreset != CarrierContainerPreset.None)
                AddEnvelopeClosed(db, tr, p, fitFrames,
                    CarrierCadGeometry.ContainerEnvelopeBySegments(p, segments),
                    LayerContainerEnvelope, ids, interactivePreview);
            if (p.ShowSafetyEnvelope)
                AddEnvelopeClosed(db, tr, p, fitFrames,
                    CarrierCadGeometry.ClearanceEnvelopeBySegments(
                        p, segments, fitFrames),
                    LayerClearance, ids, interactivePreview);
        }

        static List<List<CarrierFrame>> CopySegments(
            IList<List<CarrierFrame>> segments)
        {
            var result = new List<List<CarrierFrame>>();
            if (segments == null) return result;
            for (int i = 0; i < segments.Count; i++)
                result.Add(new List<CarrierFrame>(segments[i]));
            return result;
        }

        static List<CarrierFrame> FlattenSegments(
            IList<List<CarrierFrame>> segments)
        {
            var result = new List<CarrierFrame>();
            if (segments == null) return result;
            for (int i = 0; i < segments.Count; i++)
            {
                List<CarrierFrame> segment = segments[i];
                for (int j = 0; j < segment.Count; j++)
                {
                    if (result.Count > 0 && j == 0) continue;
                    result.Add(segment[j]);
                }
            }
            return result;
        }

        static void DrawFrame(
            Database db, Transaction tr, CarrierParams p,
            CarrierFrame frame, List<ObjectId> ids)
        {
            if (!p.ShowBody || frame == null) return;
            AddClosed(db, tr, CarrierCadGeometry.BodyPolygon(p, frame.Pose), LayerBody, ids);
            List<Point2d> container = CarrierCadGeometry.ContainerPolygon(p, frame.Pose);
            if (container.Count >= 3)
                AddClosed(db, tr, container, LayerContainerEnvelope, ids);
            for (int i = 0; i < frame.Wheels.Length; i++)
                AddClosed(db, tr, CarrierCadGeometry.WheelPolygon(p, frame, i), LayerWheel, ids);
        }

        static void AddClosed(
            Database db, Transaction tr, List<Point2d> points,
            string layer, List<ObjectId> ids)
        {
            if (points == null || points.Count < 3) return;
            ids.Add(DrawingUtil.AddPolyline(db, tr, points, true, layer));
        }

        static void AddEnvelopeClosed(
            Database db, Transaction tr, CarrierParams p, IList<CarrierFrame> frames,
            List<Point2d> points,
            string layer, List<ObjectId> ids, bool interactivePreview = false)
        {
            if (points == null || points.Count < 3) return;
            List<Point2d> fitted;
            List<double> bulges;
            if (interactivePreview)
                CarrierCadGeometry.FitEnvelopeBulgesInteractive(
                    p, points, frames, out fitted, out bulges);
            else
                CarrierCadGeometry.FitEnvelopeBulges(
                    p, points, frames, out fitted, out bulges);
            if (fitted == null || fitted.Count < 3)
            {
                AddClosed(db, tr, points, layer, ids);
                return;
            }
            ids.Add(DrawingUtil.AddPolyline(db, tr, fitted, bulges, true, layer));
        }

        static void AddOpen(
            Database db, Transaction tr, List<Point2d> points,
            string layer, List<ObjectId> ids)
        {
            if (points == null || points.Count < 2) return;
            ids.Add(DrawingUtil.AddPolyline(db, tr, points, false, layer));
        }

        static void EnsureLayers(Database db, Transaction tr)
        {
            DrawingUtil.EnsureLayer(db, tr, LayerPath, 6);
            DrawingUtil.EnsureLayer(db, tr, LayerEquipmentEnvelope, 2);
            DrawingUtil.EnsureLayer(db, tr, LayerContainerEnvelope, 30);
            DrawingUtil.EnsureLayer(db, tr, LayerClearance, 1);
            DrawingUtil.EnsureLayer(db, tr, LayerWheel, 1);
            DrawingUtil.EnsureLayer(db, tr, LayerBody, 3);
        }

        static List<CarrierSteeringMode> EnabledModes(CarrierParams p)
        {
            var all = new[]
            {
                CarrierSteeringMode.Longitudinal,
                CarrierSteeringMode.CounterPhaseFourWheel,
                CarrierSteeringMode.FrontOnly,
                CarrierSteeringMode.RearOnly,
                CarrierSteeringMode.Crab,
                CarrierSteeringMode.Lateral,
                CarrierSteeringMode.Pivot
            };
            var result = new List<CarrierSteeringMode>();
            for (int i = 0; i < all.Length; i++) if (p.Supports(all[i])) result.Add(all[i]);
            if (result.Count == 0) result.Add(CarrierSteeringMode.Longitudinal);
            return result;
        }

        static int DefaultModeIndex(List<CarrierSteeringMode> modes)
        {
            int i = modes.IndexOf(CarrierSteeringMode.CounterPhaseFourWheel);
            return i >= 0 ? i : 0;
        }

        static void ReportParameters(Editor ed, double drawingScale)
        {
            ed.WriteMessage(
                "\n[跨运车参数] {0}｜{1:F2}×{2:F2}m｜轴距 {3:F2}m｜轮距 {4:F2}m｜1m={5:g}图形单位",
                parameters.PresetName, parameters.OverallLength, parameters.OverallWidth,
                parameters.Wheelbase, parameters.TrackWidth, parameters.UnitScale);
            ed.WriteMessage("\n[箱型] {0}｜安全裕量 {1:F2}m｜数据状态 {2}（非厂家认证）。",
                parameters.ContainerPreset, parameters.SafetyMargin, parameters.DataStatus);
            if (drawingScale > 0.0 && Math.Abs(drawingScale - parameters.UnitScale) > 1e-6)
                ed.WriteMessage(
                    "\n[单位警告] 本图 INSUNITS 推算 1m={0:g} 图形单位，当前选择 1m={1:g}；若图形过大或看不到，请重新选择图纸单位。",
                    drawingScale, parameters.UnitScale);
        }

        static PromptResult SafeDrag(Editor ed, DrawJig jig, string stage)
        {
            try { return ed.Drag(jig); }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[CARRIERDRIVE {0}失败] {1}: {2}",
                    stage, ex.GetType().Name, ex.Message);
                return null;
            }
        }

        static double[] WheelAngles(CarrierFrame frame)
        {
            var result = new double[frame.Wheels.Length];
            for (int i = 0; i < result.Length; i++) result[i] = frame.Wheels[i].SteerAngleRad;
            return result;
        }

        static void EraseEntities(Database db, IList<ObjectId> ids)
        {
            if (ids == null || ids.Count == 0) return;
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                for (int i = 0; i < ids.Count; i++)
                {
                    ObjectId id = ids[i];
                    if (id.IsNull || !id.IsValid || id.IsErased) continue;
                    Entity entity = tr.GetObject(id, OpenMode.ForWrite, false) as Entity;
                    if (entity != null && !entity.IsErased) entity.Erase();
                }
                tr.Commit();
            }
        }

        static void GroupIntoBlock(Database db, Editor ed, IList<ObjectId> ids)
        {
            var unique = new List<ObjectId>();
            var seen = new HashSet<ObjectId>();
            for (int i = 0; i < ids.Count; i++)
            {
                ObjectId id = ids[i];
                if (!id.IsNull && id.IsValid && !id.IsErased && seen.Add(id)) unique.Add(id);
            }
            if (unique.Count == 0) return;

            try
            {
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    var table = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                    string name = "CarrierPath";
                    int suffix = 1;
                    while (table.Has(name)) name = "CarrierPath_" + suffix++;
                    var definition = new BlockTableRecord { Name = name };
                    table.UpgradeOpen();
                    ObjectId definitionId = table.Add(definition);
                    tr.AddNewlyCreatedDBObject(definition, true);
                    db.DeepCloneObjects(
                        new ObjectIdCollection(unique.ToArray()), definitionId, new IdMapping(), false);

                    var space = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    var reference = new BlockReference(Point3d.Origin, definitionId);
                    reference.SetDatabaseDefaults();
                    space.AppendEntity(reference);
                    tr.AddNewlyCreatedDBObject(reference, true);
                    for (int i = 0; i < unique.Count; i++)
                    {
                        Entity entity = tr.GetObject(unique[i], OpenMode.ForWrite, false) as Entity;
                        if (entity != null && !entity.IsErased) entity.Erase();
                    }
                    tr.Commit();
                    ed.WriteMessage("\n[块] 已生成「{0}」，包含 {1} 个图元。", name, unique.Count);
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[块] 打块失败（{0}: {1}），原图元已保留。",
                    ex.GetType().Name, ex.Message);
            }
        }

        static double DetectDrawingUnitScale()
        {
            try
            {
                object value = Application.GetSystemVariable("INSUNITS");
                int units = Convert.ToInt32(value);
                if (units == 4) return 1000.0;
                if (units == 5) return 100.0;
                if (units == 6) return 1.0;
            }
            catch { }
            return 0.0;
        }

        #if NET8_0_OR_GREATER
        static JsonSerializerOptions JsonOptions()
        {
            var options = new JsonSerializerOptions
            {
                IncludeFields = true,
                NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals
            };
            options.Converters.Add(new JsonStringEnumConverter());
            return options;
        }
        #endif

        /// <summary>
        /// 浩辰 CAD 2026 运行于 .NET 8，2025 运行于 .NET Framework 4.8。
        /// 两个版本采用各自平台自带的 JSON 实现，同时保持同一份图内参数格式。
        /// </summary>
        static string SerializeParams(CarrierParams value)
        {
#if NET8_0_OR_GREATER
            return JsonSerializer.Serialize(value, JsonOptions());
#else
            string json = new JavaScriptSerializer().Serialize(value);
            // JavaScriptSerializer 会输出裸 NaN/Infinity；.NET 8 的
            // System.Text.Json 只接受带引号的命名浮点值。统一为后者，
            // 保证同一张图可在浩辰 CAD 2025/2026 间互相打开。
            return json
                .Replace(":-Infinity", ":\"-Infinity\"")
                .Replace(":Infinity", ":\"Infinity\"")
                .Replace(":NaN", ":\"NaN\"");
#endif
        }

        static CarrierParams DeserializeParams(string json)
        {
#if NET8_0_OR_GREATER
            return JsonSerializer.Deserialize<CarrierParams>(json, JsonOptions());
#else
            return new JavaScriptSerializer().Deserialize<CarrierParams>(json);
#endif
        }

        static void SaveToDrawing(Database db, Editor ed, CarrierParams value)
        {
            try
            {
                string json = SerializeParams(value);
                var values = new List<TypedValue>();
                for (int i = 0; i < json.Length; i += 240)
                    values.Add(new TypedValue((int)DxfCode.Text,
                        json.Substring(i, Math.Min(240, json.Length - i))));

                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(
                        db.NamedObjectsDictionaryId, OpenMode.ForRead);
                    DBDictionary dict;
                    if (nod.Contains(ParamsDictionary))
                        dict = (DBDictionary)tr.GetObject(
                            nod.GetAt(ParamsDictionary), OpenMode.ForWrite);
                    else
                    {
                        nod.UpgradeOpen();
                        dict = new DBDictionary();
                        nod.SetAt(ParamsDictionary, dict);
                        tr.AddNewlyCreatedDBObject(dict, true);
                    }

                    Xrecord record;
                    if (dict.Contains(ParamsRecord))
                        record = (Xrecord)tr.GetObject(dict.GetAt(ParamsRecord), OpenMode.ForWrite);
                    else
                    {
                        record = new Xrecord();
                        dict.SetAt(ParamsRecord, record);
                        tr.AddNewlyCreatedDBObject(record, true);
                    }
                    record.Data = new ResultBuffer(values.ToArray());
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[跨运车参数保存失败] {0}: {1}",
                    ex.GetType().Name, ex.Message);
            }
        }

        static CarrierParams LoadFromDrawing(Database db, Editor ed)
        {
            try
            {
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(
                        db.NamedObjectsDictionaryId, OpenMode.ForRead);
                    if (!nod.Contains(ParamsDictionary)) return null;
                    var dict = (DBDictionary)tr.GetObject(
                        nod.GetAt(ParamsDictionary), OpenMode.ForRead);
                    if (!dict.Contains(ParamsRecord)) return null;
                    var record = (Xrecord)tr.GetObject(
                        dict.GetAt(ParamsRecord), OpenMode.ForRead);
                    var sb = new StringBuilder();
                    TypedValue[] values = record.Data == null
                        ? new TypedValue[0] : record.Data.AsArray();
                    for (int i = 0; i < values.Length; i++)
                        if (values[i].Value is string text) sb.Append(text);
                    CarrierParams loaded = DeserializeParams(sb.ToString());
                    if (loaded != null && loaded.SchemaVersion < CarrierParams.CurrentSchemaVersion)
                    {
                        // schema v1 的六项输出开关默认全为 true，会让普通电脑
                        // 在交互预览时同时重算三组包络和全部轨迹。只有当保存值
                        // 仍是历史「全勾选」形态时才迁移为新默认；用户已经手动
                        // 调整过的组合原样保留。
                        if (loaded.SchemaVersion == 1)
                        {
                            bool legacyAllEnabled =
                                loaded.ShowReferencePath && loaded.ShowEquipmentEnvelope &&
                                loaded.ShowContainerEnvelope && loaded.ShowSafetyEnvelope &&
                                loaded.ShowWheelTracks && loaded.ShowBody;
                            if (legacyAllEnabled)
                            {
                                loaded.ShowReferencePath = false;
                                loaded.ShowContainerEnvelope = false;
                                loaded.ShowWheelTracks = false;
                            }
                        }
                        // schema 3 新增节点车辆显示模式；旧图统一迁移为更简洁的“仅首尾”。
                        loaded.ShowAllNodeVehicles = false;
                        loaded.SchemaVersion = CarrierParams.CurrentSchemaVersion;
                    }
                    if (loaded == null || loaded.Validate().Count > 0)
                        throw new InvalidOperationException("图内跨运车参数未通过校验。");
                    tr.Commit();
                    return loaded;
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[跨运车参数读取失败，使用当前默认值] {0}: {1}",
                    ex.GetType().Name, ex.Message);
                return null;
            }
        }
    }
}
