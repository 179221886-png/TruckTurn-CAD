using System;
using System.Collections.Generic;
using System.Linq;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Runtime;

[assembly: CommandClass(typeof(TruckTurn.TruckTurnCommands))]

namespace TruckTurn
{
    /// <summary>
    /// 浩辰CAD 货车转弯路径插件（v4.3 精简版）。
    /// 命令：
    ///   TRUCKDRIVE  — 连续点击驾驶货车，实时生成扫掠包络与轮迹
    ///   TRUCKTURN90 — 一键让车辆以最小半径原地转 90°，并停在回正位置
    /// </summary>
    public class TruckTurnCommands : IExtensionApplication
    {
        private static VehicleParams _params = VehicleParams.Defaults();

        /// <summary>上次使用的车头朝向，作为下次放置的初始朝向</summary>
        private static Vector2d _lastHeading = new Vector2d(1, 0);

        private const string DictName = "TruckTurnParams";
        private const string LayerEnvelope = "货车转弯_包络"; // 颜色 2 黄
        private const string LayerWheels = "货车转弯_轮径";   // 颜色 1 红
        private const string LayerBody = "货车转弯_车体";     // 颜色 3 绿
        private const string LayerGhost = "货车转弯_姿态";    // 颜色 8 灰（中间姿态）
        private const string LayerFrontTrack = "货车转弯_前轴轨迹"; // 颜色 6 洋红
        private const string LayerRadii = "货车转弯_转弯半径";   // 颜色 5 蓝

        /// <summary>最终出图的角度采样步长（度）。越小越光滑，但点越多。</summary>
        private const double OutputStepDeg = TruckKinematics.FineStepDeg;
        // 点击确认时的累计包络使用有界关键帧，避免铰接车路径越长、每次点击越慢；
        // X 结束时仍以全部帧替换为精确结果。
        private const int CumulativeEnvelopePreviewFrameLimit = 320;

        /// <summary>
        /// v4.9.6：包络稀化容差（米）。当前管线（GeometryUtil.SimplifyEnvelopeForSpline）
        /// 内部 hardcode 使用 **5cm 大容差 RDP**（实测 1179 顶点直接砍到 ~35），
        /// 所以本常量不再实际生效 —— 保留仅为向后兼容。
        ///
        /// 调参历程（dump_env.csv 实测）：
        ///   v4.8/v4.9.2/v4.9.5 的「先 Laplacian 后 RDP(5mm)」方案：
        ///     1179 → 平滑(60 次 λ=0.2)→ 1179 → RDP(5mm) → 85~117 顶点
        ///     问题：Laplacian 不删顶点只会拉伸短边，把每帧一个的「短边+90°」反射伪尖角
        ///     变成均匀间距的小凸起（用户看到的「规律性波浪」），RDP 5mm 砍不掉近似共线段。
        ///   v4.9.6 新方案：
        ///     1179 → RDP(5cm) → 35 顶点 → 外推 48mm
        ///     5cm 容差 RDP 能把整段近似共线（每点偏离首尾 < 1mm）压成 2 个端点。
        ///     AABB 与 RAW envelope 几乎一致（差 < 5cm），无自交，max 转角 90°（车体真实折角）。
        /// </summary>
        private const double EnvelopeSimplifyTol = 0.002;

        /// <summary>
        /// v4.9.2：保角平滑的迭代次数。
        /// 早先 λ=0.5、20 次 —— 波长 2 顶点噪声一次归零，但波长 12+ 残留 25%，
        /// 残留下来的几毫米起伏会被 fit-spline 忠实复现成包络波浪（v4.9.1 实机确认）。
        /// 改 λ=0.2、60 次：波长 12 残留降到 1.6%、波长 24 残留 35%（原 71%），
        /// 仍由「转角 >60° 且至少一侧长边 >0.15m」的锚点护住真实折角。
        /// </summary>
        private const int EnvelopeSmoothIters = 60;

        public void Initialize() { LoadFromDrawing(); }
        public void Terminate() { }

        #region 连续驾驶命令：像画多段线一样逐段确认货车位置
        /// <summary>
        /// TRUCKDRIVE：连续点击驾驶货车。光标 = 前轴中心，实时显示完整货车图块、包络与轮迹；
        /// 每点击一次确认一段并立即画出该段包络/轮迹，右键/回车结束后统一出四半径。
        /// 不画灰色中间姿态。
        /// </summary>
        [CommandMethod("TRUCKDRIVE")]
        public void TruckDrive()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            // v4.5.1：整个命令体统一包一层，任何异常都把「发生在哪个阶段 + 类型 + 消息 + 堆栈」
            // 打到命令行。此前异常被 CAD 的 InvokeWorkerWithException 吞掉只留一行堆栈，
            // 表现为「点了没反应」，无从下手。
            string stage = "载入已存参数";
            try
            {
                TruckDriveBody(doc, ed, db, ref stage);
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[TRUCKDRIVE 异常 @ {0}] {1}: {2}\n{3}",
                    stage, ex.GetType().FullName, ex.Message, ex.StackTrace);
            }
        }

        private void TruckDriveBody(Document doc, Editor ed, Database db, ref string stage)
        {
            // v4.6.1：选型统一走 PromptVehicleParams（含图纸单位自动判定 + 参数摘要 + null 防御）。
            // 此前 _params 可能因选型窗校验失败而变成 null，后续 Jig 一访问字段就抛
            // NullReferenceException，CAD 只留一行 InvokeWorkerWithException，看起来就是"点了什么都没出来"。
            if (!PromptVehicleParams(db, ed, ref stage)) return;

            // 阶段一：放置前轴中心起始位置
            stage = "阶段一 放置起始点";
            ed.WriteMessage("\n[阶段1 开始] 请指定货车初始位置...");
            var placeJig = new TruckPlaceJig(_params, _lastHeading, Point3d.Origin);
            var r1 = SafeDrag(ed, placeJig, "阶段一 放置起始点");
            if (r1 == null || r1.Status != PromptStatus.OK) { ed.WriteMessage("\n[阶段1 取消] r1={0}", r1?.Status); return; }
            Point2d startFront = placeJig.Result;
            ed.WriteMessage(" [阶段1 完成] 起点=({0:F2},{1:F2})", startFront.X, startFront.Y);

            // 阶段二：确定车头朝向（绕前轴中心旋转）
            stage = "阶段二 确定车头朝向";
            ed.WriteMessage("\n[阶段2 开始] 请指定车头朝向...");
            var headJig = new TruckHeadingJig(_params, startFront, _lastHeading);
            var r2 = SafeDrag(ed, headJig, "阶段二 确定车头朝向");
            if (r2 == null || r2.Status != PromptStatus.OK) { ed.WriteMessage("\n[阶段2 取消] r2={0}", r2?.Status); return; }
            Vector2d heading = headJig.Result;
            _lastHeading = heading;
            ed.WriteMessage(" [阶段2 完成] 车头方向={0:F1}°", Math.Atan2(heading.Y, heading.X) * 180 / Math.PI);

            // 阶段三：转向方向（v4.9.18 起只保留自动判向，删除手动选左/右——
            //   自动判向已覆盖所有场景，手动选项只会与预览/确认的自动侧不一致）。
            bool autoSide = true;
            int steerSign = 1;   // 仅作初始值；自动判向下 Jig 每段自行决定转向侧
            ed.WriteMessage("\n自动判向：光标在车身左侧则左转，右侧则右转。");

            // v4.9.4：选择行驶方向（前进 / 倒退），与 TRUCKTURN90 保持一致。
            //   之前倒退只能进到驾驶循环后按 B 才看得见，用户根本找不到入口，
            //   反馈「truckdrive 没有倒退」。这里显式问一次；默认前进 + AllowNone，
            //   直接回车就走，不增加任何操作步骤。
            var pkdDrive = new PromptKeywordOptions("\n行驶方向 [前进(F)/倒退(B)] <前进>: ", "Forward Back");
            pkdDrive.Keywords.Default = "Forward";
            pkdDrive.AllowNone = true;
            var pkddr = ed.GetKeywords(pkdDrive);
            if (pkddr.Status != PromptStatus.OK && pkddr.Status != PromptStatus.None)
            { ed.WriteMessage("\n已取消。"); return; }
            int startDir = (pkddr.Status == PromptStatus.OK && pkddr.StringResult == "Back") ? -1 : 1;
            if (startDir < 0)
                ed.WriteMessage("\n已设为【倒退】：每段沿车头反方向行驶；方向盘左打，车往车身右后方走（真实物理）。"
                              + "运行中按 B 可随时切回前进。");

            WarnIfTightTurn(ed);

            stage = "生成起始帧";
            // 初始帧（前轴中心）
            double startPhi2 = Math.Atan2(heading.Y, heading.X);
            Frame initFrame;
            if (_params.Articulated)
            {
                var initList = TruckKinematics.SimulateFromStateFront(_params, startFront, heading, startPhi2,
                                                                       steerSign, 0.0, 0.0, 1, OutputStepDeg);
                initFrame = initList.Count > 0 ? initList[0] : new Frame
                {
                    O1 = startFront - _params.TractorWheelbase * heading,
                    U1 = heading,
                    K = startFront - (_params.TractorWheelbase - _params.TractorRearToKingpin) * heading,
                    U2 = heading,
                    O2 = startFront - (_params.TractorWheelbase - _params.TractorRearToKingpin + _params.TrailerKingpinToRearAxle) * heading
                };
            }
            else
            {
                Point2d O1 = startFront - _params.TractorWheelbase * heading;
                initFrame = new Frame { O1 = O1, U1 = heading, K = startFront, U2 = heading, O2 = startFront };
            }

            var allFrames = new List<Frame>();
            allFrames.Add(initFrame);
            Frame curFrame = initFrame;

            // v4.6 撤销支持：每确认一段就记三样东西，右键撤销时按它们精确还原
            //   ① segFrameCount —— 本段往 allFrames 里追加了多少帧（撤销时从尾部剪掉）
            //   ② segEntityIds  —— 本段画出了哪些图元（撤销时逐个 Erase）
            //   ③ frameHist     —— 本段开始前的车姿（撤销后货车回到这里）
            var segFrameCount = new List<int>();
            var segEntityIds = new List<List<ObjectId>>();
            var frameHist = new List<Frame>();
            // 每段确认时缓存完整车体 footprint。最终 X 出图直接合并这些缓存，
            // 不再重新遍历全部历史帧展开牵引车/车架/挂车矩形。
            var segmentEnvelopeFootprints = new List<List<Point2d[]>>();
            // 当前所有已确认段的唯一累计最外包络；确认/撤销时原位替换。
            var cumulativeEnvelopeIds = new List<ObjectId>();

            // v4.9.11：初始车体也记入图元清单，结束时一起打进路径块
            var initIds = new List<ObjectId>();
            using (var tr = db.TransactionManager.StartTransaction())
            {
                if (_params.ShowBody)
                {
                    DrawingUtil.EnsureLayer(db, tr, LayerBody, 3);
                    AddBody(db, tr, initFrame, LayerBody, initIds);
                }
                if (_params.ShowWheels)
                    DrawTires(db, tr, initFrame, _params, 0, initIds);
                tr.Commit();
            }

            // v4.9.18：撤销只保留 Esc（U 键入口已删，单一撤销方式）。
            // v4.9：新增 B = 切换前进/倒退。
            ed.WriteMessage("\n连续驾驶：左键=确认一段；Esc = 撤销上一段；B = 切换前进/倒退；X = 结束并出图；L/R = 一键 90°。");
            ed.WriteMessage("\n当前行驶方向：{0}。运行中按 B 回车可随时切换。", startDir > 0 ? "前进" : "倒退");

            // v4.9：当前行驶方向（+1 前进 / −1 倒退）。按 B 切换，撤销不会改变它。
            // v4.9.4：初值取自上面的「行驶方向」关键字，不再是写死的 1。
            int driveDir = startDir;

            while (true)
            {
                // 段号直接由已确认段数推出，撤销后自动回到正确编号，不会越撤越乱
                int seg = segEntityIds.Count + 1;
                stage = "段" + seg + " 预览/拾取";
                var driveJig = new TruckDriveJig(_params, curFrame, steerSign, autoSide, driveDir);

                PromptResult r = SafeDrag(ed, driveJig, "段" + seg + " 预览/拾取");
                // 预览内部抛异常（部分 CAD 按 Esc 也会抛）时 SafeDrag 返回 null，按结束处理
                if (r == null) break;

                if (r.Status != PromptStatus.OK)
                {
                    // v4.9：B = 切换前进 / 倒退。方向是「段」的属性，Jig 自身只读，
                    // 交给主循环改状态后重建 Jig 重新预览（下一次循环自然带上新方向）。
                    if (driveJig.Kw == "B")
                    {
                        driveDir = -driveDir;
                        ed.WriteMessage("\n已切换为【{0}】（当前共 {1} 段）。{2}",
                            driveDir > 0 ? "前进" : "倒退", segEntityIds.Count,
                            driveDir > 0
                                ? "后续每段沿车头方向行驶。"
                                : "后续每段倒退行驶：光标直接指挥车尾（铰接车=挂车后轴），车尾精确到达光标点，铰接角自动收敛不折叠。");
                        continue;
                    }

                    // 中途用户按 L / R 触发 90° 转弯（不论当前自动判向还是手动固定方向）
                    if (driveJig.KwTurn90 == "L" || driveJig.KwTurn90 == "R")
                    {
                        int turn90Sign = driveJig.KwTurn90 == "L" ? 1 : -1;
                        Point2d front0 = TruckKinematics.FrontAxle(curFrame, _params);
                        // v4.9：带上行驶方向。倒退 90° 不做挂车拉直（倒着拉不直，物理上铰接角发散）。
                        var turn90 = TruckKinematics.SimulateTurn90AndStraighten(_params, front0, curFrame.U1, turn90Sign, OutputStepDeg, driveDir);
                        if (turn90 != null && turn90.Count > 1)
                        {
                            var ids90 = new List<ObjectId>();
                            Frame endFrame = turn90[turn90.Count - 1];
                            // 切到「以车头方向为基准」：90° 转弯后铰接车挂车回正，后续仍按车头方向继续
                            using (var tr2 = db.TransactionManager.StartTransaction())
                            {
                                DrawingUtil.EnsureLayer(db, tr2, LayerEnvelope, 2);
                                DrawingUtil.EnsureLayer(db, tr2, LayerWheels, 1);
                                DrawingUtil.EnsureLayer(db, tr2, LayerFrontTrack, 6);
                                DrawingUtil.EnsureLayer(db, tr2, LayerBody, 3);
                                // 分段事务只落轨迹；确认后由累计包络事务用全部已确认帧
                                // 求并集并替换上一版，避免各段闭合边界留在内部。
                                DrawSegmentTracks(db, tr2, turn90, null, ids90,
                                    drawEnvelope: false);
                                if (_params.ShowAllNodeVehicles)
                                {
                                    if (_params.ShowBody)
                                        AddBody(db, tr2, endFrame, LayerBody, ids90);
                                    if (_params.ShowWheels)
                                    {
                                        int[] turn90Signs = TruckKinematics.ComputeSteerSigns(turn90);
                                        DrawTires(db, tr2, endFrame, _params,
                                            turn90Signs[turn90Signs.Length - 1], ids90);
                                    }
                                }
                                tr2.Commit();
                            }
                            // 三份历史一起压栈：放在出图事务之后，事务抛异常时不会只压一半导致三者错位
                            frameHist.Add(curFrame);
                            segFrameCount.Add(turn90.Count - 1);
                            segEntityIds.Add(ids90);
                            segmentEnvelopeFootprints.Add(
                                TruckKinematics.BuildEnvelopeFootprints(turn90, _params));
                            for (int i = 1; i < turn90.Count; i++) allFrames.Add(turn90[i]);
                            curFrame = endFrame;
                            _lastHeading = curFrame.U1;
                            stage = "段" + seg + " 更新累计包络";
                            ReplaceCumulativeEnvelope(db, allFrames, cumulativeEnvelopeIds);
                            // 刻意不改写 steerSign：一键 90° 是一次性动作，不该悄悄改掉
                            // 用户先前手动选定的转向偏好（否则下一次预览方向就变了）。
                            ed.WriteMessage("\n[段{0}] 一键 90°{1}转（{2}）：{3} 帧。{4}", seg,
                                turn90Sign > 0 ? "左" : "右",
                                driveDir > 0 ? "前进" : "倒退", turn90.Count,
                                (driveDir < 0 && _params.Articulated)
                                    ? "倒退 90°：车尾按平缓半径甩尾，铰接角自动收敛（司机修正模型），不再 jackknife。"
                                    : "");
                            continue;
                        }
                        // 生成失败时不 break：已画出的段都在图上了，退命令等于白干，
                        // 提示一句回到预览继续即可。
                        ed.WriteMessage("\n一键 90° 转弯未生成路径（转向方向 {0}），已回到预览。", driveJig.KwTurn90);
                        continue;
                    }

                    // 明确的「结束」通道
                    if (driveJig.Kw == "X") break;

                    // 其余非 OK 退出（右键 / 回车 / 关键字 U）：撤销上一段。
                    // 已经退到起点说明没有可撤的了，此时才当作结束。
                    if (segEntityIds.Count == 0) break;

                    int undone = segEntityIds.Count;
                    stage = "撤销段" + undone;
                    int erased = UndoLastSegment(
                        db, ed, allFrames, segFrameCount, segEntityIds, frameHist,
                        segmentEnvelopeFootprints, ref curFrame);
                    stage = "撤销后更新累计包络";
                    ReplaceCumulativeEnvelope(db, allFrames, cumulativeEnvelopeIds);
                    ed.WriteMessage("\n已撤销第 {0} 段（删除 {1} 个图元），货车回到第 {2} 段起点。继续点击重画，或按 X 结束出图。",
                        undone, erased, segEntityIds.Count + 1);
                    continue;
                }
                stage = "段" + seg + " 精细重算";
                var segFrames = driveJig.SegmentFrames;
                if (segFrames == null || segFrames.Count < 2)
                {
                    ed.WriteMessage("\n[段{0}] 该段预览路径为空（不足 2 帧），未绘制，已回到预览。", seg);
                    continue;
                }

                // v4.9.19 漂移守卫：点击瞬间按落点重算的解，与最后一次实际画出的预览
                // 差异过大（回退陡变区快速移动 / 对象捕捉跳点）时，用户确认的是他从未
                // 见过的解 —— 不确认，回到预览（新解随即画出），让用户看清楚后再点一次。
                if (!driveJig.ConfirmMatchesDrawn(out string driftWhy))
                {
                    ed.WriteMessage("\n[段{0}] 落点解与刚才的预览差异较大（{1}），已按新位置刷新预览——请再点一次确认。", seg, driftWhy);
                    continue;
                }

                // 预览用粗采样保证流畅；确认后用精细步长重算该段，使包络/轮迹光滑
                // v4.9：dir 必须是 Jig 实际使用的方向（Jig 只读，与循环变量 driveDir 一致）
                // v4.9.9：倒退改走「车尾控制」管线（挂车牵引模型），与前进的前轴控制分开。
                List<Frame> segFramesFine;
                if (driveJig.Dir < 0)
                {
                    segFramesFine = TruckKinematics.SimulateReverseFromState(_params, curFrame,
                        driveJig.ReverseSide, driveJig.TurnAngleDeg, driveJig.StraightLen,
                        driveJig.ReverseRadius, OutputStepDeg);
                }
                else
                {
                    Point2d segStartFront = TruckKinematics.FrontAxle(curFrame, _params);
                    double segStartPhi2 = Math.Atan2(curFrame.U2.Y, curFrame.U2.X);
                    segFramesFine = TruckKinematics.SimulateFromStateFront(_params, segStartFront, curFrame.U1,
                        segStartPhi2, driveJig.SteerSign, driveJig.TurnAngleDeg, driveJig.StraightLen, driveJig.Dir, OutputStepDeg, driveJig.RearRadiusOverride);
                }
                if (segFramesFine == null || segFramesFine.Count < 2)
                {
                    ed.WriteMessage("\n[段{0}] 该段重算路径为空（不足 2 帧），未绘制，已回到预览。", seg);
                    continue;
                }
                // 末端精确对齐光标：仅当前进段用了自适应单圆弧时执行；最小半径回退时不强移（避免车身平移）。
                // 倒退段不需要：车尾控制点天生精确到达光标。
                if (driveJig.Dir > 0 && driveJig.RearRadiusOverride.HasValue)
                {
                    var targetFront = TruckKinematics.FrontAxle(segFrames[segFrames.Count - 1], _params);
                    segFramesFine[segFramesFine.Count - 1] = segFramesFine[segFramesFine.Count - 1].WithFrontAt(_params, targetFront);
                }

                // 追加到总路径（跳首帧避免与当前帧重叠）
                frameHist.Add(curFrame);
                segFrameCount.Add(segFramesFine.Count - 1);
                var ids = new List<ObjectId>();
                segEntityIds.Add(ids);  // ids 是引用，出图事务往里填；先压栈保证三份历史不错位
                segmentEnvelopeFootprints.Add(
                    TruckKinematics.BuildEnvelopeFootprints(segFramesFine, _params));
                for (int i = 1; i < segFramesFine.Count; i++) allFrames.Add(segFramesFine[i]);
                curFrame = segFramesFine[segFramesFine.Count - 1];
                _lastHeading = curFrame.U1;

                stage = "段" + seg + " 出图";
                // 先写本段轮迹与节点车体；随后单独替换累计最外包络。
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    DrawingUtil.EnsureLayer(db, tr, LayerEnvelope, 2);
                    DrawingUtil.EnsureLayer(db, tr, LayerWheels, 1);
                    DrawingUtil.EnsureLayer(db, tr, LayerFrontTrack, 6);
                    DrawingUtil.EnsureLayer(db, tr, LayerBody, 3);
                    DrawSegmentTracks(db, tr, segFramesFine, null, ids,
                        drawEnvelope: false);
                    if (_params.ShowAllNodeVehicles)
                    {
                        if (_params.ShowBody)
                            AddBody(db, tr, curFrame, LayerBody, ids);
                        if (_params.ShowWheels)
                        {
                            int[] segSigns = TruckKinematics.ComputeSteerSigns(segFramesFine);
                            DrawTires(db, tr, curFrame, _params,
                                segSigns[segSigns.Length - 1], ids);
                        }
                    }
                    tr.Commit();
                }

                stage = "段" + seg + " 更新累计包络";
                ReplaceCumulativeEnvelope(db, allFrames, cumulativeEnvelopeIds);

                bool segAdaptive = driveJig.RearRadiusOverride.HasValue && driveJig.StraightLen < 1e-9;
                double segActualR = segAdaptive ? (driveJig.RearRadiusOverride.Value / _params.UnitScale)
                                                : (TruckKinematics.TurnRadius(_params) / _params.UnitScale);
                // v4.7：回退分支不再是固定的 90°，而是「沿最小半径圆尽量转向光标」。
                string segFallback = segAdaptive ? "" : "（目标太近，已按最小半径尽量转向光标，车头到不了光标点）";
                ed.WriteMessage("\n[段{0}] 已确认（{7}）：{1}转 {2:F1}° + {3}。{4}R={5:F2}m{6}",
                    seg,
                    driveJig.TurnAngleDeg > 0 ? (driveJig.SteerSign > 0 ? "左" : "右") : "",
                    driveJig.TurnAngleDeg,
                    driveJig.StraightLen < 1e-9 ? "圆弧" : "直行 " + driveJig.StraightLen.ToString("F0"),
                    segAdaptive ? "自适应圆弧 " : "最小半径 ",
                    segActualR,
                    segFallback,
                    driveJig.Dir > 0 ? "前进" : "倒退");
            }

            // v4.9.18：全部段都被撤销、只剩初始车体时，也把初始车体打成块再退出——
            // 否则图上留下零散的车体图元，与「X 结束后路径整块」的行为不一致。
            if (allFrames.Count < 2)
            {
                stage = "初始车体打块";
                GroupPathIntoBlock(db, ed, initIds);
                ed.WriteMessage("\n未确认任何行驶段；初始车辆已打成块。");
                return;
            }

            stage = "生成最终精确累计包络";
            ReplaceCumulativeEnvelope(
                db, allFrames, cumulativeEnvelopeIds, fullQuality: true,
                segmentFootprints: segmentEnvelopeFootprints);

            // 累计包络已在每次确认/撤销后更新；最终只补四半径与末端车体。
            stage = "最终出图";
            var finalIds = new List<ObjectId>();
            DrawResult(db, allFrames, null, false, false, finalIds,
                drawStartVehicle: false,
                drawEndVehicle: !_params.ShowAllNodeVehicles,
                drawEnvelope: false,
                drawMotionTracks: false);
            ReportDrawnExtent(ed, allFrames, "TRUCKDRIVE");
            ed.WriteMessage("\n已生成连续驾驶路径：共 {0} 段，{1} 帧。灰色中间姿态未绘制。", segEntityIds.Count, allFrames.Count);

            // v4.9.11：X 结束后把整条路径的全部图元（初始车体 + 各段轮迹/节点车体 +
            // 全局最外包络、最终车体与半径标注）收进一个命名块，方便整体移动/复制/删除。
            stage = "打块";
            var blockIds = new List<ObjectId>();
            blockIds.AddRange(initIds);
            foreach (var segIds in segEntityIds) blockIds.AddRange(segIds);
            blockIds.AddRange(cumulativeEnvelopeIds);
            blockIds.AddRange(finalIds);
            GroupPathIntoBlock(db, ed, blockIds);
        }

        /// <summary>
        /// v4.9.11：把一组图元收进一个命名块（TruckPath、TruckPath_1、… 自动编号）。
        /// 做法：新建 BlockTableRecord → DeepCloneObjects 把图元克隆进块定义 →
        /// 在当前空间原点插入 BlockReference（恒等变换，几何位置与原件完全一致）→
        /// 删除原图元。全部在同一事务内，任何一步失败整体回滚，原图元原样保留。
        /// </summary>
        private void GroupPathIntoBlock(Database db, Editor ed, List<ObjectId> ids)
        {
            if (ids == null || ids.Count == 0) return;
            var uniq = new List<ObjectId>();
            var seen = new HashSet<ObjectId>();
            foreach (var id in ids)
            {
                if (id.IsNull || id.IsErased || !id.IsValid) continue;
                if (seen.Add(id)) uniq.Add(id);
            }
            if (uniq.Count == 0) return;

            try
            {
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                    string name = "TruckPath";
                    int k = 1;
                    while (bt.Has(name)) { name = "TruckPath_" + k; k++; }

                    var btr = new BlockTableRecord();
                    btr.Name = name;
                    bt.UpgradeOpen();
                    ObjectId btrId = bt.Add(btr);
                    tr.AddNewlyCreatedDBObject(btr, true);

                    var idCol = new ObjectIdCollection(uniq.ToArray());
                    var map = new IdMapping();
                    db.DeepCloneObjects(idCol, btrId, map, false);

                    // 原点插入块参照：克隆进块定义的坐标就是世界坐标，
                    // 恒等变换插入后位置与原件完全一致。
                    var curSpace = (BlockTableRecord)tr.GetObject(db.CurrentSpaceId, OpenMode.ForWrite);
                    var br = new BlockReference(Point3d.Origin, btrId);
                    br.SetDatabaseDefaults();
                    curSpace.AppendEntity(br);
                    tr.AddNewlyCreatedDBObject(br, true);

                    // 删除原图元（放最后：本事务内任一步失败都会整体回滚，原件不动）
                    foreach (var id in uniq)
                    {
                        var ent = tr.GetObject(id, OpenMode.ForWrite, false) as Entity;
                        if (ent != null && !ent.IsErased) ent.Erase();
                    }

                    tr.Commit();
                    ed.WriteMessage("\n[块] 已把 {0} 个图元打成块「{1}」（可整体移动/复制/删除；要编辑内部用炸开）。", uniq.Count, name);
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[块] 打块失败（{0}: {1}），已保留原图元不打包。", ex.GetType().Name, ex.Message);
            }
        }

        /// <summary>
        /// v4.6：撤销最后确认的一段 —— 删除该段画出的图元、剪掉它追加的帧、把货车姿态退回该段起点。
        /// 返回实际删除的图元个数。
        /// </summary>
        private int UndoLastSegment(Database db, Editor ed, List<Frame> allFrames, List<int> segFrameCount,
                                    List<List<ObjectId>> segEntityIds, List<Frame> frameHist,
                                    List<List<Point2d[]>> segmentEnvelopeFootprints,
                                    ref Frame curFrame)
        {
            int last = segEntityIds.Count - 1;
            if (last < 0) return 0;

            var ids = segEntityIds[last];
            int erased = 0;
            string err = null;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                foreach (var id in ids)
                {
                    if (id.IsNull) continue;
                    try
                    {
                        var ent = tr.GetObject(id, OpenMode.ForWrite) as Entity;
                        if (ent == null || ent.IsErased) continue;
                        ent.Erase();
                        erased++;
                    }
                    catch (System.Exception ex)
                    {
                        // 不裸 catch：图元可能已被别的操作删掉，但真出错必须让用户看得见
                        err = ex.GetType().Name + ": " + ex.Message;
                    }
                }
                tr.Commit();
            }
            if (err != null) ed.WriteMessage("\n[撤销] 部分图元删除失败：{0}", err);

            // 剪掉该段追加的帧
            if (last < segFrameCount.Count)
            {
                int n = segFrameCount[last];
                if (n > 0 && n <= allFrames.Count)
                    allFrames.RemoveRange(allFrames.Count - n, n);
                segFrameCount.RemoveAt(last);
            }
            segEntityIds.RemoveAt(last);
            if (segmentEnvelopeFootprints != null && last < segmentEnvelopeFootprints.Count)
                segmentEnvelopeFootprints.RemoveAt(last);

            // 车姿退回该段起点
            if (last < frameHist.Count)
            {
                curFrame = frameHist[last];
                frameHist.RemoveAt(last);
                _lastHeading = curFrame.U1;
            }
            return erased;
        }

        /// <summary>
        /// 每次确认或撤销后，用全部已确认帧重算唯一累计最外包络。
        /// 擦除旧包络与写入新包络处于同一事务，失败时旧结果不会丢失。
        /// </summary>
        private void ReplaceCumulativeEnvelope(
            Database db, List<Frame> allFrames, List<ObjectId> currentIds,
            bool fullQuality = false,
            IList<List<Point2d[]>> segmentFootprints = null)
        {
            List<Frame> envelopeFrames = fullQuality
                ? allFrames
                : TruckKinematics.PreviewFrames(
                    allFrames, CumulativeEnvelopePreviewFrameLimit);
            List<Point2d> cachedEnvelope = null;
            int cachedFootprintCount = 0;
            if (fullQuality && segmentFootprints != null && segmentFootprints.Count > 0)
            {
                var footprints = new List<Point2d[]>();
                for (int i = 0; i < segmentFootprints.Count; i++)
                {
                    List<Point2d[]> segment = segmentFootprints[i];
                    if (segment == null) continue;
                    footprints.AddRange(segment);
                }
                cachedFootprintCount = footprints.Count;
                cachedEnvelope = TruckKinematics.EnvelopeTracksFromFootprints(footprints);
            }
            var replacementIds = new List<ObjectId>();
            using (var tr = db.TransactionManager.StartTransaction())
            {
                foreach (ObjectId id in currentIds)
                {
                    if (id.IsNull || !id.IsValid || id.IsErased) continue;
                    var entity = tr.GetObject(id, OpenMode.ForWrite, false) as Entity;
                    if (entity != null && !entity.IsErased) entity.Erase();
                }

                if (_params.ShowEnvelope && envelopeFrames != null && envelopeFrames.Count >= 2)
                {
                    DrawingUtil.EnsureLayer(db, tr, LayerEnvelope, 2);
                    if (cachedEnvelope != null && cachedEnvelope.Count >= 3)
                        DrawEnvelopeBoundary(
                            db, tr, cachedEnvelope, LayerEnvelope, replacementIds,
                            cachedFootprintCount);
                    else
                        DrawSegmentTracks(db, tr, envelopeFrames, null, replacementIds,
                            drawEnvelope: true, drawMotionTracks: false);
                }
                tr.Commit();
            }

            currentIds.Clear();
            currentIds.AddRange(replacementIds);
        }
        #endregion

        #region 一键最小半径 90° 转弯
        /// <summary>
        /// TRUCKTURN90：放置车辆后，以当前最小转弯半径原地转 90°，车辆最终停在回正位置
        /// （车头方向与原方向垂直，前轮转角为 0）。
        /// </summary>
        [CommandMethod("TRUCKTURN90")]
        public void TruckTurn90()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            if (doc == null) return;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            string stage = "载入已存参数";
            try
            {
                TruckTurn90Body(doc, ed, db, ref stage);
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[TRUCKTURN90 异常 @ {0}] {1}: {2}\n{3}",
                    stage, ex.GetType().FullName, ex.Message, ex.StackTrace);
            }
        }

        private void TruckTurn90Body(Document doc, Editor ed, Database db, ref string stage)
        {
            if (!PromptVehicleParams(db, ed, ref stage)) return;

            // 阶段一：放置前轴中心起始位置
            stage = "阶段一 放置起始点";
            var placeJig = new TruckPlaceJig(_params, _lastHeading, Point3d.Origin);
            var r1 = SafeDrag(ed, placeJig, "阶段一 放置起始点");
            if (r1 == null || r1.Status != PromptStatus.OK) { ed.WriteMessage("\n已取消。"); return; }
            Point2d startFront = placeJig.Result;

            // 阶段二：确定车头朝向
            stage = "阶段二 确定车头朝向";
            var headJig = new TruckHeadingJig(_params, startFront, _lastHeading);
            var r2 = SafeDrag(ed, headJig, "阶段二 确定车头朝向");
            if (r2 == null || r2.Status != PromptStatus.OK) { ed.WriteMessage("\n已取消。"); return; }
            Vector2d heading = headJig.Result;
            _lastHeading = heading;

            // 阶段三：选择转弯方向
            var pko = new PromptKeywordOptions("\n90°转弯方向 [左(L)/右(R)] <左>: ", "Left Right");
            pko.Keywords.Default = "Left";
            pko.AllowNone = true;
            var pkr = ed.GetKeywords(pko);
            if (pkr.Status != PromptStatus.OK && pkr.Status != PromptStatus.None)
            { ed.WriteMessage("\n已取消。"); return; }
            int steerSign = (pkr.Status == PromptStatus.OK && pkr.StringResult == "Right") ? -1 : 1;

            // v4.9：选择行驶方向（前进 / 倒退）
            var pkd = new PromptKeywordOptions("\n行驶方向 [前进(F)/倒退(B)] <前进>: ", "Forward Back");
            pkd.Keywords.Default = "Forward";
            pkd.AllowNone = true;
            var pkdr = ed.GetKeywords(pkd);
            if (pkdr.Status != PromptStatus.OK && pkdr.Status != PromptStatus.None)
            { ed.WriteMessage("\n已取消。"); return; }
            int dir = (pkdr.Status == PromptStatus.OK && pkdr.StringResult == "Back") ? -1 : 1;

            WarnIfTightTurn(ed);

            stage = "生成 90° 转弯 + 拉直路径";
            // v4.4：铰接车先以最小半径转 90°，再继续直行最短距离把挂车拉直到与车头同向
            // v4.9：倒退(dir=−1)时不做拉直 —— 铰接角对倒退是指数发散的，「倒着拉直挂车」无解。
            var frames = TruckKinematics.SimulateTurn90AndStraighten(_params, startFront, heading, steerSign, OutputStepDeg, dir);
            if (frames == null || frames.Count < 2)
            { ed.WriteMessage("\n未生成路径（帧数不足），请检查车辆参数。"); return; }
            stage = "出图";
            var ids = new List<ObjectId>();
            DrawResult(db, frames, null, true, true, ids);
            ReportDrawnExtent(ed, frames, "TRUCKTURN90");
            // v4.9.12：出图后自动打成块，方便整体移动/复制/删除
            stage = "打块";
            GroupPathIntoBlock(db, ed, ids);

            var radP = TruckKinematics.TurnRadii(_params);
            string dirTxt = dir > 0 ? "前进" : "倒退";
            if (dir > 0)
                ed.WriteMessage("\n已生成最小半径 90° 转弯（{0}）：{1}转，共 {2} 帧。车辆已处于回正位置（车头+挂车同向）。",
                                dirTxt, steerSign > 0 ? "左" : "右", frames.Count);
            else
                ed.WriteMessage("\n已生成 90° 倒退转弯：车尾向{0}甩，共 {1} 帧。{2}",
                                steerSign > 0 ? "左" : "右", frames.Count,
                                _params.Articulated
                                    ? "车尾按平缓半径甩尾（稳态铰接角 ≤ 60% 限位），铰接角自动收敛 —— 不再 jackknife。"
                                    : "倒退由后轴控制，后轴沿圆弧精确到位。");
            ed.WriteMessage("  车辆最小转弯半径(m): R1={0:F1} 前轮外缘={1:F1} 车身外角={2:F1} 后内轮={3:F1}",
                radP.Centerline / _params.UnitScale, radP.CurbToCurb / _params.UnitScale,
                radP.WallToWall / _params.UnitScale, radP.InnerTire / _params.UnitScale);
        }
        #endregion

        #region v4.6.1：图纸单位探测 / 参数摘要 / 出图自检
        /// <summary>
        /// 读 INSUNITS 推算「1 米 = ? 图形单位」。返回 0 表示探测不到（英制或未设置），此时不要瞎猜。
        /// 映射：4=毫米 5=厘米 6=米。
        ///
        /// 这是「命令跑完了却什么都看不到」的头号嫌疑：图纸按毫米画（常见的总平面图），
        /// 而选型窗的图纸单位停留在默认的「米」，画出来的车会比实际小 1000 倍，
        /// 在当前视图里等于不存在。
        /// </summary>
        private static double DetectDrawingUnitScale(Database db)
        {
            try
            {
                object v = Application.GetSystemVariable("INSUNITS");
                if (v == null) return 0;
                int u = Convert.ToInt32(v);
                switch (u)
                {
                    case 4: return 1000.0;   // 毫米
                    case 5: return 100.0;    // 厘米
                    case 6: return 1.0;      // 米
                    default: return 0;       // 英寸/英尺/无单位 → 不猜
                }
            }
            catch (System.Exception) { return 0; }
        }

        /// <summary>本图是否已经保存过货车参数（决定要不要按 INSUNITS 自动定图纸单位）。</summary>
        private static bool HasSavedParams(Database db)
        {
            try
            {
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(db.NamedObjectsDictionaryId, OpenMode.ForRead);
                    if (!nod.Contains(DictName)) return false;
                    var dict = (DBDictionary)tr.GetObject(nod.GetAt(DictName), OpenMode.ForRead);
                    bool has = dict.Contains("Params");
                    tr.Commit();
                    return has;
                }
            }
            catch (System.Exception) { return false; }
        }

        /// <summary>
        /// 两个命令共用的开头：载入已存参数 → 弹选型窗 → 存参数 → 打印参数摘要。
        /// 返回 false = 用户取消或参数无效，调用方直接 return。
        /// </summary>
        private bool PromptVehicleParams(Database db, Editor ed, ref string stage)
        {
            if (_params == null)
            {
                _params = VehicleParams.Defaults();
                ed.WriteMessage("\n[参数] 上次的参数已失效，本次重置为默认值。");
            }
            LoadFromDrawing();

            // v5.0：图纸单位默认「米」（1 米 = 1 图形单位）；不再按 INSUNITS 自动套用。
            // INSUNITS 不一致时选型窗内仍有红字告警兜底。
            double drawingScale = DetectDrawingUnitScale(db);

            stage = "车辆选型窗";
            using (var sizeForm = new TruckSizeForm(_params, drawingScale))
            {
                if (Application.ShowModalDialog(sizeForm) != System.Windows.Forms.DialogResult.OK)
                { ed.WriteMessage("\n已取消选型。"); return false; }
                if (sizeForm.Result == null)
                { ed.WriteMessage("\n[选型] 未能生成有效参数，命令终止（请重开选型窗检查输入）。"); return false; }

                _params = sizeForm.Result;
            }
            stage = "保存参数到图形";
            SaveToDrawing();
            ReportParamsAndUnit(ed, drawingScale);
            return true;
        }

        /// <summary>选型确认后把关键参数打到命令行；单位与本图不一致时红字警告。</summary>
        private void ReportParamsAndUnit(Editor ed, double drawingScale)
        {
            var p = _params;
            string unit = p.UnitScale >= 999.5 ? "毫米" : (p.UnitScale >= 99.5 ? "厘米" : "米");
            ed.WriteMessage("\n[参数] {0}｜图纸单位={1}（1 米 = {2} 图形单位）｜车宽 {3:F2}m",
                p.Articulated ? "铰接半挂" : "刚性单车", unit, p.UnitScale, p.WidthMeters());
            if (p.Articulated)
                ed.WriteMessage("｜牵引车 {0:F2}m + 挂车 {1:F2}m = 列车 {2:F2}m",
                    p.TractorLengthMeters(), p.TrailerLengthMeters(), p.LengthMeters());
            else
                ed.WriteMessage("｜整车长 {0:F2}m", p.LengthMeters());
            ed.WriteMessage("｜最小转弯半径 R1={0:F2}m", TruckKinematics.TurnRadius(p) / p.UnitScale);

            if (drawingScale > 0 && Math.Abs(drawingScale - p.UnitScale) > 1e-6)
            {
                double ratio = p.UnitScale / drawingScale;
                ed.WriteMessage("\n[单位警告] 本图 INSUNITS 判定 1 米 = {0} 图形单位，你选的是「{1}」（1 米 = {2}），相差 {3:F0} 倍。"
                    + "若画完看不到图形，把「图纸单位」改成与 INSUNITS 一致再重画。",
                    drawingScale, unit, p.UnitScale, ratio > 1 ? ratio : 1.0 / ratio);
            }
        }

        /// <summary>v4.6.1：包一层 ed.Drag。Jig 内部抛异常时 CAD 只留一行 InvokeWorkerWithException，
        /// 看不出是哪一步；这里把阶段名 + 类型 + 消息 + 堆栈打全。</summary>
        private static PromptResult SafeDrag(Editor ed, DrawJig jig, string stageName)
        {
            try { return ed.Drag(jig); }
            catch (System.Exception ex)
            {
                ed.WriteMessage("\n[{0} 预览异常] {1}: {2}\n{3}", stageName, ex.GetType().FullName, ex.Message, ex.StackTrace);
                return null;
            }
        }

        /// <summary>
        /// 出图后强制重生成，并打印结果的实际坐标范围。
        /// 「命令跑完了却什么都看不到」时，靠这两行立刻区分：是根本没画出来，还是画在了视图之外。
        /// </summary>
        private void ReportDrawnExtent(Editor ed, List<Frame> frames, string cmd)
        {
            double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
            foreach (var f in frames)
            {
                foreach (var c in TruckKinematics.Corners(f, _params))
                {
                    if (c.X < minX) minX = c.X;
                    if (c.X > maxX) maxX = c.X;
                    if (c.Y < minY) minY = c.Y;
                    if (c.Y > maxY) maxY = c.Y;
                }
            }
            try { ed.Regen(); }
            catch (System.Exception ex) { ed.WriteMessage("\n[重生成失败] {0}: {1}", ex.GetType().Name, ex.Message); }

            if (minX <= maxX)
                ed.WriteMessage("\n[{0}] 绘制范围：X[{1:F1}, {2:F1}]  Y[{3:F1}, {4:F1}]（图形单位，1 米 = {5}）。"
                    + "看不到图形时：① 确认「图纸单位」与本图 INSUNITS 一致；② ZOOM → 范围(Z E)。",
                    cmd, minX, maxX, minY, maxY, _params.UnitScale);
        }
        #endregion

        /// <summary>紧转弯（整圈稳态不可行）提示，仅提示不阻断。刚性单车不会折叠，无需提示。</summary>
        private void WarnIfTightTurn(Editor ed)
        {
            if (!_params.Articulated) return;
            double R1 = TruckKinematics.TurnRadius(_params);
            if (R1 < _params.TrailerKingpinToRearAxle)
                ed.WriteMessage("\n提示：当前前轮转角下整圈稳态转弯不可行(挂车会折叠)，拐角转弯仍会正常绘制。"
                              + "如需整圈，请减小前轮转角或加长轴距。");
        }

        #region 绘制
        /// <summary>
        /// 绘制一帧车辆的轮胎外观。
        /// v4.6：ids 非 null 时把新建图元 Id 记进去，供「撤销上一段」删除。
        /// </summary>
        private void DrawTires(Database db, Transaction tr, Frame f, VehicleParams p, int steerSign, List<ObjectId> ids = null)
        {
            // v4.9.20 修复：轮胎画在 LayerWheels 上，但本函数此前不保证该图层存在 ——
            // TRUCKDRIVE 初始车体事务只 Ensure 了 LayerBody，新图（或无轮迹层的图）
            // 首次 NETLOAD 直接 TRUCKDRIVE 时 pl.Layer=不存在的图层名 → eKeyNotFound，
            // 命令在「生成起始帧」阶段整段崩掉（跑过一次 TRUCKTURN90 后图层已建，症状消失）。
            // 在这里自给自足（幂等），所有调用点一劳永逸。
            DrawingUtil.EnsureLayer(db, tr, LayerWheels, 1);
            double tireLen = 0.5 * p.UnitScale;
            double tireWidth = 0.25 * p.UnitScale;
            foreach (var rect in TruckKinematics.TireRects(f, p, tireLen, tireWidth, steerSign))
                Add(ids, DrawingUtil.AddRectangle(db, tr, rect, LayerWheels));
        }

        /// <summary>v4.6：新建图元 Id 记入清单（ids 为 null 时忽略，例如最终出图不需要撤销）</summary>
        private static void Add(List<ObjectId> ids, ObjectId id)
        {
            if (ids != null && !id.IsNull) ids.Add(id);
        }

        /// <summary>绘制一段路径的包络、轮迹、前轴轨迹（受 _params 显示选项控制）</summary>
        private void DrawSegmentTracks(Database db, Transaction tr, List<Frame> frames,
                                       string envelopeLayer = null, List<ObjectId> ids = null,
                                       bool drawEnvelope = true, bool drawMotionTracks = true)
        {
            if (frames == null || frames.Count < 2) return;
            string envLayer = envelopeLayer ?? LayerEnvelope;

            // 扫掠包络（v4.3 为所有车体 footprint 的并集外边界，一条闭合黄线）
            // v4.3.1：若路径为圆周运动，则把到转弯中心等距的边界段拟合成 bulge 圆弧，避免折线锯齿。
            if (drawEnvelope && _params.ShowEnvelope)
            {
                DrawingUtil.EnsureLayer(db, tr, envLayer, 2);
                var envelope = TruckKinematics.EnvelopeTracks(frames, _params);
                DrawEnvelopeBoundary(db, tr, envelope, envLayer, ids, frames.Count);
            }

            // 轮迹 + 前轴轨迹
            bool needWheels = drawMotionTracks && _params.ShowWheels;
            bool needFrontTrack = drawMotionTracks && _params.ShowFrontTrack;
            if (!needWheels && !needFrontTrack) return;

            if (needWheels) DrawingUtil.EnsureLayer(db, tr, LayerWheels, 1);
            if (needFrontTrack) DrawingUtil.EnsureLayer(db, tr, LayerFrontTrack, 6);

            double half = _params.WheelTrack / 2.0;
            var frontCenter = new List<Point2d>();
            var fL = new List<Point2d>(); var fR = new List<Point2d>();
            var rL = new List<Point2d>(); var rR = new List<Point2d>();
            var tL = new List<Point2d>(); var tR = new List<Point2d>();

            foreach (var f in frames)
            {
                Vector2d n1 = new Vector2d(-f.U1.Y, f.U1.X);
                Vector2d n2 = new Vector2d(-f.U2.Y, f.U2.X);
                Point2d frontAxle = f.O1 + _params.TractorWheelbase * f.U1;
                frontCenter.Add(frontAxle);
                fL.Add(frontAxle + half * n1); fR.Add(frontAxle - half * n1);
                rL.Add(f.O1 + half * n1); rR.Add(f.O1 - half * n1);
                if (_params.Articulated)
                {
                    tL.Add(f.O2 + half * n2); tR.Add(f.O2 - half * n2);
                }
            }

            if (needFrontTrack)
                Add(ids, DrawingUtil.AddPolyline(db, tr, frontCenter, false, LayerFrontTrack));
            if (needWheels)
            {
                Add(ids, DrawingUtil.AddPolyline(db, tr, fL, false, LayerWheels));
                Add(ids, DrawingUtil.AddPolyline(db, tr, fR, false, LayerWheels));
                Add(ids, DrawingUtil.AddPolyline(db, tr, rL, false, LayerWheels));
                Add(ids, DrawingUtil.AddPolyline(db, tr, rR, false, LayerWheels));
                if (_params.Articulated)
                {
                    Add(ids, DrawingUtil.AddPolyline(db, tr, tL, false, LayerWheels));
                    Add(ids, DrawingUtil.AddPolyline(db, tr, tR, false, LayerWheels));
                }
            }
        }

        /// <summary>
        /// 将已求得的并集最外环按统一规则稀化并绘制。普通实时累计和最终缓存合并
        /// 共用此出口，保证增量缓存只改变计算路径，不改变 CAD 图元样式或安全外扩。
        /// </summary>
        private void DrawEnvelopeBoundary(
            Database db, Transaction tr, List<Point2d> envelope,
            string envLayer, List<ObjectId> ids, int sourceCount)
        {
            if (TruckKinematics.EnvelopeFallbackUsed)
            {
                // 并集边界成环失败，已退化成凸包：凸包会填平挂车 off-tracking 的真实凹口，
                // 包络比实际偏宽。这是保底不是正常结果，必须让用户看得见。
                var d0 = Application.DocumentManager.MdiActiveDocument;
                if (d0 != null)
                    d0.Editor.WriteMessage(
                        "\n[警告] 包络并集边界成环失败，已退化为凸包（{0} 个输入），结果偏宽。",
                        sourceCount);
            }
            if (envelope == null || envelope.Count < 3) return;

            // v4.9.8：包络使用外包补偿后的 Polyline，保留车体真实折角，避免样条外鼓。
            var simplified = GeometryUtil.SimplifyEnvelopeForSpline(
                envelope, EnvelopeSimplifyTol, smoothIters: EnvelopeSmoothIters);
            Add(ids, DrawingUtil.AddPolyline(
                db, tr, simplified.loop, true, envLayer));

            var doc = Application.DocumentManager.MdiActiveDocument;
            if (doc != null)
                doc.Editor.WriteMessage(
                    "\n[包络] 顶点 {0} → {1}，外扩补偿 {2:F3} m",
                    envelope.Count, simplified.loop.Count, simplified.outset);
        }

        /// <summary>
        /// 绘制一帧车体：牵引车（驾驶室）+ 车架纵梁 + 挂车。
        /// v4.4 起驾驶室后方补一条窄车架——真车挂车前部悬在车架上方，
        /// 只画驾驶室会在驾驶室与挂车之间留出一段一眼假的空洞。
        /// </summary>
        private void AddBody(Database db, Transaction tr, Frame f, string layer, List<ObjectId> ids = null)
        {
            if (_params.Articulated)
            {
                Add(ids, DrawingUtil.AddRectangle(db, tr, TruckKinematics.TractorRect(f, _params), layer));
                var chassis = TruckKinematics.ChassisRect(f, _params);
                if (chassis != null) Add(ids, DrawingUtil.AddRectangle(db, tr, chassis, layer));
                Add(ids, DrawingUtil.AddRectangle(db, tr, TruckKinematics.TrailerRect(f, _params), layer));
            }
            else
            {
                // v4.9.17：刚性车改画精致轮廓（车头斜面+挡风玻璃+后视镜+车厢），
                // 不再是一个矩形画两遍。
                foreach (var seg in TruckKinematics.RigidBodyOutlines(f, _params))
                    Add(ids, DrawingUtil.AddPolyline(db, tr, new List<Point2d>(seg), false, layer));
            }
        }

        private void DrawResult(Database db, List<Frame> frames, string envelopeLayer = null,
                                bool drawGhost = true, bool drawTracks = true,
                                List<ObjectId> ids = null,
                                bool drawStartVehicle = true, bool drawEndVehicle = true,
                                bool drawEnvelope = true, bool drawMotionTracks = true)
        {
            if (frames == null || frames.Count == 0) return;

            using (var tr = db.TransactionManager.StartTransaction())
            {
                if (_params.ShowBody) DrawingUtil.EnsureLayer(db, tr, LayerBody, 3);
                if (drawGhost && _params.ShowGhost) DrawingUtil.EnsureLayer(db, tr, LayerGhost, 8);

                // 1) 包络 + 轮迹 + 前轴轨迹
                if (drawTracks)
                    DrawSegmentTracks(db, tr, frames, envelopeLayer, ids,
                        drawEnvelope, drawMotionTracks);

                // 计算每帧前轮转向符号（直行=0）
                int[] steerSigns = TruckKinematics.ComputeSteerSigns(frames);

                // 2) 中间姿态（约 11 个），让整条行驶过程可读
                if (drawGhost && _params.ShowGhost)
                {
                    int ghostStep = Math.Max(1, frames.Count / 12);
                    for (int i = ghostStep; i < frames.Count - 1; i += ghostStep)
                    {
                        AddBody(db, tr, frames[i], LayerGhost, ids);
                        if (_params.ShowWheels)
                            DrawTires(db, tr, frames[i], _params, steerSigns[i], ids);
                    }
                }

                // 3) 起始 / 结束 车体姿态 + 轮胎
                if (drawStartVehicle && _params.ShowBody)
                {
                    AddBody(db, tr, frames[0], LayerBody, ids);
                }
                if (drawStartVehicle && _params.ShowWheels)
                    DrawTires(db, tr, frames[0], _params, steerSigns[0], ids);

                var last = frames[frames.Count - 1];
                if (drawEndVehicle && _params.ShowBody)
                {
                    AddBody(db, tr, last, LayerBody, ids);
                }
                if (drawEndVehicle && _params.ShowWheels)
                    DrawTires(db, tr, last, _params, steerSigns[steerSigns.Length - 1], ids);

                // 4) 四种转弯半径标注（对应 AutoTURN turning template）
                if (_params.ShowRadii)
                {
                    var rad = TruckKinematics.TurnRadii(_params);
                    DrawingUtil.EnsureLayer(db, tr, LayerRadii, 5);
                    double off = 5.0 * _params.UnitScale;
                    Point2d startFront = TruckKinematics.FrontAxle(frames[0], _params);
                    Point2d radPt = new Point2d(startFront.X, startFront.Y + off);
                    string radTxt = string.Format("转弯半径(m): 设计R1={0:F1} 前轮外缘={1:F1} 车身外角={2:F1} 后内轮={3:F1}",
                        rad.Centerline / _params.UnitScale, rad.CurbToCurb / _params.UnitScale,
                        rad.WallToWall / _params.UnitScale, rad.InnerTire / _params.UnitScale);
                    Add(ids, DrawingUtil.AddMText(db, tr, radPt, radTxt, Math.Max(0.3 * _params.UnitScale, 1.0), LayerRadii));
                }

                tr.Commit();
            }
        }
        #endregion

        #region 参数持久化(存入图形命名字典)
        private void SaveToDrawing()
        {
            try
            {
                Database db = Application.DocumentManager.MdiActiveDocument.Database;
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(db.NamedObjectsDictionaryId, OpenMode.ForRead);
                    DBDictionary dict;
                    if (nod.Contains(DictName))
                        dict = (DBDictionary)tr.GetObject(nod.GetAt(DictName), OpenMode.ForWrite);
                    else
                    {
                        nod.UpgradeOpen();
                        dict = new DBDictionary();
                        nod.SetAt(DictName, dict);
                        tr.AddNewlyCreatedDBObject(dict, true);
                    }

                    Xrecord xr;
                    if (dict.Contains("Params"))
                        xr = (Xrecord)tr.GetObject(dict.GetAt("Params"), OpenMode.ForWrite);
                    else
                    {
                        xr = new Xrecord();
                        dict.SetAt("Params", xr);
                        tr.AddNewlyCreatedDBObject(xr, true);
                    }

                    xr.Data = new ResultBuffer(new TypedValue((int)DxfCode.Text, _params.ToCsv()));
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                // v4.5.1：不再静默。此前这里（以及 LoadFromDrawing）用裸 catch 吞异常，
                // 一旦参数持久化写坏，表现就是「下次进命令什么都不对」，无从查起。
                var d = Application.DocumentManager.MdiActiveDocument;
                if (d != null)
                    d.Editor.WriteMessage("\n[参数保存失败] {0}: {1}", ex.GetType().Name, ex.Message);
            }
        }

        private void LoadFromDrawing()
        {
            try
            {
                Document doc = Application.DocumentManager.MdiActiveDocument;
                if (doc == null) return;
                Database db = doc.Database;
                using (var tr = db.TransactionManager.StartTransaction())
                {
                    var nod = (DBDictionary)tr.GetObject(db.NamedObjectsDictionaryId, OpenMode.ForRead);
                    if (!nod.Contains(DictName)) return;
                    var dict = (DBDictionary)tr.GetObject(nod.GetAt(DictName), OpenMode.ForRead);
                    if (!dict.Contains("Params")) return;
                    var xr = (Xrecord)tr.GetObject(dict.GetAt("Params"), OpenMode.ForRead);
                    var arr = xr.Data.AsArray();
                    if (arr.Length > 0 && arr[0].Value is string s)
                    {
                        var loaded = VehicleParams.FromCsv(s);
                        // v4.5.1：解析出的参数必须可用，否则宁可用默认值。
                        // 旧格式（字段 <15）会被判成 UnitScale=1000（毫米），与 Defaults() 的
                        // 1.0（米）差 1000 倍 —— 画出来的图会差三个数量级，看上去就是"没画出来"。
                        if (loaded != null && loaded.UnitScale > 0 &&
                            !double.IsNaN(loaded.UnitScale) && !double.IsInfinity(loaded.UnitScale))
                            _params = loaded;
                        else
                            doc.Editor.WriteMessage("\n[参数读取] 已存参数无效，本次使用默认值。");
                    }
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                var d = Application.DocumentManager.MdiActiveDocument;
                if (d != null)
                    d.Editor.WriteMessage("\n[参数读取失败，使用默认值] {0}: {1}", ex.GetType().Name, ex.Message);
            }
        }
        #endregion
    }
}
