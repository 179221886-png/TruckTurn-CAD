// v4.9.5 调试：复现用户「不贴合」尖刺
// 场景：直行5m + 左转弯 90°（最小半径），铰接车。

using System;
using System.Collections.Generic;
using System.Linq;
using Gssoft.Gscad.Geometry;
using TruckTurn;

namespace TruckTurn.Verify
{
    public static class DumpEnvelope
    {
        public static void Run()
        {
            var p = VehicleParams.Defaults();

            // 直行 5m + 转弯 90°
            var seg1 = TruckKinematics.SimulateFromFrontAxle(p, new Point2d(0, 0), new Vector2d(0, 1),
                                                              1, 0.0, 5.0, 0.25);
            var seg2 = TruckKinematics.SimulateFromFrontAxle(p, TruckKinematics.FrontAxle(seg1[seg1.Count - 1], p),
                                                              seg1[seg1.Count - 1].U1,
                                                              1, 90.0, 0.0, 0.25);
            var frames = new List<Frame>();
            foreach (var f in seg1) frames.Add(f);
            foreach (var f in seg2) frames.Add(f);
            Console.WriteLine($"总帧数: {frames.Count}");

            // 直接调内部 envelope 算法
            var envelope = TruckKinematics.EnvelopeTracks(frames, p);
            Console.WriteLine($"原始 envelope 顶点数: {envelope.Count}");

            // 检查原始 envelope 是否 CCW
            double area2 = 0;
            for (int i = 0; i < envelope.Count; i++)
            {
                var a = envelope[i];
                var b = envelope[(i + 1) % envelope.Count];
                area2 += a.X * b.Y - b.X * a.Y;
            }
            Console.WriteLine($"原始 envelope 2A = {area2:F4}  ({(area2 > 0 ? "CCW" : "CW")})");

            var (simp, totalOut) = GeometryUtil.SimplifyEnvelopeForSpline(envelope, 0.005);
            var (simp2, totalOut2) = GeometryUtil.SimplifyEnvelopeForSpline(envelope, 0.002);
            var smoothed = GeometryUtil.SmoothEnvelopeLoop(envelope, iters: 60);
            // 直接 RDP 测 SMOOTH
            var simpS = GeometryUtil.SimplifyClosedLoop(smoothed, 0.005);
            // 大容差 RDP：直接对 RAW、SMOOTH 两种输入试 0.05/0.10 容差
            var simpRaw05 = GeometryUtil.SimplifyClosedLoop(envelope, 0.05);
            var simpRaw10 = GeometryUtil.SimplifyClosedLoop(envelope, 0.10);
            var simpCol05 = GeometryUtil.SimplifyClosedLoop(smoothed, 0.05);
            var simpCol10 = GeometryUtil.SimplifyClosedLoop(smoothed, 0.10);
            Console.WriteLine($"原始 {envelope.Count} 顶点 → 平滑后 {smoothed.Count} → RDP(5mm) {simp.Count} pts out={totalOut * 1000:F2}mm → RDP(2mm) {simp2.Count} pts out={totalOut2 * 1000:F2}mm");
            Console.WriteLine($"   直 RDP  SMOOTH→{simpS.Count}");
            Console.WriteLine($"   RAW RDP(5cm)={simpRaw05.Count}  RAW RDP(10cm)={simpRaw10.Count}  SMOOTH RDP(5cm)={simpCol05.Count}  SMOOTH RDP(10cm)={simpCol10.Count}");
            // 直 RDP 的 AABB 验证
            Console.WriteLine($"   直 RDP on SMOOTH AABB:");
            double minXc = double.MaxValue, maxXc = double.MinValue, minYc = double.MaxValue, maxYc = double.MinValue;
            foreach (var pt in simpS) { if (pt.X < minXc) minXc = pt.X; if (pt.X > maxXc) maxXc = pt.X; if (pt.Y < minYc) minYc = pt.Y; if (pt.Y > maxYc) maxYc = pt.Y; }
            Console.WriteLine($"      X[{minXc:F2},{maxXc:F2}] Y[{minYc:F2},{maxYc:F2}]");
            // SIMP 最终环 AABB
            double minXf = double.MaxValue, maxXf = double.MinValue, minYf = double.MaxValue, maxYf = double.MinValue;
            foreach (var pt in simp) { if (pt.X < minXf) minXf = pt.X; if (pt.X > maxXf) maxXf = pt.X; if (pt.Y < minYf) minYf = pt.Y; if (pt.Y > maxYf) maxYf = pt.Y; }
            Console.WriteLine($"   SimplifyEnvelopeForSpline 输出环 AABB: X[{minXf:F2},{maxXf:F2}] Y[{minYf:F2},{maxYf:F2}]");

            // CSV 导出供 Python 绘图（v4.9.5 调试用）
            {
                string outCsv = System.IO.Path.Combine(
                    System.IO.Path.GetDirectoryName(typeof(DumpEnvelope).Assembly.Location) ?? ".",
                    "_env_dump.csv");
                using (var sw = new System.IO.StreamWriter(outCsv))
                {
                    sw.WriteLine("TYPE,IDX,X,Y");
                    for (int i = 0; i < envelope.Count; i++)
                        sw.WriteLine($"RAW,{i},{envelope[i].X:F6},{envelope[i].Y:F6}");
                    for (int i = 0; i < simpCol10.Count; i++)
                        sw.WriteLine($"SIMP_COL10,{i},{simpCol10[i].X:F6},{simpCol10[i].Y:F6}");
                    for (int i = 0; i < simpRaw05.Count; i++)
                        sw.WriteLine($"RAW_RDP05,{i},{simpRaw05[i].X:F6},{simpRaw05[i].Y:F6}");
                    // 用 SimplifyEnvelopeForSpline 输出（35 顶点）
                    sw.WriteLine($"=== RDP(5mm) 输出顶点 ===");
                    foreach (var pp in simp)
                        sw.WriteLine($"SIMP_FINAL,{pp.X:F6},{pp.Y:F6}");
                    for (int i = 0; i < smoothed.Count; i++)
                        sw.WriteLine($"SMOOTH,{i},{smoothed[i].X:F6},{smoothed[i].Y:F6}");
                    for (int i = 0; i < simp.Count; i++)
                        sw.WriteLine($"SIMP,{i},{simp[i].X:F6},{simp[i].Y:F6}");
                    for (int i = 0; i < simp2.Count; i++)
                        sw.WriteLine($"SIMP2,{i},{simp2[i].X:F6},{simp2[i].Y:F6}");
                    // 每 12 帧画一个车体（含首帧/末帧）
                    for (int id = 0; id < frames.Count; id += 12)
                    {
                        var c = TruckKinematics.Corners(frames[id], p);
                        for (int k = 0; k < c.Length; k++)
                            sw.WriteLine($"BODY,{id},{c[k].X:F6},{c[k].Y:F6}");
                    }
                    var cE = TruckKinematics.Corners(frames[frames.Count - 1], p);
                    for (int k = 0; k < cE.Length; k++)
                        sw.WriteLine($"BODY,{frames.Count - 1},{cE[k].X:F6},{cE[k].Y:F6}");
                }
                Console.WriteLine($"CSV 已写出: {outCsv}");
            }

            // AABB
            double minX = double.MaxValue, maxX = double.MinValue;
            double minY = double.MaxValue, maxY = double.MinValue;
            foreach (var pt in envelope)
            {
                if (pt.X < minX) minX = pt.X; if (pt.X > maxX) maxX = pt.X;
                if (pt.Y < minY) minY = pt.Y; if (pt.Y > maxY) maxY = pt.Y;
            }
            Console.WriteLine($"原始 envelope AABB: X[{minX:F2},{maxX:F2}] Y[{minY:F2},{maxY:F2}]");
            int spikes = 0;
            foreach (var pt in simp)
            {
                double dx = Math.Max(pt.X - maxX, minX - pt.X);
                double dy = Math.Max(pt.Y - maxY, minY - pt.Y);
                double dist = Math.Max(dx, dy);
                if (dist > 0.05) { spikes++; Console.WriteLine($"   外尖刺 ({pt.X:F3}, {pt.Y:F3}) 超 AABB {dist * 1000:F1} mm"); }
            }
            Console.WriteLine($"外尖刺（超出原始 envelope AABB > 5cm）共 {spikes} 个");

            // 写一个对比 SVG：原始+smooth+simp+outset 四层叠加
            string svgPath = @"_debug_envelope_layers.svg";
            using (var w = new System.IO.StreamWriter(svgPath))
            {
                double xMin = double.MaxValue, yMin = double.MaxValue, xMax = double.MinValue, yMax = double.MinValue;
                foreach (var pt in envelope) { if (pt.X < xMin) xMin = pt.X; if (pt.X > xMax) xMax = pt.X; if (pt.Y < yMin) yMin = pt.Y; if (pt.Y > yMax) yMax = pt.Y; }
                foreach (var pt in simp) { if (pt.X < xMin) xMin = pt.X; if (pt.X > xMax) xMax = pt.X; if (pt.Y < yMin) yMin = pt.Y; if (pt.Y > yMax) yMax = pt.Y; }
                double pad = 2.0;
                xMin -= pad; yMin -= pad; xMax += pad; yMax += pad;
                int W = 1000, H = 700;
                double scX = W / (xMax - xMin), scY = H / (yMax - yMin), sc = Math.Min(scX, scY);
                double tx = -xMin, ty = -yMin;
                Func<Point2d, (double X, double Y)> P = pt => ((pt.X + tx) * sc, H - (pt.Y + ty) * sc);
                w.WriteLine($"<svg xmlns='http://www.w3.org/2000/svg' width='{W}' height='{H}' viewBox='0 0 {W} {H}'>");
                w.WriteLine($"<rect width='{W}' height='{H}' fill='white'/>");
                w.WriteLine($"<text x='10' y='20' font-size='14'>orig={envelope.Count} simp={simp.Count} out={totalOut*1000:F1}mm</text>");

                // 原始 envelope（黑色）
                w.Write("<polyline points='");
                for (int i = 0; i <= envelope.Count; i++) { var pp2 = P(envelope[i % envelope.Count]); w.Write($"{pp2.X:F1},{pp2.Y:F1} "); }
                w.WriteLine("' fill='none' stroke='#888' stroke-width='0.5'/>");

                // smoothed（红色）
                w.Write("<polyline points='");
                for (int i = 0; i <= smoothed.Count; i++) { var pp2 = P(smoothed[i % smoothed.Count]); w.Write($"{pp2.X:F1},{pp2.Y:F1} "); }
                w.WriteLine("' fill='none' stroke='red' stroke-width='0.7'/>");

                // simp（绿色）
                w.Write("<polyline points='");
                for (int i = 0; i <= simp.Count; i++) { var pp2 = P(simp[i % simp.Count]); w.Write($"{pp2.X:F1},{pp2.Y:F1} "); }
                w.WriteLine("' fill='none' stroke='green' stroke-width='1.2'/>");

                // 车体起点/终点（蓝/橙）
                var f0 = frames[0];
                var fE = frames[frames.Count - 1];
                DrawTruck(w, P, f0, p, "blue");
                DrawTruck(w, P, fE, p, "orange");
                w.WriteLine("</svg>");
            }
            Console.WriteLine($"诊断 SVG: {svgPath}");

            // 找出原始 envelope 中转角最尖的顶点（外推最容易出尖刺的位置）
            Console.WriteLine("\n=== 原始 envelope 转角最大的 5 个顶点 ===");
            var topTurn = new List<(int idx, double deg, Point2d p)>();
            for (int i = 0; i < envelope.Count; i++)
            {
                var a = envelope[(i - 1 + envelope.Count) % envelope.Count];
                var c = envelope[i];
                var b = envelope[(i + 1) % envelope.Count];
                double e1x = c.X - a.X, e1y = c.Y - a.Y;
                double e2x = b.X - c.X, e2y = b.Y - c.Y;
                double cross = e1x * e2y - e1y * e2x;
                double dot = e1x * e2x + e1y * e2y;
                double turnDeg = Math.Abs(Math.Atan2(cross, dot)) * 180.0 / Math.PI;
                if (turnDeg > 30) topTurn.Add((i, turnDeg, c));
            }
            foreach (var t in topTurn.OrderByDescending(t => t.deg).Take(5))
                Console.WriteLine($"   #{t.idx} 角={t.deg:F1}° 点=({t.p.X:F2},{t.p.Y:F2})");

            // 找 simp 上转角最尖的顶点（这才是 spline 会忠实复现的尖刺来源）
            Console.WriteLine("\n=== simp 转角最大的 5 个顶点 ===");
            var topSimp = new List<(int idx, double deg, Point2d p)>();
            for (int i = 0; i < simp.Count; i++)
            {
                var a = simp[(i - 1 + simp.Count) % simp.Count];
                var c = simp[i];
                var b = simp[(i + 1) % simp.Count];
                double e1x = c.X - a.X, e1y = c.Y - a.Y;
                double e2x = b.X - c.X, e2y = b.Y - c.Y;
                double cross = e1x * e2y - e1y * e2x;
                double dot = e1x * e2x + e1y * e2y;
                double turnDeg = Math.Abs(Math.Atan2(cross, dot)) * 180.0 / Math.PI;
                topSimp.Add((i, turnDeg, c));
            }
            foreach (var t in topSimp.OrderByDescending(t => t.deg).Take(8))
                Console.WriteLine($"   #{t.idx} 角={t.deg:F1}° 点=({t.p.X:F2},{t.p.Y:F2})");

            // 在原始 envelope 中找转角最大的位置，并 dump 该位置 ±10 个顶点
            Console.WriteLine("\n=== 原始 envelope 转角最大的 #820 前后 21 顶点 ===");
            int targetIdx = 820;
            int lo = (targetIdx - 10 + envelope.Count) % envelope.Count;
            int hi = (targetIdx + 10) % envelope.Count;
            int kk = 0;
            for (int idx = lo; ; idx = (idx + 1) % envelope.Count)
            {
                double deg = 0;
                var a2 = envelope[(idx - 1 + envelope.Count) % envelope.Count];
                var c2 = envelope[idx];
                var b2 = envelope[(idx + 1) % envelope.Count];
                double e1x = c2.X - a2.X, e1y = c2.Y - a2.Y;
                double e2x = b2.X - c2.X, e2y = b2.Y - c2.Y;
                deg = Math.Abs(Math.Atan2(e1x * e2y - e1y * e2x, e1x * e2x + e1y * e2y)) * 180.0 / Math.PI;
                double distFromPrev = Math.Sqrt((c2.X - a2.X) * (c2.X - a2.X) + (c2.Y - a2.Y) * (c2.Y - a2.Y));
                string marker = idx == targetIdx ? "  ←反射" : "";
                Console.WriteLine($"   #{idx} ({c2.X:F3},{c2.Y:F3}) 转角={deg:F1}° 距前={distFromPrev * 1000:F1}mm{marker}");
                kk++;
                if (idx == hi) break;
                if (kk > 50) break;
            }

            // 检查 #820 是不是被 ChainBoundaryLoops 链环时把"前后两端"缝在一起形成的"折回"
            Console.WriteLine("\n=== envelope 是否闭合（首尾距离）===");
            var first = envelope[0];
            var last = envelope[envelope.Count - 1];
            double dLoop = Math.Sqrt((first.X - last.X) * (first.X - last.X) + (first.Y - last.Y) * (first.Y - last.Y));
            Console.WriteLine($"   首尾距 {dLoop:F6} m");

            // 检查 envelope 中有没有 > 1m 的大跳跃边
            Console.WriteLine("\n=== envelope 中 > 1m 的大跳跃边 ===");
            int bigJump = 0;
            for (int i = 0; i < envelope.Count; i++)
            {
                var a = envelope[i];
                var b = envelope[(i + 1) % envelope.Count];
                double d = Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
                if (d > 1.0)
                {
                    bigJump++;
                    if (bigJump <= 5)
                        Console.WriteLine($"   #{i}-{i + 1} 跳 {d * 1000:F1} mm   ({a.X:F2},{a.Y:F2})→({b.X:F2},{b.Y:F2})");
                }
            }
            Console.WriteLine($"   共有 {bigJump} 段 > 1m 跳跃边");
        }

        private static void DrawTruck(System.IO.StreamWriter w, Func<Point2d, (double X, double Y)> P, Frame f, VehicleParams vp, string color)
        {
            // 用 Corners(f, p) 画车体（牵引车+挂车+底盘）
            var corners = TruckKinematics.Corners(f, vp);
            w.Write($"<polygon points='");
            var pts = new[] { corners[0], corners[2], corners[3], corners[1] };
            foreach (var q in pts) { var pp = P(q); w.Write($"{pp.X:F1},{pp.Y:F1} "); }
            w.WriteLine($"' fill='none' stroke='{color}' stroke-width='1.5'/>");
            if (vp.Articulated)
            {
                w.Write($"<polygon points='");
                pts = new[] { corners[4], corners[6], corners[7], corners[5] };
                foreach (var q in pts) { var pp = P(q); w.Write($"{pp.X:F1},{pp.Y:F1} "); }
                w.WriteLine($"' fill='none' stroke='{color}' stroke-width='1.5'/>");
            }
            var chassis = TruckKinematics.ChassisRect(f, vp);
            if (chassis != null)
            {
                w.Write($"<polygon points='");
                foreach (var q in chassis) { var pp = P(q); w.Write($"{pp.X:F1},{pp.Y:F1} "); }
                w.WriteLine($"' fill='none' stroke='{color}' stroke-width='1.5' stroke-dasharray='3,2'/>");
            }
        }
    }
}
