// v4.9.9+ 调试：复现用户「S 弯（右拐→左拐→直行小段）后包络出现大幅锯齿」
// 目标：定位锯齿在 RAW envelope（并集引擎）还是 Simplify（RDP/外推）阶段引入。
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    partial class VerifyProgram
    {
        public static void _dumpSCurve()
        {
            Console.WriteLine("\n=== S 弯包络锯齿诊断（右拐→左拐→直行小段，铰接，生产步长 0.25°） ===");

            var cases = new (string label, double[] turns, double[] straights)[]
            {
                ("右90+左90+直5",  new[] { -90.0, +90.0, 0.0 }, new[] { 0.0, 0.0, 5.0 }),
                ("右90+左90+直2",  new[] { -90.0, +90.0, 0.0 }, new[] { 0.0, 0.0, 2.0 }),
                ("右90+左90",      new[] { -90.0, +90.0 },      new[] { 0.0, 0.0 }),
                ("右60+左60+直5",  new[] { -60.0, +60.0, 0.0 }, new[] { 0.0, 0.0, 5.0 }),
                ("右45+左45+直5",  new[] { -45.0, +45.0, 0.0 }, new[] { 0.0, 0.0, 5.0 }),
                ("右90+直5+左90+直5", new[] { -90.0, 0.0, +90.0, 0.0 }, new[] { 0.0, 5.0, 0.0, 5.0 }),
            };

            string csvDir = AppDomain.CurrentDomain.BaseDirectory;
            var worst = (label: "", zig: -1, frames: (List<Frame>)null, env: (List<Point2d>)null,
                         simp: (List<Point2d>)null, p: (VehicleParams)null);

            Console.WriteLine("  {0,-18} {1,7} {2,7} {3,7} {4,7} {5,7} {6,7} {7,8} {8,8} {9,6} {10,6}",
                "场景", "RAW点", "RAW齿3", "RAW齿10", "RAW齿30", "SIMP点", "S齿3", "S齿10", "S齿30", "S跳边", "S自交");
            foreach (var c in cases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = true;
                var frames = BuildMultiSegPath(p, c.turns, c.straights, TruckKinematics.FineStepDeg);
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                var (simp, outset) = GeometryUtil.SimplifyEnvelopeForSpline(env, 0.05);

                int rawZ3 = CountZigzag(env, 5, 0.03);
                int rawZ10 = CountZigzag(env, 5, 0.10);
                int rawZ30 = CountZigzag(env, 5, 0.30);
                int sZ3 = CountZigzag(simp, 5, 0.03);
                int sZ10 = CountZigzag(simp, 5, 0.10);
                int sZ30 = CountZigzag(simp, 5, 0.30);
                int jumps = CountJumpEdges(simp, 0.3);
                int selfInt = CountSelfIntersections(simp);

                Console.WriteLine("  {0,-18} {1,7} {2,7} {3,7} {4,7} {5,7} {6,7} {7,8} {8,8} {9,6} {10,6}",
                    c.label, env.Count, rawZ3, rawZ10, rawZ30, simp.Count, sZ3, sZ10, sZ30, jumps, selfInt);

                if (sZ10 > worst.zig)
                    worst = (c.label, sZ10, frames, env, simp, p);
            }

            // ---- 关键怀疑：TRUCKDRIVE 按段单独出包络。S 弯后的「直行小段」单独成段，
            //      段首挂车还带着上一段的铰接角 θ0，直行中 θ 指数衰减、挂车横向摆动，
            //      每 0.5m 一帧的离散并集在「外边界」留下台阶齿。整链包络里这些齿被
            //      上一段的扫掠体积盖住所以看不见，单段包络里就全露出来。----
            Console.WriteLine("\n=== 单段包络：直行段带初始铰接角（模拟 TRUCKDRIVE 按段出图） ===");
            Console.WriteLine("  {0,-22} {1,7} {2,8} {3,8} {4,8} {5,7} {6,7} {7,8} {8,8}",
                "场景", "RAW点", "RAW齿3", "RAW齿10", "RAW齿30", "SIMP点", "S齿3", "S齿10", "S齿30");
            var soloCases = new (string label, double th0deg, double len)[]
            {
                ("θ0=10° 直5m",  10.0, 5.0),
                ("θ0=20° 直5m",  20.0, 5.0),
                ("θ0=30° 直5m",  30.0, 5.0),
                ("θ0=40° 直5m",  40.0, 5.0),
                ("θ0=-20° 直5m", -20.0, 5.0),
                ("θ0=20° 直10m", 20.0, 10.0),
            };
            (string label, int zig, List<Frame> frames, List<Point2d> env, List<Point2d> simp, VehicleParams p) worstSolo
                = ("", -1, null, null, null, null);
            foreach (var c in soloCases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = true;
                // 构造带初始铰接角的起始帧：先让车头朝 +x，挂车航向 = +x 偏 θ0
                double th0 = c.th0deg * Math.PI / 180.0;
                var u1 = new Vector2d(1, 0);
                var u2 = new Vector2d(Math.Cos(th0), Math.Sin(th0));
                // 直接用 SimulateFromStateFront 的直行段（turnDeg=0, straight=len）
                var frames = TruckKinematics.SimulateFromStateFront(p, new Point2d(0, 0), u1,
                    Math.Atan2(u2.Y, u2.X), +1, 0.0, c.len, 1, TruckKinematics.FineStepDeg);
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                var (simp, _) = GeometryUtil.SimplifyEnvelopeForSpline(env, 0.05);
                int rz3 = CountZigzag(env, 5, 0.03), rz10 = CountZigzag(env, 5, 0.10), rz30 = CountZigzag(env, 5, 0.30);
                int sz3 = CountZigzag(simp, 5, 0.03), sz10 = CountZigzag(simp, 5, 0.10), sz30 = CountZigzag(simp, 5, 0.30);
                Console.WriteLine("  {0,-22} {1,7} {2,8} {3,8} {4,8} {5,7} {6,7} {7,8} {8,8}",
                    c.label, env.Count, rz3, rz10, rz30, simp.Count, sz3, sz10, sz30);
                if (sz10 > worstSolo.zig) worstSolo = (c.label, sz10, frames, env, simp, p);
            }
            if (worstSolo.frames != null)
            {
                Console.WriteLine($"\n=== 单段最差：{worstSolo.label} —— 锯齿峰位置 ===");
                DumpZigzagBands(worstSolo.simp, 0.05);
                string svg2 = Path.Combine(csvDir, "_scurve_solo.svg");
                WriteLayerSvg(svg2, worstSolo.label, worstSolo.frames, worstSolo.env, worstSolo.simp, worstSolo.p);
                Console.WriteLine("SVG: " + svg2);
                string csv2 = Path.Combine(csvDir, "_scurve_solo.csv");
                using (var sw = new StreamWriter(csv2, false, new UTF8Encoding(false)))
                {
                    sw.WriteLine("TYPE,IDX,X,Y");
                    for (int i = 0; i < worstSolo.env.Count; i++)
                        sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "RAW,{0},{1:F6},{2:F6}", i, worstSolo.env[i].X, worstSolo.env[i].Y));
                    for (int i = 0; i < worstSolo.simp.Count; i++)
                        sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "SIMP,{0},{1:F6},{2:F6}", i, worstSolo.simp[i].X, worstSolo.simp[i].Y));
                    for (int id = 0; id < worstSolo.frames.Count; id += 2)
                    {
                        var cc = TruckKinematics.Corners(worstSolo.frames[id], worstSolo.p);
                        for (int k = 0; k < cc.Length; k++)
                            sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "BODY,{0},{1:F6},{2:F6}", id, cc[k].X, cc[k].Y));
                    }
                }
                Console.WriteLine("CSV: " + csv2);

                // ---- 齿区归因：列出 θ0=40° 场景 RAW 环在 x∈[-7,-3], y>0.85 的顶点序列，
                //      以及前 3 帧的车体角点，确认每段边界属于哪个车体哪条边 ----
                var pA = VehicleParams.Defaults(); pA.Articulated = true;
                double thA = 40.0 * Math.PI / 180.0;
                var framesA = TruckKinematics.SimulateFromStateFront(pA, new Point2d(0, 0),
                    new Vector2d(1, 0), thA, +1, 0.0, 5.0, 1, TruckKinematics.FineStepDeg);
                var envA = TruckKinematics.EnvelopeTracks(framesA, pA);
                Console.WriteLine("\n=== 齿区 RAW 顶点（x∈[-7.5,-2.5] 且 y>0.85），按环序 ===");
                int nA = envA.Count;
                for (int i = 0; i < nA; i++)
                {
                    var v = envA[i];
                    if (v.X < -7.5 || v.X > -2.5 || v.Y < 0.85) continue;
                    var pv = envA[(i - 1 + nA) % nA]; var nx = envA[(i + 1) % nA];
                    double dPrev = Math.Sqrt((v.X - pv.X) * (v.X - pv.X) + (v.Y - pv.Y) * (v.Y - pv.Y));
                    Console.WriteLine("  #{0,4} ({1,7:F3},{2,6:F3}) 距前 {3,5:F1}mm", i, v.X, v.Y, dPrev * 1000);
                }
                Console.WriteLine("\n=== 前 4 帧车体角点（0..3 牵引车, 4..7 挂车） ===");
                for (int k = 0; k < Math.Min(4, framesA.Count); k++)
                {
                    var cc = TruckKinematics.Corners(framesA[k], pA);
                    var sb2 = new StringBuilder($"  帧{k}: 牵引车");
                    for (int j = 0; j < 4; j++) sb2.Append(string.Format(" ({0:F2},{1:F2})", cc[j].X, cc[j].Y));
                    sb2.Append(" 挂车");
                    for (int j = 4; j < 8; j++) sb2.Append(string.Format(" ({0:F2},{1:F2})", cc[j].X, cc[j].Y));
                    Console.WriteLine(sb2.ToString());
                }
            }

            // ---- v4.9.13 怀疑：转弯分支帧距=R·0.25°（不随 θ 加密），
            //      缓弯（大 R）+ 段首残留铰接角 θ0 时，挂车扇形扫掠同样产生
            //      ds·|sinθ| 级台阶齿（ds=R·dTheta 可能 ≫ 0.5m）。----
            Console.WriteLine("\n=== 单段包络：缓弯转弯段带初始铰接角（v4.9.13 目标场景） ===");
            Console.WriteLine("  {0,-26} {1,7} {2,8} {3,8} {4,8} {5,7} {6,7} {7,8} {8,8}",
                "场景", "RAW点", "RAW齿3", "RAW齿10", "RAW齿30", "SIMP点", "S齿3", "S齿10", "S齿30");
            var turnCases = new (string label, double th0deg, double r, double turnDeg, double straight)[]
            {
                ("θ0=30° R=40m 转15°",   30.0, 40.0, 15.0, 0.0),
                ("θ0=30° R=80m 转10°",   30.0, 80.0, 10.0, 0.0),
                ("θ0=40° R=25m 转20°",   40.0, 25.0, 20.0, 0.0),
                ("θ0=20° R=60m 转12°+直3", 20.0, 60.0, 12.0, 3.0),
                ("θ0=0° R=12.5m 转90°(对照)", 0.0, 12.5, 90.0, 0.0),
                ("θ0=0° R=40m 转15°(对照)",  0.0, 40.0, 15.0, 0.0),
            };
            foreach (var c in turnCases)
            {
                var p = VehicleParams.Defaults();
                p.Articulated = true;
                double th0 = c.th0deg * Math.PI / 180.0;
                var u1 = new Vector2d(1, 0);
                var frames = TruckKinematics.SimulateFromStateFront(p, new Point2d(0, 0), u1,
                    th0, +1, c.turnDeg, c.straight, 1, TruckKinematics.FineStepDeg, c.r);
                var env = TruckKinematics.EnvelopeTracks(frames, p);
                var (simp, _) = GeometryUtil.SimplifyEnvelopeForSpline(env, 0.05);
                int rz3 = CountZigzag(env, 5, 0.03), rz10 = CountZigzag(env, 5, 0.10), rz30 = CountZigzag(env, 5, 0.30);
                int sz3 = CountZigzag(simp, 5, 0.03), sz10 = CountZigzag(simp, 5, 0.10), sz30 = CountZigzag(simp, 5, 0.30);
                Console.WriteLine("  {0,-26} {1,7} {2,8} {3,8} {4,8} {5,7} {6,7} {7,8} {8,8}",
                    c.label, env.Count, rz3, rz10, rz30, simp.Count, sz3, sz10, sz30);
            }

            // ---- 最差场景：找锯齿带位置 + 出 CSV/SVG ----
            Console.WriteLine($"\n=== 最差场景：{worst.label} ===");
            DumpZigzagBands(worst.simp, 0.05);

            string csv = Path.Combine(csvDir, "_scurve_dump.csv");
            using (var sw = new StreamWriter(csv, false, new UTF8Encoding(false)))
            {
                sw.WriteLine("TYPE,IDX,X,Y");
                for (int i = 0; i < worst.env.Count; i++)
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "RAW,{0},{1:F6},{2:F6}", i, worst.env[i].X, worst.env[i].Y));
                for (int i = 0; i < worst.simp.Count; i++)
                    sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "SIMP,{0},{1:F6},{2:F6}", i, worst.simp[i].X, worst.simp[i].Y));
                for (int id = 0; id < worst.frames.Count; id += 12)
                {
                    var cc = TruckKinematics.Corners(worst.frames[id], worst.p);
                    for (int k = 0; k < cc.Length; k++)
                        sw.WriteLine(string.Format(CultureInfo.InvariantCulture, "BODY,{0},{1:F6},{2:F6}", id, cc[k].X, cc[k].Y));
                }
            }
            Console.WriteLine("CSV: " + csv);

            string svg = Path.Combine(csvDir, "_scurve_layers.svg");
            WriteLayerSvg(svg, worst.label, worst.frames, worst.env, worst.simp, worst.p);
            Console.WriteLine("SVG: " + svg);
        }

        /// <summary>数闭合环里长度超过 thr 的边（链环断裂/跳边的信号）。</summary>
        static int CountJumpEdges(List<Point2d> loop, double thr)
        {
            int cnt = 0, n = loop.Count;
            for (int i = 0; i < n; i++)
            {
                var a = loop[i]; var b = loop[(i + 1) % n];
                double d = Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
                if (d > thr) cnt++;
            }
            return cnt;
        }

        /// <summary>非相邻边的自交对数（O(n²)，simp 只有几十点，够用）。</summary>
        static int CountSelfIntersections(List<Point2d> loop)
        {
            int cnt = 0, n = loop.Count;
            for (int i = 0; i < n; i++)
            {
                var a1 = loop[i]; var a2 = loop[(i + 1) % n];
                for (int j = i + 2; j < n; j++)
                {
                    if (i == 0 && j == n - 1) continue; // 首尾边相邻
                    var b1 = loop[j]; var b2 = loop[(j + 1) % n];
                    if (SegsCross(a1, a2, b1, b2)) cnt++;
                }
            }
            return cnt;
        }

        static bool SegsCross(Point2d a1, Point2d a2, Point2d b1, Point2d b2)
        {
            double d1 = Cross(b1, b2, a1), d2 = Cross(b1, b2, a2);
            double d3 = Cross(a1, a2, b1), d4 = Cross(a1, a2, b2);
            return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                   ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
        }

        static double Cross(Point2d a, Point2d b, Point2d p)
            => (b.X - a.X) * (p.Y - a.Y) - (b.Y - a.Y) * (p.X - a.X);

        /// <summary>把锯齿（法向偏移极值超过 amp 的带）的位置打印出来，便于和截图对照。</summary>
        static void DumpZigzagBands(List<Point2d> loop, double amp)
        {
            var pts = ResampleUniform(loop, 0.05);
            int n = pts.Count, win = 5;
            if (n < 3 * win) { Console.WriteLine("  （点太少）"); return; }
            var off = new double[n];
            for (int i = 0; i < n; i++)
            {
                var a = pts[(i - win + n) % n]; var b = pts[(i + win) % n]; var c = pts[i];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double L = Math.Sqrt(dx * dx + dy * dy);
                off[i] = L < 1e-12 ? 0.0 : ((c.X - a.X) * dy - (c.Y - a.Y) * dx) / L;
            }
            int shown = 0;
            for (int i = 0; i < n && shown < 25; i++)
            {
                double pv = off[(i - 1 + n) % n], cv = off[i], qv = off[(i + 1) % n];
                if ((cv - pv) * (qv - cv) < 0 && Math.Abs(cv) > amp)
                {
                    Console.WriteLine("  锯齿峰 ({0:F2},{1:F2}) 振幅 {2:F1} mm",
                        pts[i].X, pts[i].Y, Math.Abs(cv) * 1000);
                    shown++;
                }
            }
            if (shown == 0) Console.WriteLine("  （无 >{0:F0}mm 的锯齿峰）", amp * 1000);
        }

        static void WriteLayerSvg(string path, string label, List<Frame> frames,
                                  List<Point2d> env, List<Point2d> simp, VehicleParams p)
        {
            double xMin = double.MaxValue, yMin = double.MaxValue, xMax = double.MinValue, yMax = double.MinValue;
            Action<List<Point2d>> grow = pts =>
            {
                foreach (var pt in pts)
                {
                    if (pt.X < xMin) xMin = pt.X; if (pt.X > xMax) xMax = pt.X;
                    if (pt.Y < yMin) yMin = pt.Y; if (pt.Y > yMax) yMax = pt.Y;
                }
            };
            grow(env); grow(simp);
            double pad = 2.0; xMin -= pad; yMin -= pad; xMax += pad; yMax += pad;
            int W = 1400, H = 900;
            double sc = Math.Min(W / (xMax - xMin), H / (yMax - yMin));
            Func<Point2d, string> P = pt => string.Format(CultureInfo.InvariantCulture,
                "{0:F1},{1:F1}", (pt.X - xMin) * sc, H - (pt.Y - yMin) * sc);

            var sb = new StringBuilder();
            sb.AppendLine(string.Format("<svg xmlns='http://www.w3.org/2000/svg' width='{0}' height='{1}' viewBox='0 0 {0} {1}'>", W, H));
            sb.AppendLine(string.Format("<rect width='{0}' height='{1}' fill='white'/>", W, H));
            sb.AppendLine(string.Format("<text x='10' y='22' font-size='15'>{0}  RAW={1}点(灰) SIMP={2}点(绿)</text>", label, env.Count, simp.Count));

            // RAW（灰细）
            sb.Append("<polyline points='");
            for (int i = 0; i <= env.Count; i++) sb.Append(P(env[i % env.Count]) + " ");
            sb.AppendLine("' fill='none' stroke='#999' stroke-width='0.6'/>");

            // SIMP（绿）
            sb.Append("<polyline points='");
            for (int i = 0; i <= simp.Count; i++) sb.Append(P(simp[i % simp.Count]) + " ");
            sb.AppendLine("' fill='none' stroke='#1B5E20' stroke-width='1.6'/>");
            // SIMP 顶点（红点小圆）
            foreach (var pt in simp)
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "<circle cx='{0:F1}' cy='{1:F1}' r='2.4' fill='#C62828'/>",
                    (pt.X - xMin) * sc, H - (pt.Y - yMin) * sc));

            // 车体（每 20 帧一个灰影 + 首绿末青）
            for (int k = 0; k < frames.Count; k += 20) AppendBody(sb, P, frames[k], p, "#BBBBBB", 0.8);
            AppendBody(sb, P, frames[0], p, "#2E7D32", 2.0);
            AppendBody(sb, P, frames[frames.Count - 1], p, "#00838F", 2.0);

            sb.AppendLine("</svg>");
            File.WriteAllText(path, sb.ToString(), new UTF8Encoding(false));
        }

        static void AppendBody(StringBuilder sb, Func<Point2d, string> P, Frame f, VehicleParams p, string color, double width)
        {
            var c = TruckKinematics.Corners(f, p);
            sb.Append("<polygon points='");
            foreach (var q in new[] { c[0], c[2], c[3], c[1] }) sb.Append(P(q) + " ");
            sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                "' fill='none' stroke='{0}' stroke-width='{1}'/>", color, width));
            if (p.Articulated)
            {
                sb.Append("<polygon points='");
                foreach (var q in new[] { c[4], c[6], c[7], c[5] }) sb.Append(P(q) + " ");
                sb.AppendLine(string.Format(CultureInfo.InvariantCulture,
                    "' fill='none' stroke='{0}' stroke-width='{1}'/>", color, width));
            }
        }
    }
}
