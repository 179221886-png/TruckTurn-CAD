using System;
using System.Collections.Generic;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>
    /// 几何辅助：扫掠包络用凸包(Andrew monotone chain)近似。
    /// 对“凸形体绕外部圆心旋转扫过”的运动，角点凸包是工程上常用且偏安全的包络近似。
    /// </summary>
    public static class GeometryUtil
    {
        public static List<Point2d> ConvexHull(List<Point2d> pts)
        {
            if (pts == null || pts.Count <= 2)
                return new List<Point2d>(pts ?? new List<Point2d>());

            var sorted = new List<Point2d>(pts);
            sorted.Sort((a, b) =>
            {
                int c = a.X.CompareTo(b.X);
                return c != 0 ? c : a.Y.CompareTo(b.Y);
            });

            double Cross(Point2d o, Point2d a, Point2d b)
                => (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);

            List<Point2d> Build(List<Point2d> src)
            {
                var res = new List<Point2d>();
                foreach (var p in src)
                {
                    while (res.Count >= 2 && Cross(res[res.Count - 2], res[res.Count - 1], p) <= 0)
                        res.RemoveAt(res.Count - 1);
                    res.Add(p);
                }
                return res;
            }

            var lower = Build(sorted);
            sorted.Reverse();
            var upper = Build(sorted);

            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);
            lower.AddRange(upper);
            return lower;
        }

        private static double Dist(Point2d a, Point2d b)
            => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

        /// <summary>
        /// Ramer-Douglas-Peucker 折线简化：把偏离直线小于 tol 的中间点全部删掉。
        /// 输入/输出都不闭合；不处理首尾。
        /// 返回 (vertices, segments)：segments[i] = 起点索引，segments[i+1] - 1 = 终点索引（含）。
        /// 用于把"采样密集但实际是直线"的折线段压成 2 个端点。
        /// </summary>
        public static List<Point2d> SimplifyPolyline(List<Point2d> pts, double tol = 1e-3)
        {
            var keep = new bool[pts.Count];
            keep[0] = true;
            keep[pts.Count - 1] = true;
            Rdp(pts, 0, pts.Count - 1, tol, keep);
            var res = new List<Point2d>();
            for (int i = 0; i < pts.Count; i++) if (keep[i]) res.Add(pts[i]);
            return res;
        }

        private static void Rdp(List<Point2d> pts, int lo, int hi, double tol, bool[] keep)
        {
            if (hi <= lo + 1) return;
            var a = pts[lo]; var b = pts[hi];
            double vx = b.X - a.X, vy = b.Y - a.Y;
            double len2 = vx * vx + vy * vy;
            int worstIdx = -1; double worst = 0.0;
            for (int k = lo + 1; k < hi; k++)
            {
                var p = pts[k];
                double d;
                if (len2 < 1e-12)
                    d = Math.Sqrt((p.X - a.X) * (p.X - a.X) + (p.Y - a.Y) * (p.Y - a.Y));
                else
                {
                    double t = ((p.X - a.X) * vx + (p.Y - a.Y) * vy) / len2;
                    if (t < 0) t = 0; else if (t > 1) t = 1;
                    double dx = p.X - (a.X + t * vx);
                    double dy = p.Y - (a.Y + t * vy);
                    d = Math.Sqrt(dx * dx + dy * dy);
                }
                if (d > worst) { worst = d; worstIdx = k; }
            }
            if (worstIdx > 0 && worst > tol)
            {
                keep[worstIdx] = true;
                Rdp(pts, lo, worstIdx, tol, keep);
                Rdp(pts, worstIdx, hi, tol, keep);
            }
        }

        #region v4.8：包络环稀化 + 外包补偿（配合 Spline 出图根治锯齿）

        /// <summary>
        /// 闭合环的 Ramer-Douglas-Peucker 稀化。
        /// 开放折线的 RDP 会无条件保留首尾两点，直接套在闭合环上会在接缝处留下本该删掉的点，
        /// 所以先挑「离质心最远」的顶点当起点（真实极值点，RDP 必然保留），
        /// 把环展开成 [start … start] 的开放折线再简化，最后去掉重复的闭合尾点。
        /// tol 单位为米 = 允许偏离原始环的最大距离。
        /// </summary>
        public static List<Point2d> SimplifyClosedLoop(List<Point2d> loop, double tol)
        {
            if (loop == null || loop.Count < 4)
                return new List<Point2d>(loop ?? new List<Point2d>());
            if (tol <= 0) return new List<Point2d>(loop);

            double cx = 0, cy = 0;
            foreach (var p in loop) { cx += p.X; cy += p.Y; }
            cx /= loop.Count; cy /= loop.Count;

            int start = 0; double best = -1;
            for (int i = 0; i < loop.Count; i++)
            {
                double dx = loop[i].X - cx, dy = loop[i].Y - cy;
                double d2 = dx * dx + dy * dy;
                if (d2 > best) { best = d2; start = i; }
            }

            var open = new List<Point2d>(loop.Count + 1);
            for (int i = 0; i <= loop.Count; i++)
                open.Add(loop[(start + i) % loop.Count]);

            var simp = SimplifyPolyline(open, tol);
            if (simp.Count >= 2 && Dist(simp[0], simp[simp.Count - 1]) < 1e-9)
                simp.RemoveAt(simp.Count - 1);
            return simp;
        }

        /// <summary>射线法判断点是否在闭合多边形内部（边界上算内部）。绕向无关。</summary>
        public static bool PointInClosedPolygon(Point2d p, List<Point2d> poly)
        {
            if (poly == null || poly.Count < 3) return false;
            bool inside = false;
            int n = poly.Count;
            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var pi = poly[i]; var pj = poly[j];
                if ((pi.Y > p.Y) != (pj.Y > p.Y))
                {
                    double x = pi.X + (p.Y - pi.Y) * (pj.X - pi.X) / (pj.Y - pi.Y);
                    if (p.X < x) inside = !inside;
                }
            }
            return inside;
        }

        /// <summary>
        /// 把闭合环沿顶点外法向整体外推 dist（CCW 环的外法向 = 边方向右转 90°，即 (e.Y, -e.X)/|e|）。
        /// 顶点法向取相邻两边外法向的归一化平均（角平分线）；
        /// 角平分线方向上走 dist 只能让两条边各自外移 dist·cos(半角)，
        /// 所以这里再除以 cos(半角) 补偿，保证「边」至少外移 dist，而不只是「顶点」外移。
        /// </summary>
        public static List<Point2d> OffsetClosedLoop(List<Point2d> loop, double dist)
        {
            int n = loop.Count;
            var res = new List<Point2d>(n);
            for (int i = 0; i < n; i++)
            {
                var prev = loop[(i - 1 + n) % n];
                var cur = loop[i];
                var next = loop[(i + 1) % n];

                double n1x = 0, n1y = 0, n2x = 0, n2y = 0;
                double e1x = cur.X - prev.X, e1y = cur.Y - prev.Y;
                double l1 = Math.Sqrt(e1x * e1x + e1y * e1y);
                if (l1 > 1e-12) { n1x = e1y / l1; n1y = -e1x / l1; }
                double e2x = next.X - cur.X, e2y = next.Y - cur.Y;
                double l2 = Math.Sqrt(e2x * e2x + e2y * e2y);
                if (l2 > 1e-12) { n2x = e2y / l2; n2y = -e2x / l2; }

                double nx = n1x + n2x, ny = n1y + n2y;
                double ln = Math.Sqrt(nx * nx + ny * ny);
                if (ln > 1e-12) { nx /= ln; ny /= ln; }
                else { nx = n1x; ny = n1y; }   // 退化（180° 折返 / 尖刺）：退化为单边外法向

                // cosHalf = 角平分线与单边法向夹角余弦；尖角时变小 → 需要走更远才能让边外移 dist。
                double cosHalf = nx * n1x + ny * n1y;
                if (cosHalf < 0.2) cosHalf = 0.2;   // 上限 5×dist，避免极端尖角把顶点甩出去
                double scale = dist / cosHalf;

                res.Add(new Point2d(cur.X + nx * scale, cur.Y + ny * scale));
            }
            return res;
        }

        /// <summary>
        /// 保角拉普拉斯平滑：抹掉离散 footprint 并集留下的台阶伪影，同时保住真实的车体折角。
        ///
        /// ★ 为什么不能直接做 RDP 稀化（实测教训）：
        ///   RDP 保留的是「偏离直线最远的点」，而台阶尖峰恰恰就是偏离最大者 ——
        ///   直接 RDP 会把尖峰一个个全留下，只删掉中间的过渡点，尖峰反而更孤立突出。
        ///   实测包络 1178 点 → 657 点，锯齿数却从 305 涨到 415。稀化对去锯齿无效甚至有害。
        ///
        /// ★ 为什么不能无差别平滑：
        ///   车体角点扫掠出来的外边界本来就是 90° 尖角，磨圆 = 包络内缩 = 安全事故。
        ///
        /// ★ 为什么不能用「转角大小」区分台阶和真实折角：
        ///   台阶齿顶的转角同样是 ~90°，和真实车体角点一样大，区分不开。
        ///   真正的区别在**尺度**：台阶是相邻两帧侧面直线相交形成的，齿的边长 = 采样间距（2~8 cm）；
        ///   真实角点两侧至少有一条长边（几十厘米以上）。
        ///   所以锚点判据 = 转角大 **且** 至少一侧边长 &gt; minAnchorEdge。
        ///
        /// 平滑本身有收缩性（拉普拉斯平滑的固有性质），收缩量 ≈ 台阶振幅，
        /// 由 SimplifyEnvelopeForSpline 最后的外推补偿统一补回来，保证包络只外扩不内缩。
        /// </summary>
        public static List<Point2d> SmoothEnvelopeLoop(List<Point2d> loop,
                                                       double anchorTurnDeg = 60.0,
                                                       double minAnchorEdge = 0.15,
                                                       int iters = 60,
                                                       double lambda = 0.2)
        {
            int n = loop == null ? 0 : (loop.Count);
            if (n < 5 || iters <= 0) return new List<Point2d>(loop ?? new List<Point2d>());

            // ---- 1. 标记锚点：大转角 + 至少一侧长边 ----
            var anchor = new bool[n];
            for (int i = 0; i < n; i++)
            {
                var prev = loop[(i - 1 + n) % n];
                var cur = loop[i];
                var next = loop[(i + 1) % n];

                double e1x = cur.X - prev.X, e1y = cur.Y - prev.Y;
                double e2x = next.X - cur.X, e2y = next.Y - cur.Y;
                double l1 = Math.Sqrt(e1x * e1x + e1y * e1y);
                double l2 = Math.Sqrt(e2x * e2x + e2y * e2y);

                double cross = e1x * e2y - e1y * e2x;
                double dot = e1x * e2x + e1y * e2y;
                double turnDeg = Math.Abs(Math.Atan2(cross, dot)) * 180.0 / Math.PI;

                anchor[i] = turnDeg > anchorTurnDeg && (l1 > minAnchorEdge || l2 > minAnchorEdge);
            }

            // ---- 2. 迭代拉普拉斯平滑（同步双缓冲更新，锚点原地不动）----
            var curPts = new List<Point2d>(loop);
            var newPts = new List<Point2d>(loop);
            for (int it = 0; it < iters; it++)
            {
                for (int i = 0; i < n; i++)
                {
                    if (anchor[i]) { newPts[i] = curPts[i]; continue; }
                    var a = curPts[(i - 1 + n) % n];
                    var c = curPts[i];
                    var b = curPts[(i + 1) % n];
                    newPts[i] = new Point2d(
                        c.X + lambda * ((a.X + b.X) * 0.5 - c.X),
                        c.Y + lambda * ((a.Y + b.Y) * 0.5 - c.Y));
                }
                for (int i = 0; i < n; i++) curPts[i] = newPts[i];
            }
            return curPts;
        }

        /// <summary>
        /// 为 Spline 出图准备包络环：大容差 RDP → 迭代外推补偿。
        /// 两步是 v4.9.6 实测后定下的**正确顺序**（之前的版本都不对）：
        ///
        ///   ① 大容差 RDP(0.05m = 5cm) —— RAW envelope 1179 顶点里 812 个（69%）是
        ///      车体折角被离散化反射形成的「短边 + 90°」伪尖角。
        ///      每帧车体角点位置几乎重合（直行段 0.1mm）但「最近角点」在帧间切换，
        ///      留下 0.1mm/80mm 交替边 + 90° 转角。这种顶点整段**近似共线**（步长 0.075m，
        ///      每点偏离首尾连线 < 1mm），RDP 5cm 容差能直接把整段压成 2 个端点。
        ///      实测：1179 → 35 顶点，AABB 与 RAW 几乎一致（差 0~3mm），无自交，
        ///      max 转角 90.0°（车体真实折角保留），>30° 顶点 5 个（全是车体折角）。
        ///
        ///   ② 迭代外推补偿 —— RDP 会内缩 ~5cm，外推补偿保证结果环外包 RAW envelope
        ///      （RAW 每个顶点在简化环内或环上）。
        ///
        /// ★ 不再跑 Laplacian 平滑（v4.9.6 第二版尝试）：
        ///   35 顶点上跑 Laplacian 反而把真实车体折角磨圆（锚点判据 l1/l2>0.15m 在
        ///   RDP 后部分车体折角只剩 5~10cm 边长，不再被识别为锚点），输出环内缩 ~50cm，
        ///   外推补偿要补 2.9m 才能包住原始 envelope —— 反而画蛇添足。
        ///
        ///   v4.8/v4.9.2 的「先 Laplacian 再 RDP」方案也是错的：Laplacian 不删顶点
        ///   只拉伸短边，把 1179 顶点的反射伪尖角变成均匀间距的小凸起（用户看到的
        ///   「规律性波浪」）。
        ///
        /// 返回 (处理后的环, 实际外推总量米)。
        /// </summary>
        public static (List<Point2d> loop, double outset) SimplifyEnvelopeForSpline(
            List<Point2d> envelope, double tol, int maxIter = 6, int smoothIters = 40)
        {
            // ① 大容差 RDP：直接砍到几十顶点（实测 5cm 把 1179 砍到 35）
            const double rdpTol = 0.05;
            var simp = SimplifyClosedLoop(envelope, rdpTol);
            if (simp.Count < 3) return (new List<Point2d>(envelope), 0.0);

            // ② 迭代外推补偿（不再做 Laplacian —— 35 顶点不需要平滑，平滑反而磨圆车体折角）
            double total = 0.0;
            var cur = simp;
            for (int it = 0; it < maxIter; it++)
            {
                double maxOut = 0.0;
                foreach (var p in envelope)
                {
                    if (PointInClosedPolygon(p, cur)) continue;
                    var (d, _, _) = DistancePointToPolyline(p, cur);
                    if (d > maxOut) maxOut = d;
                }
                if (maxOut <= 1e-9) break;
                cur = OffsetClosedLoop(cur, maxOut);
                total += maxOut;
            }
            return (cur, total);
        }

        #endregion

        /// <summary>点 P 到线段 AB 的最短距离及最近点</summary>
        public static (double dist, Point2d closest) DistancePointToSegment(Point2d p, Point2d a, Point2d b)
        {
            double vx = b.X - a.X;
            double vy = b.Y - a.Y;
            double wx = p.X - a.X;
            double wy = p.Y - a.Y;
            double c1 = vx * wx + vy * wy;
            if (c1 <= 0) return (Dist(p, a), a);
            double c2 = vx * vx + vy * vy;
            if (c2 <= c1) return (Dist(p, b), b);
            double t = c1 / c2;
            var closest = new Point2d(a.X + t * vx, a.Y + t * vy);
            return (Dist(p, closest), closest);
        }

        /// <summary>点 P 到多段线 poly 所有线段的最近距离和最近点（poly 顶点按顺序）</summary>
        public static (double dist, Point2d onPoly, int segmentIndex) DistancePointToPolyline(Point2d p, List<Point2d> poly)
        {
            if (poly == null || poly.Count == 0) return (double.MaxValue, p, -1);
            if (poly.Count == 1) return (Dist(p, poly[0]), poly[0], 0);

            double best = double.MaxValue;
            Point2d bestPt = poly[0];
            int bestIdx = 0;
            int n = poly.Count;
            for (int i = 0; i < n; i++)
            {
                Point2d a = poly[i];
                Point2d b = poly[(i + 1) % n]; // 闭合多段线
                var (d, cp) = DistancePointToSegment(p, a, b);
                if (d < best) { best = d; bestPt = cp; bestIdx = i; }
            }
            return (best, bestPt, bestIdx);
        }

        #region 包络圆弧拟合：把并集边界折线转成带 bulge 的光滑多段线

        /// <summary>
        /// 最小二乘圆拟合（Kasa 线性化）。返回 (成功, 圆心, 半径, 最大残差)。
        /// 至少 3 个点；点越共线/共点，返回失败。
        /// </summary>
        public static (bool ok, Point2d center, double radius, double maxErr) FitCircle(List<Point2d> pts)
        {
            int n = pts.Count;
            if (n < 3) return (false, default, 0, 0);

            double mx = 0, my = 0;
            foreach (var p in pts) { mx += p.X; my += p.Y; }
            mx /= n; my /= n;

            double Suu = 0, Suv = 0, Svv = 0, Suuu = 0, Suvv = 0, Svuu = 0, Svvv = 0;
            foreach (var p in pts)
            {
                double u = p.X - mx, v = p.Y - my;
                double u2 = u * u, v2 = v * v;
                Suu += u2; Suv += u * v; Svv += v2;
                Suuu += u2 * u; Suvv += u * v2; Svuu += v * u2; Svvv += v2 * v;
            }

            double det = Suu * Svv - Suv * Suv;
            if (Math.Abs(det) < 1e-18) return (false, default, 0, 0);

            double a = (Svv * (Suuu + Suvv) - Suv * (Svvv + Svuu)) / (2.0 * det);
            double b = (Suu * (Svvv + Svuu) - Suv * (Suuu + Suvv)) / (2.0 * det);
            double cx = mx + a, cy = my + b;
            double r = Math.Sqrt(a * a + b * b + (Suu + Svv) / n);
            if (r < 1e-9) return (false, default, 0, 0);

            double maxErr = 0;
            foreach (var p in pts)
            {
                double d = Math.Abs(Math.Sqrt((p.X - cx) * (p.X - cx) + (p.Y - cy) * (p.Y - cy)) - r);
                if (d > maxErr) maxErr = d;
            }
            return (true, new Point2d(cx, cy), r, maxErr);
        }

        /// <summary>
        /// 把带 bulge 的（可选闭合）顶点列按折线采样，圆弧段细分为 samplesPerArc 段。
        /// 用于没有 bulge 绘制能力、只能画直线的场合（例如 Jig 实时预览的 WorldLine）。
        /// bulges[i] 描述 pts[i] → pts[(i+1)%n] 这一段。
        /// </summary>
        public static List<Point2d> SampleBulgePolyline(List<Point2d> pts, List<double> bulges, int samplesPerArc = 24)
        {
            var res = new List<Point2d>();
            int n = pts == null ? 0 : pts.Count;
            if (n < 2 || samplesPerArc < 1) return res;
            for (int i = 0; i < n; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % n];
                double bv = (bulges != null && i < bulges.Count) ? bulges[i] : 0.0;
                res.Add(a);
                if (Math.Abs(bv) < 1e-12) continue;

                double theta = 4.0 * Math.Atan(bv);
                var d = b - a;
                double chord = Math.Sqrt(d.X * d.X + d.Y * d.Y);
                double den = 2.0 * Math.Sin(theta / 2.0);
                if (Math.Abs(den) < 1e-12 || chord < 1e-12) continue;
                double radius = Math.Abs(chord / den);
                if (radius > 1e6 || double.IsNaN(radius) || double.IsInfinity(radius)) continue;

                // 正 bulge：弧向弦左侧凸出，圆心在左侧
                var mid = new Point2d((a.X + b.X) * 0.5, (a.Y + b.Y) * 0.5);
                var u = new Vector2d(d.X / chord, d.Y / chord);
                var left = new Vector2d(-u.Y, u.X);
                double h = radius * Math.Cos(theta / 2.0);
                var center = new Point2d(mid.X + Math.Sign(bv) * h * left.X,
                                         mid.Y + Math.Sign(bv) * h * left.Y);
                double a0 = Math.Atan2(a.Y - center.Y, a.X - center.X);
                for (int s = 1; s < samplesPerArc; s++)
                {
                    double ang = a0 + theta * (s / (double)samplesPerArc);
                    res.Add(new Point2d(center.X + radius * Math.Cos(ang),
                                        center.Y + radius * Math.Sin(ang)));
                }
            }
            return res;
        }

        /// <summary>闭合环的有向面积（CCW 为正）。用于判定多段线的绕向。</summary>
        private static double SignedArea(List<Point2d> poly)
        {
            double s = 0.0;
            int n = poly == null ? 0 : poly.Count;
            for (int i = 0; i < n; i++)
            {
                var a = poly[i];
                var b = poly[(i + 1) % n];
                s += a.X * b.Y - b.X * a.Y;
            }
            return s * 0.5;
        }

        /// <summary>
        /// 把点 p 沿「以 center 为心的径向」外推/内移到指定半径 r 上（方向不变）。
        /// 用于保证圆弧拟合结果落在原始边界的非材料侧。p 与 center 几乎重合时原样返回。
        /// </summary>
        private static Point2d PushRadial(Point2d p, Point2d center, double r)
        {
            Vector2d v = p - center;
            double len = Math.Sqrt(v.X * v.X + v.Y * v.Y);
            if (len < 1e-9) return p;
            double s = r / len;
            return new Point2d(center.X + v.X * s, center.Y + v.Y * s);
        }

        /// <summary>由起点 A、终点 B、圆心 C 计算 AutoCAD 多段线 bulge = tan(θ/4)，θ 为有向圆心角。</summary>
        public static double BulgeFromArc(Point2d a, Point2d b, Point2d center)
        {
            Vector2d va = a - center;
            Vector2d vb = b - center;
            double theta = Math.Atan2(va.X * vb.Y - va.Y * vb.X, va.X * vb.X + va.Y * vb.Y);
            return Math.Tan(theta / 4.0);
        }

        /// <summary>
        /// 闭合环上最大的顶点折角（度），取值 0~180。
        ///
        /// 用于给 bulge 拟合结果兜底：拟合的目的是「变光滑」，绝不允许反过来
        /// 造出比原始并集边界更尖的角。实测通用圆拟合在铰接车纯 90° 圆弧上会
        /// 用一个只有 64 顶点的结果换出一个 128° 的伪尖刺（原始边界才 118.67°），
        /// 顶点数更少却是错的 —— 所以选拟合结果时必须同时看这个量。
        ///
        /// 注意：必须用 v1=cur−prev（指向顶点）、v2=next−cur（离开顶点）再
        /// atan2(cross,dot)。若写成 prev−cur 与 next−cur（都背离顶点），量到的是
        /// 内角，直线处是 180°，指标方向正好相反。
        /// 也不能用 asin(cross/(|a||b|))：值域上限恒 90°，会把 120° 折回 60°。
        /// </summary>
        public static double MaxTurnAngleDeg(List<Point2d> poly)
        {
            int n = poly == null ? 0 : poly.Count;
            if (n < 3) return 0.0;
            double maxDeg = 0.0;
            for (int i = 0; i < n; i++)
            {
                var prev = poly[(i - 1 + n) % n];
                var cur = poly[i];
                var next = poly[(i + 1) % n];
                double ax = cur.X - prev.X, ay = cur.Y - prev.Y;
                double bx = next.X - cur.X, by = next.Y - cur.Y;
                double la = Math.Sqrt(ax * ax + ay * ay);
                double lb = Math.Sqrt(bx * bx + by * by);
                if (la < 1e-12 || lb < 1e-12) continue;
                double cross = ax * by - ay * bx;
                double dot = ax * bx + ay * by;
                double deg = Math.Abs(Math.Atan2(cross, dot)) * 180.0 / Math.PI;
                if (deg > maxDeg) maxDeg = deg;
            }
            return maxDeg;
        }

        /// <summary>
        /// 包络出图前的 bulge 圆弧拟合入口：先用已知瞬时中心拟合，必要时再用通用
        /// 圆拟合补一次，取两者中「顶点更少且没有引入伪尖刺」的那个。
        ///
        /// 为什么要两个拟合器各跑一遍：
        ///   - FitArcsAboutCenters 认得转弯中心，纯圆弧路径上最准（刚性 90°：
        ///     1178 → 9 顶点 / 3 弧）；但它只能消化「绕已知瞬心」的部分，
        ///     转弯+直行的混合路径（TRUCKTURN90 拉直段）上会剩下几百个台阶顶点。
        ///   - FitBulgePolyline 是通用 Kasa 圆拟合，没有中心盲区，同一条混合路径
        ///     能把 851 顶点压到 67 顶点、车体直角从 656 降到 32；
        ///     但它在纯圆弧上会为了少几个顶点而造出 128° 的伪尖刺
        ///     （原始边界才 118.67°，实测）。
        ///
        /// 因此判据不能只看顶点数（那样纯圆弧会被伪尖刺结果顶掉），必须加上：
        ///   拟合后的最大折角 ≤ 原始边界的最大折角 + maxTurnSlackDeg
        /// 拟合的目的是变光滑，绝不允许反过来更尖。
        /// 两个拟合器都由构造保证不内缩（外凸段取 rMax、内凹段取 rMin），
        /// 所以不需要再单独判覆盖性。
        ///
/// maxTurnSlackDeg 默认值 8.0 的来源（三组实测，余量各约 1.4°）：
///   - 正常的通用拟合（TRUCKTURN90 转弯+拉直混合路径）：
///     原始 90.42° → 拟合 94.48°，只涨 4.06°，且 839 → 85 顶点，应当采纳。
///     这里涨的角度来自弧与直线的接缝，是拟合残差而非尖刺。
///   - 伪尖刺（铰接车纯 90° 圆弧）：
///     原始 118.67° → 拟合 128.15°，涨 9.48°，必须拦住。
///   - v4.5.2 新增：铰接车多段急转（左右 45° + 5m 直行 × 3 段）：
///     原始 108.89° → 拟合 115.48°，涨 6.59°。这是真实的"底盘-挂车接缝"折角
///     （90° + 累积铰接角），原本 6° 偏紧被误拒导致 965 顶点折线；放宽到 8° 后
///     用 215 顶点的合法拟合盖过去，仍留 1.4° 余量拦截更糟的伪尖刺。
/// 曾取 2.0，结果把上面那条正常拟合一起拦掉，混合路径留了 839 个顶点。
/// 曾取 6.0，case 5「真实接缝折角」会被拦掉，铰接车多段急转场景仍是 965 顶点折线。
///
/// tryGenericAboveVerts：只有多中心拟合没能压下去（顶点数超过这个阈值，
/// 说明残留了台阶）时才去跑代价更高的通用拟合，纯圆弧路径因此零额外开销。
/// </summary>
        public static void FitEnvelopeBulges(List<Point2d> envelope, IList<Point2d> centers,
            out List<Point2d> pts, out List<double> bulges,
            double tol = 3e-2, double minArcAngleDeg = 4.0, int minArcPts = 4,
            double maxPush = 2e-2, double maxSag = 2e-2,
            int tryGenericAboveVerts = 150, double maxTurnSlackDeg = 8.0,
            double straightCollapseTol = 1e-4)
        {
            pts = null;
            bulges = null;
            if (envelope == null || envelope.Count < 3) return;

            double rawMaxTurn = MaxTurnAngleDeg(envelope);

            if (centers != null && centers.Count > 0)
            {
                pts = FitArcsAboutCenters(envelope, centers, out bulges,
                    tol, minArcAngleDeg, minArcPts, 1500, maxPush, maxSag);
            }
            else
            {
                pts = FitBulgePolyline(envelope, out bulges,
                    tol, minArcAngleDeg, minArcPts, 300, maxPush, maxSag);
                return;
            }

            if (pts == null || pts.Count <= tryGenericAboveVerts)
            {
                if (straightCollapseTol > 1e-4)
                {
                    var reduced = CollapseStraightRuns(pts, bulges, straightCollapseTol);
                    pts = reduced.pts;
                    bulges = reduced.bulges;
                }
                return;
            }

            List<double> bulges2;
            var alt = FitBulgePolyline(envelope, out bulges2,
                tol, minArcAngleDeg, minArcPts, 300, maxPush, maxSag);
            if (alt == null || bulges2 == null) return;
            if (alt.Count >= pts.Count) return;                       // 没更简洁，不换
            if (bulges2.Count == 0) return;
            if (MaxTurnAngleDeg(alt) > rawMaxTurn + maxTurnSlackDeg) return; // 引入伪尖刺，不换
            pts = alt;
            bulges = bulges2;

            if (straightCollapseTol > 1e-4)
            {
                var reduced = CollapseStraightRuns(pts, bulges, straightCollapseTol);
                pts = reduced.pts;
                bulges = reduced.bulges;
            }
        }

        /// <summary>
        /// 把闭合多段线 pts 的近似圆弧段识别出来，转换为带 bulge 的顶点列。
        /// 输出与输入同向（假设为 CCW）；直线段 bulge=0，圆弧段 bulge≠0。
        /// tol：拟合最大残差容差（米）；minArcAngleDeg：圆弧最小圆心角；
        /// minArcPts：单弧最少连续点数，防止把任意 3 点都拟合成小圆；
        /// maxArcPts：单弧最大连续点数，防止过度合并把直角也吞掉。
        /// </summary>
        /// 与 FitArcsAboutCenters 同样取 5e-3（详见那里的容差说明）。
        /// maxArcPts 保持 300：这里的 Kasa 圆拟合是 O(len)，总代价 O(n·maxArcPts²)，
        /// 再大就要秒级了。已知圆心的 FitArcsAboutCenters 是 O(1) 增量，才可以开到 1500。
        public static List<Point2d> FitBulgePolyline(List<Point2d> pts, out List<double> bulges,
            double tol = 3e-2, double minArcAngleDeg = 4.0, int minArcPts = 4, int maxArcPts = 300,
            double maxPush = 2e-2, double maxSag = 2e-2)
        {
            bulges = new List<double>();
            if (pts == null || pts.Count < 3)
            {
                for (int k = 0; pts != null && k < pts.Count; k++) bulges.Add(0);
                return pts != null ? new List<Point2d>(pts) : new List<Point2d>();
            }

            int n = pts.Count;
            var outPts = new List<Point2d>();
            var outBulges = new List<double>();

            // 环的绕向：决定弧半径取 rMax（外凸段）还是 rMin（内凹段），
            // 判据与 FitArcsAboutCenters 完全一致。
            bool ringCCW = SignedArea(pts) > 0;

            int i = 0;
            while (i < n)
            {
                // 滑动窗口：从 i 开始向后扩展，取「能拟合成同一圆」的最长子序列。
                // 注意必须取最长、而不是残差最小的那段：后者会把一整条 90° 弧切成
                // 十几段 5°~15° 的碎片弧，出图呈扇贝状锯齿（v4.4 的黄色包络锯齿根因）。
                int bestEnd = -1;
                Point2d bestCenter = default;
                double bestRArc = 0, bestSweep = 0;

                int jLimit = Math.Min(n, i + maxArcPts);
                for (int j = i + minArcPts; j <= jLimit; j++)
                {
                    var sub = new List<Point2d>(j - i);
                    for (int k = i; k < j; k++) sub.Add(pts[k]);
                    var fit = FitCircle(sub);
                    if (!fit.ok) continue;

                    // 限制半径：极大半径的"弧"实质是直线
                    if (fit.radius > 1e6 || fit.radius < 1e-3) continue;

                    // 残差容差：绝对值 + 相对半径比值
                    if (fit.maxErr > Math.Max(tol, fit.radius * 1e-6)) continue;

                    // 极角必须沿同一方向单调推进。
                    // 这是必须的：包络边界会在同一条圆弧上来回切换车体角点，
                    // 若只校验「圆拟合残差」而不看角度方向，拟合弧会用首尾点「抄近路」，
                    // 把真实弯曲切掉、向外鼓出（实测可让包络面积虚增 17%）。
                    double prev = 0, sweep = 0;
                    int dir = 0;
                    bool mono = true;
                    for (int k = 0; k < sub.Count; k++)
                    {
                        var d = sub[k] - fit.center;
                        double rk = Math.Sqrt(d.X * d.X + d.Y * d.Y);
                        double ang = Math.Atan2(d.Y, d.X);
                        if (k > 0)
                        {
                            double da = ang - prev;
                            while (da > Math.PI) da -= 2.0 * Math.PI;
                            while (da <= -Math.PI) da += 2.0 * Math.PI;
                            if (k == 1)
                            {
                                if (Math.Abs(da) < 1e-14) { mono = false; break; }
                                dir = da > 0 ? 1 : -1;
                            }
                            else if (dir * da < -1e-12) { mono = false; break; }
                            // 相邻点角步约束：与 FitArcsAboutCenters 同一条。
                            // 缺了它，车体初始/终止姿态的长直边（只有首尾 2 个顶点）会被弧
                            // "抄近路"，实测把包络向外顶出 0.635 m。
                            double argC = 1.0 - maxSag / Math.Max(rk, 1e-6);
                            if (argC < -1.0) argC = -1.0; else if (argC > 1.0) argC = 1.0;
                            if (Math.Abs(da) > 2.0 * Math.Acos(argC)) { mono = false; break; }
                            sweep += da;
                        }
                        prev = ang;
                    }
                    if (!mono) continue;

                    // 圆心角必须足够大。
                    // 注意：不可写成 `if (sweep < 0) sweep += 2π` 再判 sweepDeg —— 那会把
                    // -2.8° 的顺时针短弧折成 357.2°，绕过最小圆心角检查，
                    // 让大量毫无光滑价值却会引入厘米级错位的碎片弧混进结果。
                    double sweepDeg = Math.Abs(sweep) * 180.0 / Math.PI;
                    if (sweepDeg < minArcAngleDeg) continue;
                    if (sweepDeg > 180.0) continue; // 优弧无法用单段 bulge 可靠表达

                    // 窗口内相对拟合圆心的最小/最大半径。
                    double rArc = 0, rMin = double.MaxValue;
                    for (int k = 0; k < sub.Count; k++)
                    {
                        var d = sub[k] - fit.center;
                        double rr = Math.Sqrt(d.X * d.X + d.Y * d.Y);
                        if (rr > rArc) rArc = rr;
                        if (rr < rMin) rMin = rr;
                    }

                    // ====== 圆心角直接取两端点的真实极角差（v4.5 关键修正）======
                    // 出图的弧由「两端点 + bulge」唯一确定，实际半径 = 弦长/(2·sin(θ/2))。
                    // 只有 θ 等于两端点绕圆心的真实极角差时，画出来的弧才正好是以 fit.center
                    // 为心、rArc 为半径的那段圆；此时"弧上每点半径 = rArc ≥ 窗口内所有点半径"，
                    // 覆盖性由构造保证，与 tol 取多大无关。
                    //
                    // 不能用累积 sweep 当 θ：包络边界会在不同车体角点之间切换，切换瞬间极角
                    // 会跳一大步，sweep 把跳变也算进去 → θ 偏大 → 反算出的弧半径偏小 →
                    // 弧整体向内收缩，把车体角点漏在包络外面。实测某段 sweep=28.84° 而真实
                    // 极角差小得多，漏覆盖达 0.62 m（这正是 v4.5 开发中期铰接车 FAIL 的根因）。
                    //
                    // 极角差必须先归一到 (-π, π]，再按旋向 dir 定向还原，
                    // 否则 |θ|>180° 的优弧会被折成反方向的劣弧。
                    var da0 = sub[0] - fit.center;
                    var db0 = sub[sub.Count - 1] - fit.center;
                    double angA = Math.Atan2(da0.Y, da0.X);
                    double angB = Math.Atan2(db0.Y, db0.X);
                    double dAng = angB - angA;
                    while (dAng > Math.PI) dAng -= 2.0 * Math.PI;
                    while (dAng <= -Math.PI) dAng += 2.0 * Math.PI;
                    if (dir > 0 && dAng < 0) dAng += 2.0 * Math.PI;
                    else if (dir < 0 && dAng > 0) dAng -= 2.0 * Math.PI;
                    if (Math.Abs(dAng) * 180.0 / Math.PI < minArcAngleDeg) continue;
                    if (Math.Abs(dAng) * 180.0 / Math.PI > 179.0) continue;

                    // 与 FitArcsAboutCenters 同一条判据：外凸段取 rMax（外扩），
                    // 内凹段（圆心在空腔侧，如挂车 off-tracking 内侧）取 rMin。
                    double rSel = ((dAng > 0) == ringCCW) ? rArc : rMin;

                    // 端点位移约束：与 FitArcsAboutCenters 同一条，防止弧端点被径向推远后
                    // 与相邻折线之间被拉出长直线、把包络局部顶出去（实测虚胖 0.635 m）。
                    var e0 = sub[0] - fit.center;
                    var e1 = sub[sub.Count - 1] - fit.center;
                    double rEnd0 = Math.Sqrt(e0.X * e0.X + e0.Y * e0.Y);
                    double rEnd1 = Math.Sqrt(e1.X * e1.X + e1.Y * e1.Y);
                    if (Math.Abs(rEnd0 - rSel) > maxPush) continue;
                    if (Math.Abs(rEnd1 - rSel) > maxPush) continue;

                    bestEnd = j - 1;
                    bestCenter = fit.center;
                    bestRArc = rSel;
                    bestSweep = dAng;
                }

                if (bestEnd > i)
                {
                    Point2d a = PushRadial(pts[i], bestCenter, bestRArc);
                    Point2d b = PushRadial(pts[bestEnd], bestCenter, bestRArc);
                    double bulge = Math.Tan(bestSweep / 4.0);
                    if (Math.Abs(bulge) > 1e-6 && !double.IsNaN(bulge) && !double.IsInfinity(bulge))
                    {
                        outPts.Add(a);
                        outBulges.Add(bulge);
                        outPts.Add(b);
                        outBulges.Add(0.0);
                        i = bestEnd + 1;
                        continue;
                    }
                }

                outPts.Add(pts[i]);
                outBulges.Add(0.0);
                i++;
            }

            bulges = outBulges;
            var cs2 = CollapseStraightRuns(outPts, outBulges, 1e-4);
            bulges = cs2.bulges;
            return cs2.pts;
        }

        /// <summary>单圆心版本，转发到 <see cref="FitArcsAboutCenters"/>（默认参数与之保持一致）。</summary>
        public static List<Point2d> FitArcsAboutCenter(List<Point2d> pts, Point2d center, out List<double> bulges,
            double tol = 3e-2, double minArcAngleDeg = 4.0, int minArcPts = 4, int maxArcPts = 1500,
            double maxPush = 2e-2, double maxSag = 2e-2)
        {
            return FitArcsAboutCenters(pts, new[] { center }, out bulges, tol, minArcAngleDeg, minArcPts, maxArcPts, maxPush, maxSag);
        }

        /// <summary>
        /// 已知若干候选圆心（例如牵引车转弯中心 C1 与挂车转弯中心 C2），把边界点列中
        /// 到同一圆心距离近似恒定的连续段识别为 bulge 圆弧。比无约束圆拟合更稳健。
        ///
        /// 为什么需要多个圆心：铰接车稳态圆周时牵引车绕 C1 转动、挂车绕 C2 转动，
        /// 两者的瞬时中心不同（这正是挂车 off-tracking 的几何来源）。若只给 C1，
        /// 挂车那一段外廓一条弧都拟合不出来，出图会退化成上千个顶点的密集折线。
        ///
        /// 默认参数（tol=3cm, minArcAngleDeg=4°, minArcPts=4, maxPush=2cm, maxSag=2cm）
        /// 是 v4.5 在「铰接车 90° 转弯、0.25° 生产采样」这一最苛刻场景上实测标定出来的：
        ///   1215 个原始顶点 → 118 个顶点 / 13 条弧，漏覆盖 1.6mm、外扩 2.4cm、锯齿顶点 0。
        /// tol 的物理意义：并集边界是「离散帧车体矩形的并集」，沿弧线是一串台阶，
        ///   台阶的径向振幅 ≈ 帧间转角 × 车体尺寸，0.25° 采样下实测 3cm。tol 必须 ≥ 振幅/2
        ///   才能把台阶整段吞进一条弧；取 3cm（允许 6cm 极差）留了一倍余量。
        ///   继续放大 tol 收益很小（0.03→0.12 只把顶点数从 118 降到 112），却会让弧外扩更多。
        /// 保证不内缩的机制不依赖 tol：弧半径按「材料在圆盘哪一侧」取 rMax 或 rMin
        ///   （见函数内注释），外凸段永远外扩、内凹段永远向空腔让。
        /// maxPush：弧端点沿径向推到 rArc 的最大位移，限制接缝被拉出的长度。
        /// maxSag：相邻两原始点所夹圆心角对应的弦矢高上限，即弧相对原始折线的最大偏离。
        ///   少了它，只有首尾 2 个顶点的长直边（车体初始/终止姿态，最长 9.5m）会被弧
        ///   "抄近路"，实测把包络向外顶出 0.635 m。
        /// minArcAngleDeg：最小圆心角；minArcPts/maxArcPts 控制段长。
        /// maxArcPts 必须够大（默认 1500），否则 0.25° 采样下 90° 弧（360 点）会被切成两段。
        /// </summary>
        public static List<Point2d> FitArcsAboutCenters(List<Point2d> pts, IList<Point2d> centers, out List<double> bulges,
            double tol = 3e-2, double minArcAngleDeg = 4.0, int minArcPts = 4, int maxArcPts = 1500,
            double maxPush = 2e-2, double maxSag = 2e-2)
        {
            bulges = new List<double>();
            if (pts == null || pts.Count < 3)
            {
                for (int k = 0; pts != null && k < pts.Count; k++) bulges.Add(0);
                return pts != null ? new List<Point2d>(pts) : new List<Point2d>();
            }

            var cands = new List<Point2d>();
            if (centers != null)
                foreach (var c in centers)
                    if (!double.IsNaN(c.X) && !double.IsNaN(c.Y)) cands.Add(c);
            if (cands.Count == 0)
            {
                for (int k = 0; k < pts.Count; k++) bulges.Add(0);
                return new List<Point2d>(pts);
            }

            int n = pts.Count;
            var outPts = new List<Point2d>();
            var outBulges = new List<double>();

            // 环的绕向：决定「材料」在行进方向的哪一侧，进而决定弧半径该取 rMax 还是 rMin。
            bool ringCCW = SignedArea(pts) > 0;

            int i = 0;
            while (i < n)
            {
                int bestEnd = -1;
                Point2d bestCenter = default;
                double bestRArc = 0, bestSweep = 0;

                foreach (var center in cands)
                {
                    int end = -1;
                    double endRArc = 0, endSweep = 0;

                    // 只增量维护半径的 min / max：这一步是 O(1)，保证 maxArcPts 可以开到
                    // 1500 而整体仍是 O(n · maxArcPts)。若改成每个 j 都重扫一遍求均值/残差，
                    // 复杂度会变成 O(n · maxArcPts²)，90° 弧（360 点）根本跑不动。
                    // 极差关于 j 单调不减，一旦 range > 2*tol 就一定有 maxErr > tol，可安全 break；
                    // 于是 max|r - avgR| ≤ (rMax-rMin)/2 ≤ tol 自动成立，无需再算真实残差。
                    double rMin = double.MaxValue, rMax = double.MinValue;
                    double prevAng = 0, sweep = 0;
                    int dir = 0;
                    bool matInside = false;
                    var d0 = pts[i] - center;
                    double rStart = Math.Sqrt(d0.X * d0.X + d0.Y * d0.Y);
                    int jLimit = Math.Min(n, i + maxArcPts);

                    for (int j = i; j < jLimit; j++)
                    {
                        var dp = pts[j] - center;
                        double r = Math.Sqrt(dp.X * dp.X + dp.Y * dp.Y);
                        if (r < rMin) rMin = r;
                        if (r > rMax) rMax = r;
                        if (rMax - rMin > 2.0 * tol) break;

                        // 极角必须沿同一方向单调推进。
                        // 这是必须的：包络边界会在同一条圆弧上来回切换车体角点，
                        // 若只校验「半径恒定」而不看角度方向，拟合弧会用首尾点「抄近路」，
                        // 把真实弯曲切掉、向外鼓出（实测可让包络面积虚增 17%）。
                        double ang = Math.Atan2(dp.Y, dp.X);
                        if (j > i)
                        {
                            double da = ang - prevAng;
                            while (da > Math.PI) da -= 2.0 * Math.PI;
                            while (da <= -Math.PI) da += 2.0 * Math.PI;
                            if (j == i + 1)
                            {
                                if (Math.Abs(da) < 1e-14) break; // 前两点重合，定不出旋向
                                dir = da > 0 ? 1 : -1;
                                // 定向后的 dAng 符号恒等于 dir，因此这里就能定下 rMax / rMin 的取舍
                                matInside = (dir > 0) == ringCCW;
                            }
                            else if (dir * da < -1e-12) break;   // 极角回退，run 到此为止

                            // ====== 相邻点角步约束（v4.5 第四个关键修正）======
                            // 「窗口内半径几乎恒定」只是必要条件：判据只看 run 里出现过的
                            // 采样点，而这些点可能全挤在两端。典型受害者是车体初始/终止姿态
                            // 的长直边 —— 一条 9.5 m 的直边只有首尾 2 个顶点，半径判据轻松通过，
                            // 于是被一段 R≈18 m 的弧"抄近路"，弧中点相对直边外凸 0.635 m。
                            //
                            // 直接约束几何量：相邻两点所夹圆心角 da 对应的弦矢高
                            //   sag = r · (1 - cos(da/2))
                            // 就是弧相对这一段原始折线的最大偏离。要求 sag ≤ maxSag，等价于
                            //   |da| ≤ 2·acos(1 - maxSag / r)
                            // 取 maxSag = 2cm、R=20m 时阈值约 2.6°；而锯齿台阶的角步只有
                            // 0.125°（矢高 1e-5 m），正常弧完全不受影响，只有长直边会被挡下。
                            double argC = 1.0 - maxSag / Math.Max(r, 1e-6);
                            if (argC < -1.0) argC = -1.0; else if (argC > 1.0) argC = 1.0;
                            if (Math.Abs(da) > 2.0 * Math.Acos(argC)) break;

                            sweep += da;
                        }
                        prevAng = ang;

                        // ====== 端点位移约束（v4.5 第三个关键修正）======
                        // 弧的两个端点要沿径向推到 rSel，才能与 bulge 一起构成精确的
                        // (center, rSel) 圆。若某个端点本来离 rSel 很远（落在锯齿谷底），
                        // 这一推就是几厘米甚至更多，而相邻折线仍按原始点连接 ——
                        // 弧端点与相邻顶点之间会被拉出一条长直线，把包络局部顶出去。
                        // 实测未加此约束时铰接车包络局部虚胖 0.635 m。
                        //
                        // 起点侧：rSel 随 j 单调远离 rStart，一旦超差就再也不会回来 → break。
                        // 终点侧：r 沿锯齿在 [rMin, rMax] 间起伏，本次超差下一次可能合格 → continue。
                        if (dir != 0)
                        {
                            double rSelCand = matInside ? rMax : rMin;
                            if (Math.Abs(rStart - rSelCand) > maxPush) break;
                            if (Math.Abs(r - rSelCand) > maxPush) continue;
                        }

                        int len = j - i + 1;
                        if (len < minArcPts) continue;

                        // 圆心角用累积 sweep，而不是首尾两点的 atan2 —— 后者在角度
                        // 非单调时会折返，得到错误的小角甚至反号。
                        double sweepDeg = Math.Abs(sweep) * 180.0 / Math.PI;
                        if (sweepDeg < minArcAngleDeg) continue;
                        if (sweepDeg > 350.0) break; // 接近整圈，单段 bulge 无法表达

                        // ====== 圆心角直接取两端点的真实极角差（v4.5 关键修正）======
                        // 出图的弧由「两端点 + bulge」唯一确定，实际半径 = 弦长/(2·sin(θ/2))。
                        // 只有 θ 等于两端点绕 center 的真实极角差时，画出来的弧才正好是以
                        // center 为心、rArc 为半径的那段圆；此时"弧上每点半径 = rArc
                        // ≥ 窗口内所有点半径"，覆盖性由构造保证，与 tol 取多大无关。
                        //
                        // 不能用累积 sweep 当 θ：包络边界会在不同车体角点之间切换，切换瞬间
                        // 极角跳一大步，sweep 把跳变算进去 → θ 偏大 → 弧半径被反算成更小 →
                        // 弧向内收缩，把车体角点漏在外面（实测漏覆盖 0.62 m）。
                        // 更早期版本用 tol 做"弦-角-半径自洽"容差，等于允许了 tol 量级的半径
                        // 误差；tol 放宽到 3 cm 消锯齿后实测漏覆盖 4.9 cm，故彻底改为取真实角差。
                        //
                        // 极角差先归一到 (-π, π]，再按旋向 dir 定向还原，
                        // 否则 |θ|>180° 的优弧会被折成反方向的劣弧。
                        var da0 = pts[i] - center;
                        var db0 = pts[j] - center;
                        double angA = Math.Atan2(da0.Y, da0.X);
                        double angB = Math.Atan2(db0.Y, db0.X);
                        double dAng = angB - angA;
                        while (dAng > Math.PI) dAng -= 2.0 * Math.PI;
                        while (dAng <= -Math.PI) dAng += 2.0 * Math.PI;
                        if (dir > 0 && dAng < 0) dAng += 2.0 * Math.PI;
                        else if (dir < 0 && dAng > 0) dAng -= 2.0 * Math.PI;
                        if (Math.Abs(dAng) * 180.0 / Math.PI < minArcAngleDeg) continue;
                        // 上限取 179°（而非 350°）：|θ|>180° 时圆心跑到弦的另一侧，
                        // 下面「材料在圆盘哪一侧」的判据会整体反过来，单段 bulge 也难以可靠表达。
                        // 实际包络的单段弧都在 90°~120° 量级，179° 足够。
                        if (Math.Abs(dAng) * 180.0 / Math.PI > 179.0) break;

                        // ====== 弧半径取 rMax 还是 rMin（v4.5 第二个关键修正）======
                        // 圆心角 |θ|<180° 时，弧总是向「背离圆心」的一侧鼓出。于是：
                        //   · 材料在圆盘内部（圆心位于材料一侧）→ 弧必须比所有原始点更远离圆心
                        //     → rArc = rMax（外扩，绝不内缩，这正是包络"最外延"的语义）；
                        //   · 材料在圆盘外部（圆心位于空腔一侧）→ 弧必须比所有原始点更靠近圆心
                        //     → rArc = rMin（弧退到空腔一侧，等价于把内凹边界往外让）。
                        //
                        // 第二种情况不是假想：铰接车 off-tracking 的内侧边界正是一个面朝
                        // 转弯中心的内凹段，圆心落在扫掠空腔里。若一律取 rMax，弧会被
                        // 推进车体内部 —— 实测产生 165 个深入车体 >5cm 的虚构边界点，
                        // 这也是 v4.5 开发中期铰接车 bulge 拟合 FAIL 的真正根因。
                        //
                        // 判据：CCW 环的材料在行进方向左侧。绕圆心 CCW 前进（θ>0）时左侧朝向
                        // 圆心，材料在圆盘内；绕圆心 CW 前进（θ<0）时左侧背离圆心，材料在圆盘外。
                        // CW 环（材料在右侧）结论正好相反，故统一写成 (θ>0) == ringCCW。
                        // matInside 在 j==i+1 定旋向时已定，与 (dAng>0)==ringCCW 等价
                        // （定向后的 dAng 符号恒等于 dir），这里直接复用。
                        double rSel = matInside ? rMax : rMin;

                        // 取最长段（end 随 j 递增被不断覆盖），而不是"残差最小"的那段 ——
                        // 后者会把一整条 90° 弧切成十几段 5°~15° 的碎片，出图呈扇贝状锯齿。
                        // 端点随后沿径向推到 rSel，画出的弧恰好是 (center, rSel) 圆，
                        // 覆盖性由构造保证，与 tol 取多大无关。
                        end = j; endRArc = rSel; endSweep = dAng;
                    }

                    if (end > bestEnd)
                    {
                        bestEnd = end; bestCenter = center;
                        bestRArc = endRArc; bestSweep = endSweep;
                    }
                }

                if (bestEnd > i)
                {
                    // 两端点沿径向推到 bestRArc（外凸段推远、内凹段拉近，见上面判据），
                    // 保证拟合弧恰好落在这一段所有原始边界点的「非材料侧」，覆盖性由构造保证。
                    Point2d a = PushRadial(pts[i], bestCenter, bestRArc);
                    Point2d b = PushRadial(pts[bestEnd], bestCenter, bestRArc);
                    double bulge = Math.Tan(bestSweep / 4.0);
                    if (Math.Abs(bulge) > 1e-6 && !double.IsNaN(bulge) && !double.IsInfinity(bulge))
                    {
                        // 弧终点也作为一个顶点输出，保证 bulge 的圆心角与端点严格对应
                        outPts.Add(a);
                        outBulges.Add(bulge);
                        outPts.Add(b);
                        outBulges.Add(0.0);
                        i = bestEnd + 1;
                        continue;
                    }
                }

                outPts.Add(pts[i]);
                outBulges.Add(0.0);
                i++;
            }

            bulges = outBulges;
            var cs = CollapseStraightRuns(outPts, outBulges, 1e-4);
            bulges = cs.bulges;
            return cs.pts;
        }

        /// <summary>
        /// 把 bulge=0 的连续直线段用 RDP 简化（容忍 tol 米的偏离），圆弧段及其端点原样保留。
        /// 返回 (新顶点, 新bulge) —— bulge 列表长度始终等于顶点列表长度。
        ///
        /// 语义约定：bulges[k] 描述「顶点 pts[k] → 顶点 pts[(k+1)%n]」这一段（闭环）。
        /// 因此 bulge≠0 的段，其起点和终点都是**关键点**，绝对不能被 RDP 删掉 —— 否则
        /// bulge 就会挂到别的顶点上，圆弧错位甚至整条弧退化成直线。
        /// 历史 bug：曾按「顶点 bulge==0」分组，导致弧起点的 bulge 被清零、
        /// 弧终点被 RDP 吃掉，出图退化成 31 个顶点的折线（正是锯齿的来源）。
        /// </summary>
        static (List<Point2d> pts, List<double> bulges) CollapseStraightRuns(List<Point2d> pts, List<double> bulges, double tol)
        {
            if (pts == null || bulges == null || pts.Count < 3 || bulges.Count != pts.Count)
                return (pts ?? new List<Point2d>(), bulges ?? new List<double>());
            int n = pts.Count;

            // 没有任何圆弧时，整条是闭合折线，不做整环 RDP：
            // 闭合环首尾点重合，RDP 的初始弦退化成零长度，简化结果不可控。
            int a0 = -1;
            for (int k = 0; k < n; k++)
                if (Math.Abs(bulges[k]) > 1e-9) { a0 = k; break; }
            if (a0 < 0) return (pts, bulges);

            // 把环旋转到「以某条弧的起点开头」，保证顶点 0 一定是直线段的边界，
            // 从而任何一条直线段都不会跨越数组首尾（无需处理 wrap-around）。
            var P = new List<Point2d>(n + 1);
            var B = new List<double>(n);
            for (int t = 0; t < n; t++)
            {
                P.Add(pts[(a0 + t) % n]);
                B.Add(bulges[(a0 + t) % n]);
            }
            P.Add(P[0]); // 最后一段：P[n-1] → P[0]

            var newPts = new List<Point2d>();
            var newBulges = new List<double>();

            int s = 0;
            while (s < n)
            {
                if (Math.Abs(B[s]) > 1e-9)
                {
                    // 圆弧段：起点 + bulge，终点由下一段（或闭合）承接
                    newPts.Add(P[s]);
                    newBulges.Add(B[s]);
                    s++;
                }
                else
                {
                    int e = s;
                    while (e < n && Math.Abs(B[e]) < 1e-9) e++;
                    // 直线段 s..e-1，覆盖顶点 P[s..e]
                    var sub = new List<Point2d>();
                    for (int k = s; k <= e; k++) sub.Add(P[k]);
                    var simp = SimplifyPolyline(sub, tol);
                    // 末点不在这里输出：它是下一段的起点（或环的首点），由下一段负责
                    for (int k = 0; k < simp.Count - 1; k++)
                    {
                        newPts.Add(simp[k]);
                        newBulges.Add(0.0);
                    }
                    s = e;
                }
            }
            return (newPts, newBulges);
        }
        #endregion
    }
}
