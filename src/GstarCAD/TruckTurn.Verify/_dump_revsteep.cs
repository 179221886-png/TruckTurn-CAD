using System;
using Gssoft.Gscad.Geometry;

namespace TruckTurn
{
    /// <summary>
    /// _dump_revsteep.cs — 验证「倒退目标点靠近车尾正后方延长线时，扫掠角陡峭跳变」。
    /// 几何事实：R2 = D2/(2|d|)，d→0 时 R→∞ 且圆心角 → 180°；
    /// 悬停点 A（d 大、扫掠小）与落点 B（d 小、扫掠近 180°）只相差一两米，
    /// 但路径天差地别——「预览小转角、确认绕大半圈」，
    /// 且车尾精确到达光标点 ⇒ 用户感觉「终点对、多绕一圈」。
    /// </summary>
    internal static partial class VerifyProgram
    {
        static void _dumpRevSteep()
        {
            var p = VehicleParams.Defaults();
            p.Articulated = true;
            var cur = MakeStartFrame(p, new Point2d(0, 0), new Vector2d(1, 0));
            // 车尾控制点 O2 在原点后方；u2 朝 +X。目标点取在 O2 后方 a=-8m、横向 d 从 6m 扫到 0.1m
            Console.WriteLine("O2=({0:F2},{1:F2})  挂车航向 +X", cur.O2.X, cur.O2.Y);
            Console.WriteLine("目标相对O2: a=-8m 固定, d 变化");
            Console.WriteLine("   d(m)   分支   R2(m)     扫掠角°   side");
            foreach (double d in new[] { 6, 4, 3, 2, 1.5, 1, 0.7, 0.5, 0.3, 0.2, 0.1 })
            {
                var target = new Point2d(cur.O2.X - 8.0, cur.O2.Y + d);
                int side; double turnDeg, rR, straight;
                string branch = "精确";
                bool ok = TruckKinematics.TrySolveReverseArc(p, cur, target,
                    out side, out turnDeg, out rR, out straight);
                if (!ok)
                {
                    branch = "回退";
                    ok = TruckKinematics.ReverseTurnTowardAtMinRadius(p, cur, target,
                        out side, out turnDeg, out rR);
                }
                Console.WriteLine("{0,6:F1}  {1} {2,8:F1} {3,9:F1} {4,5}   {5}",
                    d, branch, rR, turnDeg, ok ? side : 0, ok ? "" : "不可行");
            }

            // 决定性验证：全方向×全距离网格，找「相距 <2m 但扫掠角差 >60°」的相邻目标对
            // —— 若存在，则「鼠标快速移动/捕捉跳变后立刻点击」就会让确认段与所见预览差出半圈。
            Console.WriteLine("\n相邻陡变对（|A-B|<2m 且 |Δ扫掠|>60°）：");
            double[] dists = { 5, 8, 12, 16, 20, 26, 32, 40, 50, 65 };
            int pairs = 0;
            for (int angI = 0; angI < 72; angI++)
            {
                double beta = angI * 5.0 * Math.PI / 180.0;
                for (int di = 0; di < dists.Length - 1; di++)
                {
                    double sweepA, sweepB, rA, rB; bool okA, okB; string brA, brB;
                    Solve(cur, p, beta, dists[di], out sweepA, out rA, out okA, out brA);
                    Solve(cur, p, beta, dists[di + 1], out sweepB, out rB, out okB, out brB);
                    if (!okA || !okB) continue;
                    if (Math.Abs(sweepA - sweepB) > 60.0)
                    {
                        pairs++;
                        Console.WriteLine("  极角{0,4:F0}° dist {1,4:F0}→{2,4:F0}  扫掠 {3,7:F1}°→{4,7:F1}°  ({5}→{6})",
                            angI * 5.0, dists[di], dists[di + 1], sweepA, sweepB, brA, brB);
                    }
                }
                // 同 dist、相邻角度（5°网格 ≈ dist·0.087m 弧距，dist≤23 时 <2m）
                double sweep1, sweep2, r1, r2; bool ok1, ok2; string b1, b2;
                double beta2 = (angI + 1) * 5.0 * Math.PI / 180.0;
                foreach (double dist in dists)
                {
                    if (dist * 5.0 * Math.PI / 180.0 > 2.0) continue;
                    Solve(cur, p, beta, dist, out sweep1, out r1, out ok1, out b1);
                    Solve(cur, p, beta2, dist, out sweep2, out r2, out ok2, out b2);
                    if (!ok1 || !ok2) continue;
                    if (Math.Abs(sweep1 - sweep2) > 60.0)
                    {
                        pairs++;
                        Console.WriteLine("  dist{0,5:F0} 极角 {1,4:F0}°→{2,4:F0}°  扫掠 {3,7:F1}°→{4,7:F1}°  ({5}→{6})",
                            dist, angI * 5.0, (angI + 1) * 5.0, sweep1, sweep2, b1, b2);
                    }
                }
            }
            Console.WriteLine(pairs == 0 ? "  （无）" : "共 " + pairs + " 对");
        }

        static void Solve(Frame cur, VehicleParams p, double beta, double dist,
                          out double sweep, out double rR, out bool ok, out string branch)
        {
            var target = new Point2d(cur.O2.X + dist * Math.Cos(beta),
                                     cur.O2.Y + dist * Math.Sin(beta));
            int side; double turnDeg, straight;
            branch = "精确";
            ok = TruckKinematics.TrySolveReverseArc(p, cur, target,
                out side, out turnDeg, out rR, out straight);
            if (!ok)
            {
                branch = "回退";
                ok = TruckKinematics.ReverseTurnTowardAtMinRadius(p, cur, target,
                    out side, out turnDeg, out rR);
            }
            sweep = ok ? turnDeg : 0.0;
            if (!ok) rR = 0.0;
        }
    }
}
