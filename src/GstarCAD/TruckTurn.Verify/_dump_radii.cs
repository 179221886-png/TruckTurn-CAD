using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using TruckTurn;

namespace TruckTurn
{
    /// <summary>
    /// _dump_radii.cs — 输出「车型 × 参数 → 四种转弯半径」对照表 CSV（供说明文档/Excel 用）。
    /// 参数推导严格复刻 TruckSizeForm.BuildResult 的比例（刚性：v4.9.24 起前悬=clamp(0.12L,1.1,1.5)/
    /// 后悬=clamp(0.20L,0.8,2.6)/轴距=余量；铰接：牵引车 L=0.60/前悬0.25/鞍座0.10×牵引车长，挂车 L2=0.85×挂车长）。
    /// 半径直接调 TruckKinematics.TurnRadii，保证与插件出图标注逐点一致。
    /// 另输出铰接车稳态铰接角/内轮差（解析式：Rk=sqrt(R1²+ak²)，sinθ=L2/Rk）。
    /// </summary>
    partial class VerifyProgram
    {
        private static VehicleParams Rigid(double totalM, double widthM, double steerDeg)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = false;
            p.UnitScale = 1.0;
            p.SteerAngleDeg = steerDeg;
            // v4.9.24：与 TruckSizeForm.RigidLayoutM 保持一致
            double fo = Math.Min(1.5, Math.Max(1.1, 0.12 * totalM));
            double ro = Math.Min(2.6, Math.Max(0.8, 0.20 * totalM));
            p.TractorFrontOverhang = fo;
            p.TractorWheelbase = Math.Max(1.2, totalM - fo - ro);
            p.TractorRearToKingpin = 0;
            p.TractorWidth = widthM;
            p.RigidRearOverhang = ro;
            p.WheelTrack = Math.Max(0.9, widthM * 0.75);
            p.TrailerKingpinToRearAxle = 0;
            p.TrailerWidth = widthM;
            return p;
        }

        private static VehicleParams Semi(double tractorM, double trailerM, double widthM, double steerDeg)
        {
            var p = VehicleParams.Defaults();
            p.Articulated = true;
            p.UnitScale = 1.0;
            p.SteerAngleDeg = steerDeg;
            p.TractorFrontOverhang = 0.25 * tractorM;
            p.TractorWheelbase = 0.60 * tractorM;
            p.TractorRearToKingpin = 0.10 * tractorM;
            p.TractorWidth = widthM;
            double tre = 0.12 * trailerM, tf = 0.03 * trailerM;
            p.TrailerRearOverhang = tre;
            p.TrailerKingpinToFront = tf;
            p.TrailerKingpinToRearAxle = trailerM - tre - tf;
            p.TrailerWidth = widthM;
            p.WheelTrack = Math.Max(0.9, widthM * 0.75);
            return p;
        }

        public static void _dumpRadii()
        {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            string csv = Path.Combine(dir, "_radii_dump.csv");
            var inv = CultureInfo.InvariantCulture;
            using (var w = new StreamWriter(csv, false, System.Text.Encoding.UTF8))
            {
                w.WriteLine("SECTION,车型,车长m,车宽m,轴距L_m,前悬m,轮距m,转向角deg,挂车L2_m,鞍座ak_m,R1后轴中心m,前轮外缘m,车身外角m,后内轮m,稳态铰接角deg,挂车内轮差m,限位45度最小持续R1_m");

                // ---- SECTION=PRESET：6 种选型窗预设 ----
                var presets = new (string name, VehicleParams p)[]
                {
                    // v4.9.24：转向角与 TruckSizeForm.Presets 同步（实车约 40°，铰接车受稳态铰接角限制）
                    ("微型货车", Rigid(4.5, 1.70, 40)),
                    ("轻型货车", Rigid(6.8, 2.20, 40)),
                    ("中型货车", Rigid(9.6, 2.50, 38)),
                    ("重型整车", Rigid(12.0, 2.55, 38)),
                    // 铰接两档维持 20/18：实测再大稳态铰接角就爆表（24°→70.7°、20°→≥90°，实车限位约 45~55°）
                    ("二轴半挂", Semi(6.0, 9.0, 2.50, 20)),
                    ("三轴半挂", Semi(6.0, 13.0, 2.55, 18)),
                };
                foreach (var (name, p) in presets) WriteRow(w, "PRESET", name, p);

                // ---- SECTION=STEER_SWEEP：三轴半挂转向角扫描（其余参数不变）----
                foreach (double s in new[] { 15.0, 18.0, 20.0, 25.0, 30.0 })
                    WriteRow(w, "STEER_SWEEP", "三轴半挂", Semi(6.0, 13.0, 2.55, s));

                // ---- SECTION=LEN_SWEEP：半挂挂车长扫描（牵引车 6m/转向 18° 不变）----
                foreach (double t in new[] { 9.0, 11.0, 13.0, 15.0, 17.0 })
                    WriteRow(w, "LEN_SWEEP", "半挂(挂车长变)", Semi(6.0, t, 2.55, 18));

                // ---- SECTION=RIGID_LEN_SWEEP：刚性车长扫描（宽 2.5/转向 28° 不变）----
                foreach (double t in new[] { 4.5, 6.0, 8.0, 9.6, 12.0, 14.0 })
                    WriteRow(w, "RIGID_LEN_SWEEP", "刚性(车长变)", Rigid(t, 2.50, 28));

                // ---- SECTION=MATRIX：通用 轴距×转向角 → R1 矩阵 ----
                foreach (double L in new[] { 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.6, 7.2 })
                    foreach (double s in new[] { 15.0, 18.0, 20.0, 25.0, 28.0, 30.0, 35.0, 40.0 })
                    {
                        var p = Rigid(L / 0.55, 2.5, s);   // 反解出目标轴距
                        var r = TruckKinematics.TurnRadii(p);
                        w.WriteLine(string.Format(inv,
                            "MATRIX,,,,{0:F2},,,{1:F0},,,{2:F2},,,,,,",
                            L, s, r.Centerline));
                    }
            }
            Console.WriteLine("CSV: " + csv);
        }

        private static void WriteRow(StreamWriter w, string section, string name, VehicleParams p)
        {
            var inv = CultureInfo.InvariantCulture;
            var r = TruckKinematics.TurnRadii(p);
            double len = p.LengthMeters();
            double L = p.TractorWheelbase, fh = p.TractorFrontOverhang;
            double track = p.WheelTrack, steer = p.SteerAngleDeg;
            double L2 = p.Articulated ? p.TrailerKingpinToRearAxle : 0;
            double ak = p.Articulated ? p.TractorRearToKingpin : 0;

            // 铰接稳态（解析）：Rk = sqrt(R1² + ak²)，sinθ = L2/Rk，内轮差 = Rk − Rk·cosθ
            string ssTheta = "", offTrack = "", minSustain = "";
            if (p.Articulated)
            {
                double Rk = Math.Sqrt(r.Centerline * r.Centerline + ak * ak);
                if (L2 < Rk)
                {
                    double theta = Math.Asin(L2 / Rk) * 180.0 / Math.PI;
                    double off = Rk - Math.Sqrt(Rk * Rk - L2 * L2);
                    ssTheta = theta.ToString("F1", inv);
                    offTrack = off.ToString("F2", inv);
                }
                else { ssTheta = ">=90(不可达)"; offTrack = "-"; }
                // 最大铰接角 45° 限位下的最小可持续 R1：sqrt(R1²+ak²) = L2/sin45°
                double need = L2 / Math.Sin(45.0 * Math.PI / 180.0);
                double r1min = Math.Sqrt(Math.Max(0.0, need * need - ak * ak));
                minSustain = r1min.ToString("F2", inv);
            }

            w.WriteLine(string.Format(inv,
                "{0},{1},{2:F1},{3:F2},{4:F2},{5:F2},{6:F2},{7:F0},{8},{9},{10:F2},{11:F2},{12:F2},{13:F2},{14},{15},{16}",
                section, name, len, p.TractorWidth, L, fh, track, steer,
                p.Articulated ? L2.ToString("F2", inv) : "-",
                p.Articulated ? ak.ToString("F2", inv) : "-",
                r.Centerline, r.CurbToCurb, r.WallToWall, r.InnerTire,
                ssTheta, offTrack, minSustain));
        }
    }
}
