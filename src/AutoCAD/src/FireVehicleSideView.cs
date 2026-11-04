using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;
using System.Globalization;

namespace TruckTurn
{
    /// <summary>纯展示绘图。米制尺寸只读，不生成/修改任何车辆参数或运动几何。</summary>
    internal static class FireVehicleSideView
    {
        static readonly Color Ink = Color.FromArgb(58, 70, 80);
        static readonly Color Muted = Color.FromArgb(121, 133, 143);
        static readonly Color Rule = Color.FromArgb(217, 224, 229);
        static readonly Color Red = Color.FromArgb(177, 57, 65);
        static readonly Color RedEdge = Color.FromArgb(130, 45, 54);
        static readonly Color Metal = Color.FromArgb(237, 241, 243);
        static readonly Color MetalEdge = Color.FromArgb(172, 184, 192);

        public static void Draw(Graphics g, RectangleF bounds, double front, double wheelbase,
            double rear, double height, string message = null)
        {
            if (g == null || !Finite(bounds.Width) || !Finite(bounds.Height) ||
                bounds.Width < 1 || bounds.Height < 1) return;
            GraphicsState state = g.Save();
            try
            {
                g.SetClip(bounds, CombineMode.Intersect);
                g.TranslateTransform(bounds.X, bounds.Y);
                // Match Windows font scaling, not CAD drawing units.
                float ui = Math.Max(1f, g.DpiX / 96f);
                g.ScaleTransform(ui, ui);
                float w = bounds.Width / ui, h = bounds.Height / ui;
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                g.TextRenderingHint = TextRenderingHint.AntiAliasGridFit;
                g.FillRectangle(Brushes.White, 0, 0, w, h);
                if (w < 150 || h < 120) return;

                using (var heading = Font(11.5f, FontStyle.Bold))
                using (var label = Font(10f))
                using (var tiny = Font(9f))
                using (var ink = new SolidBrush(Ink))
                using (var muted = new SolidBrush(Muted))
                using (var accent = new SolidBrush(Red))
                using (var border = new Pen(Rule, 1f))
                {
                    g.DrawRectangle(border, .5f, .5f, w - 1, h - 1);
                    g.FillRectangle(accent, 16, 15, 3, 12);
                    g.DrawString("消防车侧视", heading, ink, 26, 12);
                    RightText(g, "行驶收拢状态", tiny, muted, w - 16, 15);
                    g.DrawLine(border, 16, 38, w - 16, 38);

                    double length = front + wheelbase + rear;
                    if (message == null && (!Finite(front) || !Finite(wheelbase) || !Finite(rear) ||
                        !Finite(height) || !Finite(length) || front < 0 || rear < 0 ||
                        wheelbase <= 0 || height <= 0 || length <= 0))
                        message = "输入有效参数后显示车辆侧视图";
                    if (message != null || w < 240 || h < 190)
                    {
                        using (var format = new StringFormat { Alignment = StringAlignment.Center,
                            LineAlignment = StringAlignment.Center })
                            g.DrawString(message ?? "放大窗口以查看车辆侧视图", label, muted,
                                new RectangleF(22, 48, w - 44, h - 82), format);
                        return;
                    }

                    float ground = h - 65;
                    double scale = Math.Min((w - 86) / length, (h - 132) / height);
                    if (!Finite(scale) || scale < 1e-6 || scale > 1e6) return;
                    var v = new View(g, (float)((w - length * scale) * .5), ground, (float)scale);

                    // A short datum, rather than a decorative grid or a rendered floor.
                    g.DrawLine(border, v.X(0) - 14, ground + .5f, v.X(length) + 14, ground + .5f);
                    Vehicle(v, front, wheelbase, rear, height);

                    double top = height;
                    HorizontalDimension(g, label, v.X(0), v.X(length), 55,
                        "整车长  " + Number(length) + " m");
                    g.DrawLine(border, v.X(0), 51, v.X(0), v.Y(top) - 5);
                    g.DrawLine(border, v.X(length), 51, v.X(length), v.Y(top) - 5);
                    float axle1 = v.X(front), axle2 = v.X(front + wheelbase);
                    g.DrawLine(border, axle1, ground + 5, axle1, ground + 25);
                    g.DrawLine(border, axle2, ground + 5, axle2, ground + 25);
                    HorizontalDimension(g, label, axle1, axle2, ground + 21,
                        "轴距  " + Number(wheelbase) + " m");
                    VerticalDimension(g, tiny, v.X(length) + 18, v.Y(height), ground,
                        "车高 " + Number(height) + " m");
                    g.DrawString("外观示意 · 高度不参与二维包络计算", tiny, muted, 16, h - 20);
                }
            }
            finally { g.Restore(state); }
        }

        static void Vehicle(View v, double front, double wb, double rear, double h)
        {
            Graphics g = v.G;
            double l = front + wb + rear;
            double r = Math.Min(Math.Min(.48, h * .16), Math.Min(wb * .22, l * .09));
            double arch = r * 1.15;
            double sill = r * 1.22;
            double cabLength = Math.Min(l * .43, Math.Max(l * .23, front + Math.Min(1.05, wb * .27)));
            double cabTop = h * .87;
            double boxTop = h * .94;
            double boxStart = Math.Min(l * .76, cabLength + l * .018);
            double boxEnd = l - l * .009;
            double boxLength = boxEnd - boxStart;

            using (var chassis = new SolidBrush(Color.FromArgb(65, 76, 84)))
            using (var edge = new Pen(RedEdge, 1.25f))
            using (var red = new SolidBrush(Red))
            using (var darkRed = new SolidBrush(Color.FromArgb(153, 45, 54)))
            using (var metal = new SolidBrush(Metal))
            using (var metalPen = new Pen(MetalEdge, .8f))
            using (var detail = new Pen(Color.FromArgb(221, 166, 169), .8f))
            using (var glass = new SolidBrush(Color.FromArgb(184, 205, 216)))
            using (var glassEdge = new Pen(Color.FromArgb(96, 126, 143), .9f))
            using (var shadow = new SolidBrush(Color.FromArgb(20, 42, 53, 63)))
            {
                foreach (double axle in new[] { front, front + wb })
                    g.FillEllipse(shadow, v.X(axle - r * 1.15), v.Ground - 1, (float)(r * 2.3 * v.Scale), 3);
                g.FillRectangle(chassis, v.Rect(l * .02, r * .72, l * .96, r * .45));

                using (var cab = new GraphicsPath())
                {
                    cab.AddPolygon(new[] {
                        v.P(0, sill), v.P(0, cabTop * .55), v.P(cabLength * .045, cabTop * .81),
                        v.P(cabLength * .20, cabTop), v.P(cabLength * .94, cabTop),
                        v.P(cabLength, cabTop * .96), v.P(cabLength, sill) });
                    CutBody(v, cab, front, r, arch, red, edge, delegate
                    {
                        // Two separate side panes, a sloped screen and a restrained door seam.
                        double wx0 = cabLength * .115, wx1 = cabLength * .54, wx2 = cabLength * .91;
                        double wy0 = cabTop * .60, wy1 = cabTop * .91;
                        var pane = new[] { v.P(wx0, wy0), v.P(cabLength * .23, wy1),
                            v.P(wx1, wy1), v.P(wx1, wy0) };
                        g.FillPolygon(glass, pane); g.DrawPolygon(glassEdge, pane);
                        RectangleF second = v.Rect(wx1 + cabLength * .045, wy0,
                            wx2 - wx1 - cabLength * .045, wy1 - wy0);
                        g.FillRectangle(glass, second); g.DrawRectangle(glassEdge, second.X, second.Y, second.Width, second.Height);
                        g.DrawLine(detail, v.P(cabLength * .565, cabTop * .95), v.P(cabLength * .565, sill));
                        g.DrawLine(detail, v.P(cabLength * .955, wy0 - h * .035), v.P(cabLength * .955, sill));
                        g.DrawLine(metalPen, v.P(cabLength * .78, wy0 - h * .045), v.P(cabLength * .91, wy0 - h * .045));
                        g.FillRectangle(darkRed, v.Rect(0, sill, cabLength, h * .075));
                        g.FillRectangle(metal, v.Rect(0, h * .34, cabLength, h * .045));
                        g.FillRectangle(metal, v.Rect(cabLength * .65, sill, cabLength * .28, h * .035));
                        g.DrawLine(metalPen, v.P(cabLength * .66, sill + h * .045), v.P(cabLength * .93, sill + h * .045));
                    });
                }

                using (var body = new GraphicsPath())
                {
                    body.AddRectangle(v.Rect(boxStart, sill, boxLength, boxTop - sill));
                    CutBody(v, body, front + wb, r, arch, red, edge, delegate
                    {
                        double inset = Math.Min(.16, boxLength * .06);
                        double gap = Math.Min(.09, boxLength * .025);
                        int count = boxLength > 4.2 ? 3 : 2;
                        double paneWidth = (boxLength - inset * 2 - gap * (count - 1)) / count;
                        double bottom = Math.Max(sill + h * .13, h * .34);
                        double top = boxTop - h * .065;
                        for (int n = 0; n < count; n++)
                        {
                            double x = boxStart + inset + n * (paneWidth + gap);
                            RectangleF panel = v.Rect(x, bottom, paneWidth, top - bottom);
                            g.FillRectangle(metal, panel);
                            g.DrawRectangle(metalPen, panel.X, panel.Y, panel.Width, panel.Height);
                            int ribs = Math.Max(3, Math.Min(10, (int)(panel.Height / 7)));
                            for (int j = 1; j < ribs; j++)
                            {
                                float y = panel.Top + panel.Height * j / ribs;
                                g.DrawLine(metalPen, panel.Left + 1, y, panel.Right - 1, y);
                            }
                            g.DrawLine(glassEdge, panel.Left + panel.Width * .35f, panel.Bottom - 3,
                                panel.Left + panel.Width * .65f, panel.Bottom - 3);
                        }
                        g.FillRectangle(darkRed, v.Rect(boxStart, sill, boxLength, h * .075));
                        g.FillRectangle(metal, v.Rect(boxStart, h * .34, boxLength, h * .045));
                        // A small rear lamp is enough; no branding or invented model details.
                        using (var lamp = new SolidBrush(Color.FromArgb(221, 183, 109)))
                            g.FillRectangle(lamp, v.Rect(boxEnd - l * .016, h * .24, l * .012, h * .09));
                    });
                }

                // Stowed roof equipment is a line drawing, never a huge solid red rectangle.
                double ladderStart = boxStart + boxLength * .08, ladderEnd = boxEnd - boxLength * .04;
                foreach (double support in new[] { .23, .78 })
                    g.DrawLine(metalPen, v.P(boxStart + boxLength * support, boxTop),
                        v.P(boxStart + boxLength * support, h * .982));
                g.DrawLine(metalPen, v.P(ladderStart, h), v.P(ladderEnd, h));
                g.DrawLine(metalPen, v.P(ladderStart, h * .976), v.P(ladderEnd, h * .976));
                int rungs = Math.Max(3, Math.Min(11, (int)(boxLength * v.Scale / 26)));
                for (int i = 0; i <= rungs; i++)
                {
                    double x = ladderStart + (ladderEnd - ladderStart) * i / rungs;
                    g.DrawLine(metalPen, v.P(x, h), v.P(x, h * .976));
                }
                using (var blue = new SolidBrush(Color.FromArgb(99, 143, 166)))
                {
                    g.FillRectangle(chassis, v.Rect(cabLength * .25, cabTop, cabLength * .25, h * .016));
                    g.FillRectangle(blue, v.Rect(cabLength * .28, cabTop + h * .016, cabLength * .19, h * .035));
                }
                // Mirror and bumper stay within the illustrated overall length.
                g.DrawLine(glassEdge, v.P(cabLength * .075, cabTop * .79), v.P(cabLength * .025, cabTop * .78));
                g.FillRectangle(chassis, v.Rect(cabLength * .015, cabTop * .67, cabLength * .045, cabTop * .13));
                g.FillRectangle(metal, v.Rect(0, sill - h * .035, cabLength * .15, h * .045));
                foreach (double axle in new[] { front, front + wb }) Wheel(v, axle, r);
            }
        }

        static void CutBody(View v, GraphicsPath outer, double axle, double radius, double arch,
            Brush fill, Pen outline, Action details)
        {
            RectangleF wheel = v.Rect(axle - arch, radius - arch, arch * 2, arch * 2);
            using (var occupied = new Region(outer))
            using (var hole = new GraphicsPath())
            {
                hole.AddEllipse(wheel); occupied.Exclude(hole);
                v.G.FillRegion(fill, occupied);
                GraphicsState state = v.G.Save();
                try
                {
                    v.G.SetClip(occupied, CombineMode.Intersect);
                    details();
                    v.G.DrawPath(outline, outer);
                }
                finally { v.G.Restore(state); }
                state = v.G.Save();
                try { v.G.SetClip(outer, CombineMode.Intersect); v.G.DrawEllipse(outline, wheel); }
                finally { v.G.Restore(state); }
            }
        }

        static void Wheel(View v, double axle, double radius)
        {
            Graphics g = v.G;
            float r = (float)(radius * v.Scale), x = v.X(axle), y = v.Y(radius);
            if (r < .1f) return;
            using (var tire = new SolidBrush(Color.FromArgb(49, 58, 66)))
            using (var rim = new SolidBrush(Color.FromArgb(222, 228, 232)))
            using (var hub = new SolidBrush(Color.FromArgb(124, 139, 149)))
            using (var bead = new Pen(Color.FromArgb(104, 117, 127), .8f))
            {
                g.FillEllipse(tire, x - r, y - r, r * 2, r * 2);
                g.DrawEllipse(bead, x - r * .83f, y - r * .83f, r * 1.66f, r * 1.66f);
                g.FillEllipse(rim, x - r * .53f, y - r * .53f, r * 1.06f, r * 1.06f);
                g.DrawEllipse(bead, x - r * .53f, y - r * .53f, r * 1.06f, r * 1.06f);
                g.FillEllipse(hub, x - r * .17f, y - r * .17f, r * .34f, r * .34f);
            }
        }

        static void HorizontalDimension(Graphics g, System.Drawing.Font font, float x1, float x2, float y, string text)
        {
            using (var pen = new Pen(Muted, .75f))
            using (var brush = new SolidBrush(Muted))
            {
                g.DrawLine(pen, x1, y, x2, y);
                g.DrawLine(pen, x1 - 2, y + 3, x1 + 2, y - 3);
                g.DrawLine(pen, x2 - 2, y + 3, x2 + 2, y - 3);
                SizeF size = g.MeasureString(text, font);
                float x = (x1 + x2 - size.Width) / 2;
                g.FillRectangle(Brushes.White, x - 5, y - size.Height * .5f - 1, size.Width + 10, size.Height + 2);
                g.DrawString(text, font, brush, x, y - size.Height * .5f);
            }
        }

        static void VerticalDimension(Graphics g, System.Drawing.Font font, float x, float y1, float y2, string text)
        {
            using (var pen = new Pen(Muted, .75f))
            using (var brush = new SolidBrush(Muted))
            {
                g.DrawLine(pen, x, y1, x, y2);
                g.DrawLine(pen, x - 3, y1 + 2, x + 3, y1 - 2);
                g.DrawLine(pen, x - 3, y2 + 2, x + 3, y2 - 2);
                GraphicsState state = g.Save();
                try
                {
                    g.TranslateTransform(x + 10, (y1 + y2) * .5f); g.RotateTransform(-90);
                    SizeF size = g.MeasureString(text, font);
                    g.DrawString(text, font, brush, -size.Width * .5f, -size.Height * .5f);
                }
                finally { g.Restore(state); }
            }
        }

        static void RightText(Graphics g, string text, System.Drawing.Font font, Brush brush, float right, float y)
        { g.DrawString(text, font, brush, right - g.MeasureString(text, font).Width, y); }
        static System.Drawing.Font Font(float pixels, FontStyle style = FontStyle.Regular)
        { return new System.Drawing.Font("Microsoft YaHei UI", pixels, style, GraphicsUnit.Pixel); }
        // 标注保留毫米级读数（例如轴距3.815m），不可将已公开的轴距显示成3.82m。
        // 这只是标注格式，输入/回填/保存均独立保留double完整精度。
        static string Number(double x) { return x.ToString("0.000", CultureInfo.CurrentCulture); }
        static bool Finite(double x) { return !double.IsNaN(x) && !double.IsInfinity(x); }

        sealed class View
        {
            public readonly Graphics G;
            public readonly float Ground, Scale;
            readonly float left;
            public View(Graphics g, float x, float ground, float scale) { G = g; left = x; Ground = ground; Scale = scale; }
            public float X(double x) { return left + (float)(x * Scale); }
            public float Y(double y) { return Ground - (float)(y * Scale); }
            public PointF P(double x, double y) { return new PointF(X(x), Y(y)); }
            public RectangleF Rect(double x, double bottom, double width, double height)
            { return new RectangleF(X(x), Y(bottom + height), (float)(width * Scale), (float)(height * Scale)); }
        }
    }
}
