using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace TruckTurn
{
    /// <summary>
    /// 画路径前弹出的「货车选型」对话框。
    /// 尺寸输入采用米(m)，图纸单位可选「米」或「自动（随本图 INSUNITS）」。
    /// GB1589-2016 外廓尺寸/轴数限值作为合规提示（不强制拦截）。
    /// 覆盖绝大多数货车：长度 3.5~17.5 m、宽度 1.5~2.6 m、轴数 2~6，含微/轻/中/重卡与二/三轴半挂。
    /// v4.9.24：单位收敛（删毫米/厘米手动档）+ 刚性车前悬/后悬按真实比例（修 12m 车前悬 3m 失真）+ 预览造型再精修 + 预设下拉不再侵入预览区。
    /// </summary>
    public class TruckSizeForm : Form
    {
        private ComboBox cboPreset;
        private ComboBox cboUnits;       // 图纸单位：米 / 毫米 / 厘米
        private CheckBox chkArticulated;
        private TextBox txtTractorLen;   // 铰接：牵引车长(m)
        private TextBox txtMainLen;      // 主车长(m)：刚性=整车长，铰接=挂车长
        private TextBox txtWidth;        // 车宽(m)
        private TextBox txtAxles;        // 轴数
        private TextBox txtSteer;        // 前轮最大转角(°)
        private Label lblTractorLen;
        private Label lblHint;
        private Button btnOk;
        private Button btnCancel;

        // v4.4 新增：耦合几何与图层显示选项
        private TextBox txtKingpinToCabRear; // 鞍座到牵引车尾部距离(m)
        private TextBox txtTrailerKingpinToFront; // 鞍座到挂车前端距离(m)
        private TextBox txtMaxArticulation;      // 最大铰接角(°)
        private Label lblUnitWarn;  // v4.6.1：图纸单位与 INSUNITS 探测结果不一致时的告警行
        private CheckBox chkShowEnvelope;
        private CheckBox chkShowBody;
        private CheckBox chkShowWheels;
        private CheckBox chkShowGhost;
        private CheckBox chkShowFrontTrack;
        private CheckBox chkShowRadii;
        private ComboBox cboNodeVehicleDisplay;

        // v4.9.22：右侧实时侧面预览（v4.9.23 重绘）
        private DoubleBufferedPanel pnlPreview;

        /// <summary>确认后得到的车辆参数（单位与所选图纸单位一致）</summary>
        public VehicleParams Result { get; private set; }

        /// <summary>v4.6.1：由 INSUNITS 探测到的本图「1 米 = ? 图形单位」。0 = 探测不到，不给提示。</summary>
        private readonly double _drawingUnitScale;

        // 车型预设（内部单位米，显示时乘以 UnitScale 换算）。Articulated=false 为刚性单车。
        private class Preset
        {
            public bool Articulated;
            public double TractorLenM;   // 铰接用：牵引车长
            public double MainLenM;      // 刚性=整车长；铰接=挂车长
            public double WidthM;
            public int Axles;
            public double SteerDeg;
        }

        private static readonly Dictionary<string, Preset> Presets = new Dictionary<string, Preset>
        {
            // v4.9.24：前轮最大转角按实车重新标定。普通货车前轮最大转角约 40°、转向机械限位通常左右各 45°
            // （卡车之家/百科），旧的 18~25° 严重偏小，导致算出的最小转弯半径比实车大一圈。
            // 铰接车受挂车稳态铰接角限制（长挂车折不过来），取值须低于刚性车，详见使用说明。
            { "微型货车",   new Preset { Articulated=false, MainLenM=4.5,  WidthM=1.70, Axles=2, SteerDeg=40 } },
            { "轻型货车",   new Preset { Articulated=false, MainLenM=6.8,  WidthM=2.20, Axles=2, SteerDeg=40 } },
            { "中型货车",   new Preset { Articulated=false, MainLenM=9.6,  WidthM=2.50, Axles=3, SteerDeg=38 } },
            { "重型整车",   new Preset { Articulated=false, MainLenM=12.0, WidthM=2.55, Axles=4, SteerDeg=38 } },
            { "二轴半挂",   new Preset { Articulated=true,  TractorLenM=6.0, MainLenM=9.0,  WidthM=2.50, Axles=2, SteerDeg=20 } },
            { "三轴半挂",   new Preset { Articulated=true,  TractorLenM=6.0, MainLenM=13.0, WidthM=2.55, Axles=3, SteerDeg=18 } },
        };

        public TruckSizeForm(VehicleParams current) : this(current, 0.0) { }

        /// <param name="drawingUnitScale">本图 INSUNITS 推算出的「1 米 = ? 图形单位」，0 = 未知。</param>
        public TruckSizeForm(VehicleParams current, double drawingUnitScale)
        {
            current = current ?? VehicleParams.Defaults();
            _drawingUnitScale = drawingUnitScale;
            InitializeComponent();

            // 用当前/默认参数回填各输入框
            chkArticulated.Checked = current.Articulated;
            SetUnitByScale(current.UnitScale);
            txtTractorLen.Text = current.TractorLengthMeters().ToString("F1");
            // v4.6.1：铰接时「主车长」= 挂车长，必须用 TrailerLengthMeters()。
            // 之前误用 LengthMeters()（列车总长），每次开窗挂车就被多加一次牵引车长，
            // 参数在反复打开选型窗的过程中不断膨胀。
            txtMainLen.Text = (current.Articulated
                ? current.TrailerLengthMeters()
                : current.LengthMeters()).ToString("F1");
            txtWidth.Text = current.WidthMeters().ToString("F2");
            txtAxles.Text = current.AxleCount().ToString();
            txtSteer.Text = current.SteerAngleDeg.ToString("F0");
            txtKingpinToCabRear.Text = (current.KingpinToCabRear / current.UnitScale).ToString("F2");
            txtTrailerKingpinToFront.Text = (current.TrailerKingpinToFront / current.UnitScale).ToString("F2");
            txtMaxArticulation.Text = current.MaxArticulationAngleDeg.ToString("F0");
            chkShowEnvelope.Checked = current.ShowEnvelope;
            chkShowBody.Checked = current.ShowBody;
            chkShowWheels.Checked = current.ShowWheels;
            chkShowGhost.Checked = current.ShowGhost;
            chkShowFrontTrack.Checked = current.ShowFrontTrack;
            chkShowRadii.Checked = current.ShowRadii;
            cboNodeVehicleDisplay.SelectedIndex = current.ShowAllNodeVehicles ? 1 : 0;
            SyncArticulatedUI();
            UpdateHint();
            UpdateUnitWarn();
            RedrawPreview();
        }

        /// <summary>v4.6.1：图纸单位与本图 INSUNITS 不一致时立刻红字告警（不必等画完才发现看不见）。</summary>
        private void UpdateUnitWarn()
        {
            if (lblUnitWarn == null) return;
            if (_drawingUnitScale <= 0)
            {
                lblUnitWarn.Text = "";
                return;
            }
            double sel = GetSelectedScale();
            if (Math.Abs(sel - _drawingUnitScale) < 1e-6)
            {
                lblUnitWarn.ForeColor = Color.FromArgb(0, 120, 0);
                lblUnitWarn.Text = "与本图 INSUNITS 一致。";
                return;
            }
            double ratio = sel / _drawingUnitScale;
            lblUnitWarn.ForeColor = Color.FromArgb(180, 60, 60);
            lblUnitWarn.Text = string.Format(
                "本图 INSUNITS = {0}。当前选「{1}」，画出来的车会偏差 {2:F0} 倍！请改成「{0}」。",
                ScaleName(_drawingUnitScale), ScaleName(sel),
                ratio < 1 ? 1.0 / ratio : ratio);
        }

        private static string ScaleName(double scale)
        {
            if (Math.Abs(scale - 1.0) < 1e-6) return "米";
            if (Math.Abs(scale - 1000.0) < 1e-6) return "毫米";
            if (Math.Abs(scale - 100.0) < 1e-6) return "厘米";
            return scale.ToString("G", System.Globalization.CultureInfo.InvariantCulture) + " 单位/米";
        }

        // ---------------- v4.9.23 布局 ----------------
        // 左列（x=16）：车型预设 / 图纸单位 / 单位告警 / 铰接开关 / 8 行参数 / 提示
        // 右侧（x=330）：上部 452×330 侧面预览，下部 452×112 图层选项（两行三列）
        // 底部：确定 / 取消（右下角）
        private void InitializeComponent()
        {
            this.Text = "货车选型（米）";
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.StartPosition = FormStartPosition.CenterScreen;
            this.ClientSize = new Size(800, 540);

            int x = 16;          // 左列起始
            int tbX = 182;       // 输入框固定列（标签 AutoSize，不裁字）
            int tbW = 96;
            int lh = 22, gap = 30;
            int y = 16;

            var lbl = new Label { Text = "车型预设：", AutoSize = true, Left = x, Top = y + 2 };
            cboPreset = new ComboBox { Left = tbX, Top = y, Width = 140, Height = lh, DropDownStyle = ComboBoxStyle.DropDownList };
            cboPreset.Items.Add("自定义");
            foreach (var k in Presets.Keys) cboPreset.Items.Add(k);
            cboPreset.SelectedIndex = 0;
            cboPreset.SelectedIndexChanged += (s, e) => ApplyPreset();
            this.Controls.Add(lbl); this.Controls.Add(cboPreset);

            y += gap;
            var lblUnit = new Label { Text = "图纸单位：", AutoSize = true, Left = x, Top = y + 2 };
            // v4.9.24：单位只保留「米」+「自动（随本图 INSUNITS）」。
            // 毫米/厘米手动选项实用性极低（INSUNITS 自动探测已覆盖），删掉避免选错。
            cboUnits = new ComboBox { Left = tbX, Top = y, Width = 140, Height = lh, DropDownStyle = ComboBoxStyle.DropDownList };
            cboUnits.Items.Add("米");
            if (_drawingUnitScale > 0) cboUnits.Items.Add("自动（随本图 INSUNITS）");
            cboUnits.SelectedIndex = 0;
            cboUnits.SelectedIndexChanged += (s, e) => { cboPreset.SelectedIndex = 0; UpdateUnitWarn(); RedrawPreview(); };
            this.Controls.Add(lblUnit); this.Controls.Add(cboUnits);

            // v4.9.23：单位告警收在左列内部（原先横排 270px 会压到右侧预览）
            lblUnitWarn = new Label
            {
                Left = x, Top = y + 26, Width = 300, Height = 40,
                ForeColor = Color.FromArgb(180, 60, 60)
            };
            this.Controls.Add(lblUnitWarn);

            y = 118;
            chkArticulated = new CheckBox { Text = "铰接半挂车（不勾=刚性单车）", AutoSize = true, Left = x, Top = y };
            chkArticulated.CheckedChanged += (s, e) => { SyncArticulatedUI(); UpdateHint(); RedrawPreview(); };
            this.Controls.Add(chkArticulated);

            y = 150;
            lblTractorLen = new Label { Text = "牵引车长(m)：", AutoSize = true, Left = x, Top = y + 2 };
            txtTractorLen = new TextBox { Left = tbX, Top = y, Width = tbW, Height = lh };
            txtTractorLen.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; UpdateHint(); RedrawPreview(); };
            this.Controls.Add(lblTractorLen); this.Controls.Add(txtTractorLen);

            y += gap;
            AddRow("主车长(m)：", ref txtMainLen, y);
            txtMainLen.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; UpdateHint(); RedrawPreview(); };

            y += gap;
            AddRow("车宽(m)：", ref txtWidth, y);
            txtWidth.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; UpdateHint(); RedrawPreview(); };

            y += gap;
            AddRow("轴数：", ref txtAxles, y);
            txtAxles.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; UpdateHint(); RedrawPreview(); };

            y += gap;
            AddRow("前轮最大转角(°)：", ref txtSteer, y);
            txtSteer.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; RedrawPreview(); };

            y += gap;
            AddRow("驾驶室后壁到鞍座(m)：", ref txtKingpinToCabRear, y);
            txtKingpinToCabRear.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; RedrawPreview(); };

            y += gap;
            AddRow("挂车前端到鞍座(m)：", ref txtTrailerKingpinToFront, y);
            txtTrailerKingpinToFront.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; RedrawPreview(); };

            y += gap;
            AddRow("最大铰接角(°)：", ref txtMaxArticulation, y);
            txtMaxArticulation.TextChanged += (s, e) => { cboPreset.SelectedIndex = 0; RedrawPreview(); };

            y += gap + 4;
            lblHint = new Label { Left = x, Top = y, Width = 300, Height = 64, ForeColor = Color.FromArgb(0, 120, 0) };
            this.Controls.Add(lblHint);

            // v4.6.1：DialogResult 必须是 None，由 Click 里校验通过后再置 OK。
            // 之前 btnOk.DialogResult 直接写成 OK —— WinForms 会在 Click 处理器之后照样把
            // 窗体的 DialogResult 置成 OK 并关闭窗体，于是 BuildResult() 校验失败（return false）
            // 也拦不住关闭，Result 仍为 null，调用方拿到 null 参数，
            // 后续 Jig 里一访问字段就 NullReferenceException。
            btnOk = new Button { Text = "确定", Left = 560, Top = 498, Width = 90, Height = 30, DialogResult = DialogResult.None };
            btnCancel = new Button { Text = "取消", Left = 660, Top = 498, Width = 90, Height = 30, DialogResult = DialogResult.Cancel };
            btnOk.Click += (s, e) => { if (BuildResult()) this.DialogResult = DialogResult.OK; };
            this.Controls.Add(btnOk); this.Controls.Add(btnCancel);
            this.AcceptButton = btnOk;
            this.CancelButton = btnCancel;

            // v4.9.23：右上侧面预览
            pnlPreview = new DoubleBufferedPanel
            {
                Left = 330,
                Top = 16,
                Width = 452,
                Height = 330,
                BorderStyle = BorderStyle.FixedSingle,
                BackColor = Color.White
            };
            pnlPreview.Paint += (s, e) => DrawSideProfile(e.Graphics);
            this.Controls.Add(pnlPreview);

            // v4.9.23：图层选项改两行三列，放在预览正下方（原先 180×220 竖排与预览/窗底冲突）
            var grpLayers = new GroupBox { Text = "CAD 输出", Left = 330, Top = 356, Width = 452, Height = 134 };
            AddLayerCheck(grpLayers, ref chkShowEnvelope, "包络", 18, 26);
            AddLayerCheck(grpLayers, ref chkShowBody, "车体", 18, 62);
            AddLayerCheck(grpLayers, ref chkShowWheels, "轮径", 168, 26);
            AddLayerCheck(grpLayers, ref chkShowGhost, "姿态（中间帧）", 168, 62);
            AddLayerCheck(grpLayers, ref chkShowFrontTrack, "前轴轨迹", 318, 26);
            AddLayerCheck(grpLayers, ref chkShowRadii, "转弯半径", 318, 62);
            var lblNodeVehicleDisplay = new Label
            {
                Text = "节点车辆：", AutoSize = true, Left = 18, Top = 100
            };
            cboNodeVehicleDisplay = new ComboBox
            {
                Left = 94, Top = 95, Width = 190,
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            cboNodeVehicleDisplay.Items.AddRange(new object[] { "仅首尾", "全部节点" });
            cboNodeVehicleDisplay.SelectedIndex = 0;
            grpLayers.Controls.Add(lblNodeVehicleDisplay);
            grpLayers.Controls.Add(cboNodeVehicleDisplay);
            this.Controls.Add(grpLayers);
        }

        private void AddLayerCheck(GroupBox grp, ref CheckBox chk, string text, int lx, int ly)
        {
            chk = new CheckBox { Text = text, AutoSize = true, Left = lx, Top = ly };
            grp.Controls.Add(chk);
        }

        private void AddRow(string label, ref TextBox tb, int y)
        {
            var l = new Label { Text = label, AutoSize = true, Left = 16, Top = y + 2 };
            tb = new TextBox { Left = 182, Top = y, Width = 96, Height = 22 };
            this.Controls.Add(l); this.Controls.Add(tb);
        }

        private void SyncArticulatedUI()
        {
            bool art = chkArticulated.Checked;
            lblTractorLen.Enabled = art;
            txtTractorLen.Enabled = art;
            lblTractorLen.Text = art ? "牵引车长(m)：" : "整车长(m)：";
            txtTrailerKingpinToFront.Enabled = art;
            txtKingpinToCabRear.Enabled = art;
            txtMaxArticulation.Enabled = art;
        }

        private void ApplyPreset()
        {
            if (cboPreset.SelectedIndex <= 0) return; // 自定义
            var name = cboPreset.Text;
            if (!Presets.TryGetValue(name, out var ps)) return;
            chkArticulated.Checked = ps.Articulated;
            txtTractorLen.Text = ps.TractorLenM.ToString("F1");
            txtMainLen.Text = ps.MainLenM.ToString("F1");
            txtWidth.Text = ps.WidthM.ToString("F2");
            txtAxles.Text = ps.Axles.ToString();
            txtSteer.Text = ps.SteerDeg.ToString("F0");
            // v4.4 预设默认 kingpin 间隙与最大铰接角
            txtKingpinToCabRear.Text = (ps.Articulated ? 0.21 * ps.TractorLenM : 0.0).ToString("F2");
            txtTrailerKingpinToFront.Text = (ps.Articulated ? 0.03 * ps.MainLenM : 0.0).ToString("F2");
            txtMaxArticulation.Text = "90";
            SyncArticulatedUI();
            UpdateHint();
            RedrawPreview();
        }

        private void UpdateHint()
        {
            var msgs = new List<string>();
            if (!double.TryParse(txtWidth.Text, out double wM)) wM = 0;
            if (wM > 2.55 && wM <= 2.60) msgs.Add("车宽超 2.55m（冷藏车限值 2.6m，需确认）");
            else if (wM > 2.60) msgs.Add("车宽超过 GB1589 限值 2.55m");

            if (chkArticulated.Checked)
            {
                if (!double.TryParse(txtTractorLen.Text, out double tM)) tM = 0;
                if (!double.TryParse(txtMainLen.Text, out double trM)) trM = 0;
                if ((tM + trM) > 17.1) msgs.Add("铰接列车总长超过 17.1m 限值");
            }
            else
            {
                if (!double.TryParse(txtMainLen.Text, out double lM)) lM = 0;
                if (lM > 12.0) msgs.Add("单车长超过 12m 限值");
            }

            if (msgs.Count == 0) { lblHint.Text = "符合 GB1589-2016 常见限值，可放心使用。"; lblHint.ForeColor = Color.FromArgb(0, 120, 0); }
            else { lblHint.Text = "提示：" + string.Join("；", msgs) + "。仍可继续（仅提示）。"; lblHint.ForeColor = Color.FromArgb(180, 90, 0); }
        }

        /// <summary>v4.9.22：根据当前界面输入构造一个仅用于预览的 VehicleParams（UnitScale=1，单位米）。</summary>
        private VehicleParams BuildPreviewParams()
        {
            var p = new VehicleParams { UnitScale = 1.0 };
            p.Articulated = chkArticulated.Checked;
            p.SteerAngleDeg = ParseD(txtSteer.Text, 18);
            p.TurnAngleDeg = 90;
            p.MaxArticulationAngleDeg = Math.Max(10.0, ParseD(txtMaxArticulation.Text, 90.0));

            double wM = ParseD(txtWidth.Text, 2.5);
            double trackM = Math.Max(0.9, wM * 0.75);
            p.WheelTrack = trackM;
            p.TractorWidth = wM;
            p.TrailerWidth = wM;

            if (p.Articulated)
            {
                double tractorM = ParseD(txtTractorLen.Text, 6.0);
                double trailerM = ParseD(txtMainLen.Text, 11.0);
                double kpCabRearM = Math.Max(0.0, ParseD(txtKingpinToCabRear.Text, 0.21 * tractorM));
                double kpTrailerFrontM = Math.Max(0.0, ParseD(txtTrailerKingpinToFront.Text, 0.03 * trailerM));
                p.KingpinToCabRear = kpCabRearM;
                p.TrailerKingpinToFront = kpTrailerFrontM;
                p.ChassisWidth = Math.Min(wM, 1.0);
                p.CabClearance = 0.1;
                p.ChassisRearOverhang = 0.15 * tractorM;
                p.TractorFrontOverhang = 0.25 * tractorM;
                p.TractorWheelbase = 0.60 * tractorM;
                p.TractorRearToKingpin = 0.10 * tractorM;
                double tre = 0.12 * trailerM;
                p.TrailerRearOverhang = tre;
                p.TrailerKingpinToRearAxle = Math.Max(1.0, trailerM - tre - p.TrailerKingpinToFront);
                p.TrailerAxles = (int)Math.Max(1, Math.Min(3, ParseD(txtAxles.Text, 3)));
                p.RigidRearOverhang = 0;
                p.RigidAxles = 2;
            }
            else
            {
                double totalM = ParseD(txtMainLen.Text, 8.0);
                RigidLayoutM(totalM, out double foM, out double wbM, out double roM);
                p.TractorFrontOverhang = foM;
                p.TractorWheelbase = wbM;
                p.TractorRearToKingpin = 0;
                p.RigidRearOverhang = roM;
                p.RigidAxles = (int)Math.Max(2, Math.Min(6, ParseD(txtAxles.Text, 2)));
                p.TrailerKingpinToRearAxle = 0;
                p.TrailerRearOverhang = 0;
                p.TrailerAxles = 3;
                p.KingpinToCabRear = 0;
                p.TrailerKingpinToFront = 0;
                p.ChassisRearOverhang = 0;
                p.ChassisWidth = wM;
                p.CabClearance = 0;
            }
            return p;
        }

        /// <summary>v4.9.22：触发侧面预览面板重绘。</summary>
        private void RedrawPreview()
        {
            if (pnlPreview != null) pnlPreview.Invalidate();
        }

        /// <summary>v5.0：未选型（自定义且关键尺寸为 0/未填）时预览不画车，只显示引导文字。</summary>
        private bool PreviewInputEmpty()
        {
            double w = ParseD(txtWidth.Text, 0);
            if (w <= 0.1) return true;
            if (chkArticulated.Checked)
                return ParseD(txtTractorLen.Text, 0) <= 0.1 || ParseD(txtMainLen.Text, 0) <= 0.1;
            return ParseD(txtMainLen.Text, 0) <= 0.1;
        }

        // ================= v4.9.23 侧面预览绘制 =================
        // 车头朝左（与工程图参考一致），以米为单位按面板尺寸自适应缩放。
        // 配色：车身浅灰蓝 / 描边深灰 / 玻璃淡蓝 / 轮胎深灰 / 尺寸线中灰。
        private void DrawSideProfile(Graphics g)
        {
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;
            g.Clear(Color.White);

            // v5.0：未选型时空白态——居中引导文字，不画乱比例的车
            if (PreviewInputEmpty())
            {
                using (var hintFont = new Font(this.Font.FontFamily, 10.5f, FontStyle.Regular))
                using (var sf = new StringFormat { Alignment = StringAlignment.Center, LineAlignment = StringAlignment.Center })
                {
                    using (var b = new SolidBrush(Color.FromArgb(150, 158, 166)))
                        g.DrawString("请先在左侧选择「车型预设」\n或输入车辆尺寸后查看预览",
                            hintFont, b, new RectangleF(0, 0, pnlPreview.Width, pnlPreview.Height), sf);
                }
                return;
            }

            VehicleParams p;
            try { p = BuildPreviewParams(); }
            catch { return; }
            if (p == null) return;

            double L;
            if (p.Articulated)
                L = (p.TractorFrontOverhang + p.TractorWheelbase + p.TractorRearToKingpin)
                  + (p.TrailerKingpinToFront + p.TrailerKingpinToRearAxle + p.TrailerRearOverhang);
            else
                L = p.TractorFrontOverhang + p.TractorWheelbase + p.RigidRearOverhang;
            if (double.IsNaN(L) || double.IsInfinity(L) || L < 1.0) L = 1.0;

            int pad = 20;
            int availW = Math.Max(60, pnlPreview.Width - 2 * pad);
            float groundY = pnlPreview.Height - pad - 14;
            // 宽度：车长 + 左右各 0.6m 余量；高度：车高 3.5m + 顶部尺寸线 ~54px
            double scaleW = availW / (L + 1.2);
            double scaleH = (groundY - pad - 54) / 3.8;
            double scale = Math.Min(scaleW, scaleH);
            if (scale < 4) scale = 4;

            float xL = (float)((pnlPreview.Width - L * scale) / 2.0); // 车头（左）
            float xR = xL + (float)(L * scale);                       // 车尾（右）

            using (var dimFont = new Font(this.Font.FontFamily, 8.25f, FontStyle.Regular))
            {
                if (p.Articulated)
                    DrawArticulatedSide(g, p, (float)scale, groundY, xL, xR, dimFont);
                else
                    DrawRigidSide(g, p, (float)scale, groundY, xL, xR, dimFont);
            }
        }

        // ---- 通用元素 ----

        private void DrawGround(Graphics g, float x1, float x2, float groundY)
        {
            using (var pen = new Pen(Color.FromArgb(168, 175, 181), 1f))
                g.DrawLine(pen, x1, groundY, x2, groundY);
        }

        private void DrawWheel(Graphics g, float cx, float groundY, double rM, double scale)
        {
            float r = (float)(rM * scale);
            if (r < 3.5f) r = 3.5f;
            float cy = groundY - r;
            using (var tire = new SolidBrush(Color.FromArgb(52, 59, 65)))
            using (var rim = new SolidBrush(Color.FromArgb(207, 213, 218)))
            using (var hub = new SolidBrush(Color.FromArgb(126, 136, 144)))
            using (var pen = new Pen(Color.FromArgb(39, 46, 52), 1.05f))
            {
                g.FillEllipse(tire, cx - r, cy - r, r * 2, r * 2);
                g.DrawEllipse(pen, cx - r, cy - r, r * 2, r * 2);
                float rr = r * 0.50f;
                g.FillEllipse(rim, cx - rr, cy - rr, rr * 2, rr * 2);
                g.DrawEllipse(pen, cx - rr, cy - rr, rr * 2, rr * 2);
                float hr = r * 0.13f;
                g.FillEllipse(hub, cx - hr, cy - hr, hr * 2, hr * 2);
            }
        }

        /// <summary>驾驶室轮廓（车头朝左）。只保留外廓、玻璃、车门与前灯四个识别层级。</summary>
        private void DrawCab(Graphics g, double scale, float groundY,
            float cabFrontX, float cabRearX, double topM)
        {
            float s = (float)scale;
            float Y(double m) { return groundY - (float)(m * s); }
            float x0 = cabFrontX, x1 = cabRearX;
            if (x1 - x0 < 0.6f * s) x1 = x0 + 0.6f * s;
            float tm = Y(topM);

            using (var fill = new SolidBrush(Color.FromArgb(238, 243, 247)))
            using (var lower = new SolidBrush(Color.FromArgb(207, 216, 223)))
            using (var glass = new SolidBrush(Color.FromArgb(190, 215, 229)))
            using (var lamp = new SolidBrush(Color.FromArgb(207, 170, 89)))
            using (var pen = new Pen(Color.FromArgb(51, 68, 80), 1.55f))
            using (var thin = new Pen(Color.FromArgb(125, 139, 149), 0.9f))
            {
                var poly = new[]
                {
                    new PointF(x0 + 0.03f*s, Y(0.72f)),
                    new PointF(x0,           Y(1.25f)),
                    new PointF(x0 + 0.12f*s, Y(2.20f)),
                    new PointF(x0 + 0.50f*s, tm),
                    new PointF(x1 - 0.12f*s, tm),
                    new PointF(x1,           tm + 0.13f*s),
                    new PointF(x1,           Y(0.72f)),
                };
                g.FillPolygon(fill, poly);
                g.DrawPolygon(pen, poly);

                g.FillRectangle(lower, x0 + 0.03f*s, Y(0.92f),
                    x1 - x0 - 0.03f*s, 0.20f*s);

                // 一体式玻璃面，以一条立柱区分前风挡和侧窗。
                var window = new[]
                {
                    new PointF(x0 + 0.16f*s, Y(2.18f)),
                    new PointF(x0 + 0.52f*s, tm + 0.09f*s),
                    new PointF(x1 - 0.17f*s, tm + 0.09f*s),
                    new PointF(x1 - 0.17f*s, Y(2.18f)),
                };
                g.FillPolygon(glass, window);
                g.DrawPolygon(pen, window);
                float pillarX = x0 + 0.52f*s;
                g.DrawLine(pen, pillarX, tm + 0.09f*s, pillarX, Y(2.18f));
                float doorX = Math.Min(x1 - 0.16f*s, x0 + 0.62f*s);
                g.DrawLine(thin, doorX, Y(0.76f), doorX, Y(2.12f));
                g.DrawLine(thin, doorX + 0.08f*s, Y(1.80f),
                    Math.Min(x1 - 0.08f*s, doorX + 0.24f*s), Y(1.80f));
                g.FillRectangle(lamp, x0 + 0.02f*s, Y(1.35f), 0.12f*s, 0.09f*s);
            }
        }

        /// <summary>货厢：克制的工程轮廓与少量面板分格。</summary>
        private void DrawCargoBox(Graphics g, double scale, float groundY,
            float boxFrontX, float boxRearX, double bottomM, double topM)
        {
            float s = (float)scale;
            float bx = boxFrontX, bw = boxRearX - boxFrontX;
            if (bw < 4) return;
            float bt = groundY - (float)(topM * s);
            float bb = groundY - (float)(bottomM * s);

            using (var fill = new SolidBrush(Color.FromArgb(246, 247, 248)))
            using (var seam = new Pen(Color.FromArgb(214, 219, 223), 0.9f))
            using (var pen = new Pen(Color.FromArgb(61, 75, 85), 1.5f))
            {
                var r = new RectangleF(bx, bt, bw, bb - bt);
                g.FillRectangle(fill, r);
                for (int k = 1; k < 4; k++)
                {
                    float rx = bx + bw * k / 4f;
                    g.DrawLine(seam, rx, bt + 2, rx, bb - 2);
                }
                g.DrawRectangle(pen, r.X, r.Y, r.Width, r.Height);
            }
        }

        /// <summary>底盘纵梁。</summary>
        private void DrawChassis(Graphics g, double scale, float groundY, float x1, float x2)
        {
            float s = (float)scale;
            var r = new RectangleF(x1, groundY - 1.05f * s, x2 - x1, 0.32f * s);
            if (r.Width < 2) return;
            using (var fill = new SolidBrush(Color.FromArgb(91, 105, 115)))
            using (var pen = new Pen(Color.FromArgb(55, 69, 78), 1.1f))
            {
                g.FillRectangle(fill, r);
                g.DrawRectangle(pen, r.X, r.Y, r.Width, r.Height);
            }
        }

        /// <summary>水平尺寸线（双箭头 + 端点刻度 + 上方文字）。</summary>
        private void DrawHDim(Graphics g, Font font, float x1, float x2, float y, string text)
        {
            if (x2 < x1) { float t = x1; x1 = x2; x2 = t; }
            if (x2 - x1 < 6f) return;
            using (var pen = new Pen(Color.FromArgb(140, 148, 155), 1f))
            using (var brush = new SolidBrush(Color.FromArgb(70, 78, 84)))
            {
                g.DrawLine(pen, x1, y, x2, y);
                g.DrawLine(pen, x1, y - 4, x1, y + 4);
                g.DrawLine(pen, x2, y - 4, x2, y + 4);
                g.DrawLine(pen, x1, y, x1 + 6, y - 2.6f);
                g.DrawLine(pen, x1, y, x1 + 6, y + 2.6f);
                g.DrawLine(pen, x2, y, x2 - 6, y - 2.6f);
                g.DrawLine(pen, x2, y, x2 - 6, y + 2.6f);

                var sz = g.MeasureString(text, font);
                g.DrawString(text, font, brush, (x1 + x2) / 2f - sz.Width / 2f, y - sz.Height - 2f);
            }
        }

        private static string Meters(double m) { return string.Format("{0:F2} m", m); }

        // ---- 刚性单车（车头朝左） ----
        private void DrawRigidSide(Graphics g, VehicleParams p, float s, float groundY, float xL, float xR, Font dimFont)
        {
            double foM = p.TractorFrontOverhang;   // 前悬
            double wbM = p.TractorWheelbase;       // 轴距
            double roM = p.RigidRearOverhang;      // 后悬

            float xFA = xL + (float)(foM * s);     // 前轴
            float xRA = xFA + (float)(wbM * s);    // 后轴（基准）
            double cabRearM = foM + 0.60;          // 驾驶室后壁（前轴后 0.6m）
            float xCabRear = xL + (float)(cabRearM * s);

            // 货厢（驾驶室后壁 + 0.12m 缝 → 车尾）
            DrawCargoBox(g, s, groundY, xCabRear + 0.12f * s, xR, 1.10, 3.50);
            // 底盘
            DrawChassis(g, s, groundY, xL + 0.10f * s, xR);
            // 驾驶室
            DrawCab(g, s, groundY, xL, xCabRear, 3.20);
            // 油箱（轴距足够时）
            if (wbM > 3.6)
            {
                float tx1 = xFA + 0.9f * s, tx2 = tx1 + 1.0f * s;
                if (tx2 < xRA - 0.5f * s)
                {
                    using (var fill = new SolidBrush(Color.FromArgb(183, 192, 201)))
                    using (var pen = new Pen(Color.FromArgb(55, 71, 79), 1.1f))
                    {
                        var r = new RectangleF(tx1, groundY - 0.75f * s, tx2 - tx1, 0.30f * s);
                        g.FillRectangle(fill, r);
                        g.DrawRectangle(pen, r.X, r.Y, r.Width, r.Height);
                    }
                }
            }
            // 车轮：前轴 + 后轴组（多轴向前排）
            int axles = Math.Max(2, p.RigidAxles);
            DrawWheel(g, xFA, groundY, 0.52, s);
            for (int i = 0; i < axles - 1; i++)
                DrawWheel(g, xRA - (float)(1.35 * i * s), groundY, 0.52, s);

            // 地面
            DrawGround(g, xL - 0.3f * s, xR + 0.3f * s, groundY);

            // 尺寸：顶部总长；底部 前悬/轴距/后悬
            float yTop = groundY - 3.55f * s - 30;
            DrawHDim(g, dimFont, xL, xR, yTop, Meters(foM + wbM + roM));
            float yBot = groundY + 18;
            if (foM * s > 46) DrawHDim(g, dimFont, xL, xFA, yBot, Meters(foM));
            if (wbM * s > 46) DrawHDim(g, dimFont, xFA, xRA, yBot, "轴距 " + Meters(wbM));
            if (roM * s > 46) DrawHDim(g, dimFont, xRA, xR, yBot, Meters(roM));
        }

        // ---- 铰接半挂（车头朝左） ----
        private void DrawArticulatedSide(Graphics g, VehicleParams p, float s, float groundY, float xL, float xR, Font dimFont)
        {
            double foM = p.TractorFrontOverhang;
            double wbM = p.TractorWheelbase;
            double akM = p.TractorRearToKingpin;
            double tfM = p.TrailerKingpinToFront;
            double laM = p.TrailerKingpinToRearAxle;
            double ro2M = p.TrailerRearOverhang;

            float xTA = xR - (float)(ro2M * s);        // 挂车后轴（最右轴）
            float xKp = xTA - (float)(laM * s);        // 鞍座
            float xTrF = xKp - (float)(tfM * s);       // 挂车前端
            float xTRA = xKp - (float)(akM * s);       // 牵引车后轴
            float xTFA = xTRA - (float)(wbM * s);      // 牵引车前轴
            float xCabRear = xTFA + 0.60f * s;         // 驾驶室后壁

            // 挂车货厢（先画，之后牵引车底盘压在其上视觉正确）
            DrawCargoBox(g, s, groundY, xTrF + 0.05f * s, xR, 1.20, 3.55);
            // 牵引车底盘 + 驾驶室
            DrawChassis(g, s, groundY, xL + 0.10f * s, xKp + 0.30f * s);
            DrawCab(g, s, groundY, xL, xCabRear, 3.20);
            // 鞍座（第五轮）
            using (var fw = new SolidBrush(Color.FromArgb(70, 82, 92)))
            {
                var r = new RectangleF(xKp - 0.38f * s, groundY - 1.18f * s, 0.76f * s, 0.16f * s);
                g.FillRectangle(fw, r);
            }
            // 支腿（鞍座与挂车后轴之间）
            float legX = xKp + 0.55f * s;
            if (legX < xTA - 0.8f * s)
            {
                using (var pen = new Pen(Color.FromArgb(90, 101, 112), 2f))
                {
                    g.DrawLine(pen, legX, groundY - 1.15f * s, legX, groundY - 0.35f * s);
                    g.DrawLine(pen, legX - 0.14f * s, groundY - 0.35f * s, legX + 0.14f * s, groundY - 0.35f * s);
                }
            }
            // 车轮：牵引车前轴 / 后轴；挂车 1~3 轴（向前排）
            DrawWheel(g, xTFA, groundY, 0.52, s);
            DrawWheel(g, xTRA, groundY, 0.52, s);
            for (int i = 0; i < Math.Max(1, p.TrailerAxles); i++)
                DrawWheel(g, xTA - (float)(1.35 * i * s), groundY, 0.52, s);

            // 地面
            DrawGround(g, xL - 0.3f * s, xR + 0.3f * s, groundY);

            // 尺寸：顶部两行（列车总长 / 牵引车+挂车分段）；底部（前悬 / 轴距 / 鞍座-挂车后轴）
            float yTop1 = groundY - 3.60f * s - 26;
            float yTop2 = yTop1 - 20;
            DrawHDim(g, dimFont, xL, xR, yTop2, "列车总长 " + Meters((foM + wbM + akM) + (tfM + laM + ro2M)));
            DrawHDim(g, dimFont, xL, xKp, yTop1, "牵引车 " + Meters(foM + wbM + akM));
            DrawHDim(g, dimFont, xTrF, xR, yTop1, "挂车 " + Meters(tfM + laM + ro2M));
            float yBot = groundY + 18;
            if (foM * s > 46) DrawHDim(g, dimFont, xL, xTFA, yBot, Meters(foM));
            if (wbM * s > 46) DrawHDim(g, dimFont, xTFA, xTRA, yBot, "轴距 " + Meters(wbM));
            if (laM * s > 46) DrawHDim(g, dimFont, xKp, xTA, yBot, "鞍座-后轴 " + Meters(laM));
        }

        /// <summary>v4.9.22：开启双缓冲的 Panel，避免预览重绘闪烁。</summary>
        private class DoubleBufferedPanel : Panel
        {
            public DoubleBufferedPanel()
            {
                this.DoubleBuffered = true;
                this.ResizeRedraw = true;
            }
        }

        /// <summary>
        /// v4.9.24：刚性车纵向布局（米）。前悬 1.1~1.5m、后悬 0.8~2.6m，轴距吃剩余长度。
        /// 旧版前悬 = 0.25×车长（12m 重卡算出 3m）严重失真——真车前悬只有约 1.2~1.5m。
        /// </summary>
        private static void RigidLayoutM(double totalM, out double fo, out double wb, out double ro)
        {
            fo = Math.Min(1.5, Math.Max(1.1, 0.12 * totalM));
            ro = Math.Min(2.6, Math.Max(0.8, 0.20 * totalM));
            wb = Math.Max(1.2, totalM - fo - ro);
        }

        private bool BuildResult()
        {
            var p = new VehicleParams();
            p.Articulated = chkArticulated.Checked;
            p.SteerAngleDeg = ParseD(txtSteer.Text, 18);
            p.TurnAngleDeg = 90;
            p.UnitScale = GetSelectedScale();

            double scale = p.UnitScale;
            double wM = ParseD(txtWidth.Text, 2.5);
            double wDrawing = wM * scale;
            double trackM = Math.Max(0.9, wM * 0.75);   // 物理轮距 ≈ 车宽*0.75，至少 0.9m
            p.WheelTrack = trackM * scale;

            // v4.4 图层显示选项
            p.ShowEnvelope = chkShowEnvelope.Checked;
            p.ShowBody = chkShowBody.Checked;
            p.ShowWheels = chkShowWheels.Checked;
            p.ShowGhost = chkShowGhost.Checked;
            p.ShowFrontTrack = chkShowFrontTrack.Checked;
            p.ShowRadii = chkShowRadii.Checked;
            p.ShowAllNodeVehicles = cboNodeVehicleDisplay.SelectedIndex == 1;

            // v4.4 最大铰接角（仅铰接有效，刚性忽略）
            p.MaxArticulationAngleDeg = Math.Max(10.0, ParseD(txtMaxArticulation.Text, 90.0));

            if (p.Articulated)
            {
                double tractorM = ParseD(txtTractorLen.Text, 6.0);
                double trailerM = ParseD(txtMainLen.Text, 11.0);
                double tractorDrawing = tractorM * scale;
                double trailerDrawing = trailerM * scale;

                // v4.4：驾驶室后壁到鞍座、挂车前端到鞍座
                double kpCabRearM = Math.Max(0.0, ParseD(txtKingpinToCabRear.Text, 0.21 * tractorM));
                double kpTrailerFrontM = Math.Max(0.0, ParseD(txtTrailerKingpinToFront.Text, 0.03 * trailerM));
                p.KingpinToCabRear = kpCabRearM * scale;
                p.TrailerKingpinToFront = kpTrailerFrontM * scale;
                p.ChassisWidth = Math.Min(wDrawing, 1.0 * scale);   // 车架纵梁约 1m
                p.CabClearance = 0.1 * scale;

                // 牵引车纵向布局（以后轴为原点，向前为正）：
                //   车架尾端 = -ChassisRearOverhang，鞍座 = +后轴到鞍座，驾驶室后壁 = 鞍座 + KingpinToCabRear，
                //   车头 = 轴距 + 前悬。四段之和 = 牵引车总长。
                p.ChassisRearOverhang = 0.15 * tractorDrawing;
                p.TractorFrontOverhang = 0.25 * tractorDrawing;
                p.TractorWheelbase = 0.60 * tractorDrawing;
                p.TractorRearToKingpin = 0.10 * tractorDrawing;
                p.TractorWidth = wDrawing;

                // 守卫：驾驶室后壁必须留在车头之前，且驾驶室至少占轴距的 25%。
                // 安全下限由 CabRearX() 依据挂车前角最大摆幅反推，两者冲突说明车太短/挂车太宽。
                double frontX = p.TractorWheelbase + p.TractorFrontOverhang;
                double cabRearLimit = frontX - 0.25 * p.TractorWheelbase;
                if (p.CabRearX() > cabRearLimit)
                {
                    p.KingpinToCabRear = Math.Max(0.0, cabRearLimit - p.TractorRearToKingpin);
                    if (p.CabRearX() > cabRearLimit)
                    {
                        MessageBox.Show(this, "牵引车过短或挂车过宽：按当前尺寸，挂车转弯时无法避开驾驶室。请加长牵引车或减小车宽。",
                                        "货车选型", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        return false;
                    }
                }

                // v4.6.1：守卫用到的 TrailerRearOverhang 必须在守卫之前算出来。
                // 之前 p.TrailerRearOverhang 还是 new VehicleParams() 的初始值 0，
                // 这条「挂车前端到鞍座 + 后悬 > 挂车总长」的校验形同虚设。
                double tre = 0.12 * trailerDrawing;
                p.TrailerRearOverhang = tre;
                double trailerGap = p.TrailerKingpinToFront + p.TrailerRearOverhang;
                if (trailerGap >= trailerDrawing - 1e-3)
                {
                    MessageBox.Show(this, "挂车前端到鞍座距离+后悬过大，已超过挂车总长。", "货车选型", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return false;
                }
                p.TrailerKingpinToRearAxle = Math.Max(scale, trailerDrawing - tre - p.TrailerKingpinToFront);
                p.TrailerWidth = wDrawing;
                p.TrailerAxles = (int)Math.Max(1, Math.Min(3, ParseD(txtAxles.Text, 3)));
                p.RigidRearOverhang = 0;
                p.RigidAxles = 2;
            }
            else
            {
                double totalM = ParseD(txtMainLen.Text, 8.0);
                double totalDrawing = totalM * scale;
                RigidLayoutM(totalM, out double foM, out double wbM, out double roM);
                p.TractorFrontOverhang = foM * scale;
                p.TractorWheelbase = wbM * scale;
                p.TractorRearToKingpin = 0;
                p.TractorWidth = wDrawing;
                p.RigidRearOverhang = roM * scale;
                p.RigidAxles = (int)Math.Max(2, Math.Min(6, ParseD(txtAxles.Text, 2)));
                p.TrailerKingpinToRearAxle = 0;
                p.TrailerRearOverhang = 0;
                p.TrailerWidth = wDrawing;
                p.TrailerAxles = 3;
                p.KingpinToCabRear = 0;
                p.TrailerKingpinToFront = 0;
                p.ChassisRearOverhang = 0;
                p.ChassisWidth = wDrawing;
                p.CabClearance = 0;
            }

            // 合理性守卫：轴距/长必须为正，否则无法计算转弯半径
            if (p.TractorWheelbase <= 1e-6)
            {
                MessageBox.Show(this, "车长/轴距无效，请检查输入。", "货车选型", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return false;
            }
            Result = p;
            return true;
        }

        /// <summary>当前下拉选中的「1 米 = ? CAD 单位」</summary>
        private double GetSelectedScale()
        {
            switch (cboUnits.SelectedIndex)
            {
                case 0: return 1.0;      // 米
                case 1: return _drawingUnitScale > 0 ? _drawingUnitScale : 1.0; // 自动（随本图 INSUNITS）
                default: return 1.0;
            }
        }

        private void SetUnitByScale(double scale)
        {
            // v4.9.24：只有「米」与「自动」两项。探测到图纸单位且与回填比例一致 → 自动；
            // 旧图存了毫米/厘米但探测不到时，回落到「米」。
            bool hasAuto = _drawingUnitScale > 0 && cboUnits.Items.Count > 1;
            if (hasAuto && Math.Abs(scale - _drawingUnitScale) < 1e-6) cboUnits.SelectedIndex = 1;
            else cboUnits.SelectedIndex = 0;
        }

        private static double ParseD(string s, double def)
        {
            return double.TryParse(s, System.Globalization.NumberStyles.Any,
                System.Globalization.CultureInfo.InvariantCulture, out var v) ? v : def;
        }
    }
}
