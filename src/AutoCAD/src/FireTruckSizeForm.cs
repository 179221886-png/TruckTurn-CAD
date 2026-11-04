using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.Windows.Forms;

namespace TruckTurn
{
    /// <summary>独立消防车选型：两轴前桥模型；未取得的参数不会显示为厂家认证值。</summary>
    public sealed class FireTruckSizeForm : Form
    {
        private readonly VehicleParams _current;
        private readonly double _drawingScale;
        private readonly IList<FireVehicleProfile> _profiles;
        private readonly List<double> _unitScales = new List<double>();
        private readonly ComboBox _profile = new ComboBox();
        private readonly ComboBox _units = new ComboBox();
        private readonly Dictionary<string, TextBox> _inputs = new Dictionary<string, TextBox>();
        private readonly Dictionary<string, Label> _dataStatus = new Dictionary<string, Label>();
        private readonly Label _state = new Label();
        private readonly Label _unitState = new Label();
        private readonly Label _length = new Label();
        private readonly TextBox _source = new TextBox();
        private readonly LinkLabel _sourceLink = new LinkLabel();
        private readonly Label _turning = new Label();
        private readonly Label _error = new Label();
        private readonly PreviewPanel _preview = new PreviewPanel();
        private readonly Button _ok = new Button();
        private readonly CheckBox _body = new CheckBox();
        private readonly CheckBox _envelope = new CheckBox();
        private readonly CheckBox _track = new CheckBox();
        private readonly CheckBox _wheels = new CheckBox();
        private readonly CheckBox _ghost = new CheckBox();
        private readonly CheckBox _radii = new CheckBox();
        private readonly ComboBox _nodes = new ComboBox();
        private readonly ToolTip _tips = new ToolTip();
        private bool _loading;
        private bool _modified;
        private FireVehicleProfile Selected { get { return _profile.SelectedItem as FireVehicleProfile; } }

        public VehicleParams Result { get; private set; }
        public string SelectedProfileName { get; private set; }
        public string SourceSummary { get; private set; }
        /// <summary>独立消防高度元数据；仅预览，不能据此执行净高碰撞判断。</summary>
        public double ResultHeightM { get; private set; }

        public FireTruckSizeForm(VehicleParams current, double drawingUnitScale, double currentHeightM = 3.30)
        {
            _current = current;
            _drawingScale = FireVehicleCatalog.IsFinite(drawingUnitScale) && drawingUnitScale > 0 ? drawingUnitScale : 0;
            _profiles = FireVehicleCatalog.CreateProfiles();
            BuildUi();
            _loading = true;
            foreach (FireVehicleProfile p in _profiles) _profile.Items.Add(p);
            _units.Items.Add("米（1米 = 1图形单位）");
            _unitScales.Add(1.0);
            if (_drawingScale > 0) { _units.Items.Add("自动（随本图 INSUNITS）"); _unitScales.Add(_drawingScale); }
            _units.Items.Add("毫米（1米 = 1000图形单位）");
            _unitScales.Add(1000.0);
            _units.Items.Add("厘米（1米 = 100图形单位）");
            _unitScales.Add(100.0);
            if (current != null && FireVehicleCatalog.IsFinite(current.UnitScale) && current.UnitScale > 0 && current.UnitScale <= 1000000 &&
                _unitScales.FindIndex(value => Math.Abs(value - current.UnitScale) <= current.UnitScale * 1e-8) < 0)
            {
                _units.Items.Add("上次：1米 = " + current.UnitScale.ToString("R", CultureInfo.CurrentCulture) + "单位");
                _unitScales.Add(current.UnitScale);
            }
            _units.SelectedIndex = _drawingScale > 0 && Math.Abs(_drawingScale - 1) > 1e-8 ? 1 : 0;
            _body.Checked = current == null || current.ShowBody;
            _envelope.Checked = current == null || current.ShowEnvelope;
            _track.Checked = current == null || current.ShowFrontTrack;
            _wheels.Checked = current != null && current.ShowWheels;
            _ghost.Checked = current != null && current.ShowGhost;
            _radii.Checked = current != null && current.ShowRadii;
            _nodes.SelectedIndex = current != null && current.ShowAllNodeVehicles ? 1 : 0;
            _loading = false;
            double s = current == null ? 0 : current.UnitScale;
            // 已保存参数即使与更严格的新外廓校验冲突，也必须保留给用户修正。
            // 不能因为新校验失败而悄悄替换成通用模板或另一条轮距估算。
            if (current != null && !current.Articulated && FireVehicleCatalog.IsFinite(s) && s > 0 &&
                s <= 1000000 && FireVehicleCatalog.IsFinite(current.TractorWidth / s) &&
                FireVehicleCatalog.IsFinite(current.TractorWheelbase / s) && FireVehicleCatalog.IsFinite(current.TractorFrontOverhang / s) &&
                FireVehicleCatalog.IsFinite(current.RigidRearOverhang / s) && FireVehicleCatalog.IsFinite(current.WheelTrack / s) &&
                FireVehicleCatalog.IsFinite(current.SteerAngleDeg))
            {
                // 只有独立消防参数调用方才传入 current；回填为自定义，不能暗中继承厂家身份。
                _loading = true;
                SetValue("width", current.TractorWidth / s);
                SetValue("height", FireVehicleCatalog.IsFinite(currentHeightM) ? currentHeightM : 3.30);
                SetValue("wheelbase", current.TractorWheelbase / s); SetValue("front", current.TractorFrontOverhang / s);
                SetValue("rear", current.RigidRearOverhang / s); SetValue("track", current.WheelTrack / s); SetValue("steer", current.SteerAngleDeg);
                int lastUnit = _unitScales.FindIndex(value => Math.Abs(value - s) <= s * 1e-8);
                _units.SelectedIndex = lastUnit >= 0 ? lastUnit : 0;
                _loading = false;
                _profile.SelectedIndex = 2;
            }
            else _profile.SelectedIndex = 0;
        }

        private void BuildUi()
        {
            Text = "消防车选型 · 两轴前桥转向";
            ClientSize = new Size(990, 690);
            MinimumSize = new Size(930, 670);
            StartPosition = FormStartPosition.CenterScreen;
            MinimizeBox = false;
            MaximizeBox = false;
            AutoScaleMode = AutoScaleMode.Font;
            Font = new Font("Microsoft YaHei UI", 9F);
            BackColor = Color.FromArgb(245, 247, 249);
            var outer = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 3, Padding = new Padding(16) };
            outer.RowStyles.Add(new RowStyle(SizeType.Absolute, 84));
            outer.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            outer.RowStyles.Add(new RowStyle(SizeType.Absolute, 64));
            Controls.Add(outer);
            var top = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 2 };
            top.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 84));
            top.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            top.Controls.Add(new Label { Text = "消防车型", AutoSize = true, Anchor = AnchorStyles.Left }, 0, 0);
            _profile.DropDownStyle = ComboBoxStyle.DropDownList;
            _profile.Dock = DockStyle.Fill;
            _profile.SelectedIndexChanged += (s, e) => ApplyProfile();
            top.Controls.Add(_profile, 1, 0);
            _state.AutoSize = false; _state.Dock = DockStyle.Fill; _state.ForeColor = Color.FromArgb(160, 85, 35);
            top.Controls.Add(_state, 1, 1);
            outer.Controls.Add(top, 0, 0);

            var middle = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1 };
            middle.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 48));
            middle.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 52));
            var tabs = new TabControl { Dock = DockStyle.Fill, Margin = new Padding(0, 0, 12, 0) };
            var geometry = new TabPage("车辆参数（米）") { Padding = new Padding(10), UseVisualStyleBackColor = true };
            var output = new TabPage("CAD 输出") { Padding = new Padding(14), UseVisualStyleBackColor = true };
            tabs.TabPages.Add(geometry); tabs.TabPages.Add(output); middle.Controls.Add(tabs, 0, 0);

            var fields = new TableLayoutPanel { Dock = DockStyle.Top, AutoSize = true, ColumnCount = 3 };
            fields.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 106));
            fields.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 93));
            fields.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            AddField(fields, "width", "最大行驶车宽", "含轮胎在全部允许转角内以及固定突出物的最大行驶宽度；CAD本体和包络均按此宽度建模，不另加超宽后视镜。");
            AddField(fields, "height", "车高（预览）", "仅用于侧立面显示，不执行净高或三维碰撞校核。");
            AddField(fields, "wheelbase", "前后轴距", "前轴中心到后轴中心的距离，不是整车长。");
            AddField(fields, "front", "前悬", "前轴到车头最前端，含行驶时固定前伸部分。");
            AddField(fields, "rear", "后悬", "后轴到车尾最后端，含行驶时固定后伸部分。");
            AddField(fields, "track", "轮距", "同轴左右轮中心距离，不是轮胎外缘距离；应给0.50×0.25米轮胎在全部允许转角内留足空间。");
            AddField(fields, "steer", "最大前轮转角 °", "前轮相对车身的转角，不是方向盘转角，也不是转弯半径。");
            int row = fields.RowCount++;
            fields.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
            fields.Controls.Add(new Label { Text = "整车长", Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft }, 0, row);
            _length.Dock = DockStyle.Fill; _length.TextAlign = ContentAlignment.MiddleLeft;
            fields.Controls.Add(_length, 1, row); fields.SetColumnSpan(_length, 2);
            row = fields.RowCount++; fields.RowStyles.Add(new RowStyle(SizeType.Absolute, 35));
            fields.Controls.Add(new Label { Text = "图纸单位", Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft }, 0, row);
            _units.DropDownStyle = ComboBoxStyle.DropDownList; _units.Dock = DockStyle.Fill;
            _units.SelectedIndexChanged += (s, e) => RefreshFeedback();
            fields.Controls.Add(_units, 1, row); fields.SetColumnSpan(_units, 2);
            var left = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 3 };
            left.RowStyles.Add(new RowStyle(SizeType.AutoSize)); left.RowStyles.Add(new RowStyle(SizeType.Absolute, 58));
            left.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            left.Controls.Add(fields, 0, 0);
            _unitState.Dock = DockStyle.Fill; _unitState.Margin = new Padding(2, 8, 0, 0);
            left.Controls.Add(_unitState, 0, 1);
            left.Controls.Add(new Label { Dock = DockStyle.Fill, Text = "模型：两轴刚性车、仅前桥转向。\r\n长度 = 前悬 + 轴距 + 后悬。\r\n云梯/支腿展开、三维净高、动态侧倾不在本版校核范围。", ForeColor = Color.DimGray }, 0, 2);
            geometry.Controls.Add(left);
            BuildOutput(output);

            var right = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 4 };
            right.RowStyles.Add(new RowStyle(SizeType.Percent, 64));
            right.RowStyles.Add(new RowStyle(SizeType.Absolute, 28));
            right.RowStyles.Add(new RowStyle(SizeType.Percent, 23));
            right.RowStyles.Add(new RowStyle(SizeType.Percent, 13));
            _preview.Dock = DockStyle.Fill; _preview.BackColor = Color.White; _preview.Paint += DrawPreview;
            right.Controls.Add(_preview, 0, 0);
            _sourceLink.Text = "查看厂家原始资料（需联网）"; _sourceLink.Dock = DockStyle.Fill;
            _sourceLink.LinkClicked += OpenSource; right.Controls.Add(_sourceLink, 0, 1);
            _source.Dock = DockStyle.Fill; _source.Multiline = true; _source.ReadOnly = true;
            _source.ScrollBars = ScrollBars.Vertical; _source.BorderStyle = BorderStyle.None;
            _source.BackColor = BackColor; _source.ForeColor = Color.FromArgb(75, 80, 88);
            right.Controls.Add(_source, 0, 2);
            _turning.Dock = DockStyle.Fill; _turning.ForeColor = Color.FromArgb(160, 85, 35);
            right.Controls.Add(_turning, 0, 3);
            middle.Controls.Add(right, 1, 0); outer.Controls.Add(middle, 0, 1);

            var bottom = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, Padding = new Padding(0, 12, 0, 0) };
            bottom.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100)); bottom.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 112));
            bottom.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 92));
            _error.Dock = DockStyle.Fill; _error.ForeColor = Color.Firebrick; bottom.Controls.Add(_error, 0, 0);
            _ok.Text = "确定并绘制"; _ok.Dock = DockStyle.Fill; _ok.Click += Confirm;
            var cancel = new Button { Text = "取消", Dock = DockStyle.Fill, DialogResult = DialogResult.Cancel };
            bottom.Controls.Add(_ok, 1, 0); bottom.Controls.Add(cancel, 2, 0);
            outer.Controls.Add(bottom, 0, 2); AcceptButton = _ok; CancelButton = cancel;
        }

        private void AddField(TableLayoutPanel table, string key, string caption, string explanation)
        {
            int row = table.RowCount++; table.RowStyles.Add(new RowStyle(SizeType.Absolute, 35));
            var label = new Label { Text = caption, Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft };
            var box = new TextBox { Dock = DockStyle.Fill, Margin = new Padding(2, 5, 8, 3) };
            var status = new Label { Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft, ForeColor = Color.DimGray };
            _inputs.Add(key, box); _dataStatus.Add(key, status);
            box.TextChanged += (s, e) => { if (!_loading) { _modified = true; RefreshFeedback(); } };
            _tips.SetToolTip(label, explanation); _tips.SetToolTip(box, explanation);
            table.Controls.Add(label, 0, row); table.Controls.Add(box, 1, row); table.Controls.Add(status, 2, row);
        }

        private void BuildOutput(TabPage page)
        {
            var layout = new TableLayoutPanel { Dock = DockStyle.Top, AutoSize = true, ColumnCount = 1 };
            foreach (CheckBox box in new[] { _envelope, _body, _track, _wheels, _ghost, _radii })
            { box.AutoSize = true; box.Margin = new Padding(0, 7, 0, 7); layout.Controls.Add(box); }
            _envelope.Text = "车辆累计最外包络"; _body.Text = "车辆本体"; _track.Text = "前轴参考轨迹";
            _wheels.Text = "轮迹"; _ghost.Text = "中间采样姿态（图元较多）"; _radii.Text = "半径标注";
            layout.Controls.Add(new Label { AutoSize = true, Text = "确认节点车体保留方式", Margin = new Padding(0, 18, 0, 4) });
            _nodes.DropDownStyle = ComboBoxStyle.DropDownList; _nodes.Width = 310;
            _nodes.Items.Add("仅保留初始位置与结束位置"); _nodes.Items.Add("保留全部确认节点车辆"); layout.Controls.Add(_nodes);
            layout.Controls.Add(new Label { AutoSize = true, MaximumSize = new Size(360, 0), Margin = new Padding(0, 18, 0, 0),
                Text = "首尾模式不删除中间包络。中间采样姿态与确认节点是不同选项；需要简洁图面时请关闭采样姿态。", ForeColor = Color.DimGray });
            page.Controls.Add(layout);
        }

        private void ApplyProfile()
        {
            if (_loading || Selected == null) return;
            FireVehicleProfile p = Selected;
            if (p.Supported && p.Id != "custom-two-axle")
            {
                _loading = true;
                SetValue("width", p.WidthM); SetValue("height", p.HeightM); SetValue("wheelbase", p.WheelbaseM);
                SetValue("front", p.FrontOverhangM); SetValue("rear", p.RearOverhangM); SetValue("track", p.TrackM); SetValue("steer", p.SteerDeg);
                _loading = false;
            }
            _modified = false;
            foreach (TextBox input in _inputs.Values) input.Enabled = p.Supported;
            _sourceLink.Visible = !string.IsNullOrEmpty(p.SourceUrl);
            _source.Text = p.SourceTitle + "\r\n" + p.SourceNotes;
            _turning.Text = p.PublishedTurningData;
            RefreshFeedback();
        }

        private void SetValue(string key, double value) { _inputs[key].Text = value.ToString("R", CultureInfo.CurrentCulture); }
        private double Value(string key)
        {
            double v;
            string text = _inputs[key].Text.Trim();
            if (double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out v) ||
                double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out v)) return v;
            return double.NaN;
        }
        private double UnitScale { get { return _units.SelectedIndex >= 0 && _units.SelectedIndex < _unitScales.Count ? _unitScales[_units.SelectedIndex] : 1.0; } }
        private bool ValidateInput(out string error)
        {
            if (Selected == null || !Selected.Supported) { error = "该转向结构尚未支持，不能套用两轴模型。"; return false; }
            return FireVehicleCatalog.ValidateGeometry(Value("width"), Value("height"), Value("wheelbase"), Value("front"),
                Value("rear"), Value("track"), Value("steer"), UnitScale, out error);
        }

        private void RefreshFeedback()
        {
            if (_loading || Selected == null) return;
            FireVehicleProfile p = Selected;
            _state.Text = !p.Supported ? "当前型号仅作结构边界提示，禁止使用错误模型绘制。" :
                p.IsManufacturerReference ? (_modified ? "公开参考模板已修改 · 运动参数仍待核验 · 非厂家认证" : "公开尺寸 + 估算运动参数 · 非厂家认证") :
                "通用/用户参数 · 未核验 · 不代表任何指定厂家车型";
            foreach (var pair in _dataStatus)
            {
                bool published = p.IsManufacturerReference && (pair.Key == "width" || pair.Key == "height" || pair.Key == "wheelbase");
                pair.Value.Text = !p.Supported ? "不适用" : _modified ? "已编辑/待核验" : published ? "公开参考值" : "估算/待核验";
                pair.Value.ForeColor = published && !_modified ? Color.FromArgb(65, 105, 90) : Color.FromArgb(160, 85, 35);
            }
            double len = Value("front") + Value("wheelbase") + Value("rear");
            _length.Text = FireVehicleCatalog.IsFinite(len) ? len.ToString("0.###") + " m（前悬＋轴距＋后悬）" : "—";
            bool match = _drawingScale > 0 && Math.Abs(UnitScale - _drawingScale) <= _drawingScale * 1e-8;
            _unitState.ForeColor = _drawingScale <= 0 || match ? Color.DimGray : Color.Firebrick;
            _unitState.Text = _drawingScale <= 0 ? "未识别本图单位；当前1米 = " + UnitScale.ToString("0.###") + "图形单位，请核对。" :
                match ? "与本图 INSUNITS 一致：1米 = " + UnitScale.ToString("0.###") + " 图形单位。" :
                "注意：当前1米 = " + UnitScale.ToString("0.###") + "单位，本图 INSUNITS 为1米 = " + _drawingScale.ToString("0.###") + "单位；请核对实际图形尺度。";
            string error; bool valid = ValidateInput(out error); _error.Text = valid ? "请核实参考参数后用于项目。" : error; _ok.Enabled = valid;
            _preview.Invalidate();
        }

        private void Confirm(object sender, EventArgs e)
        {
            string error;
            if (!ValidateInput(out error)) { _error.Text = error; return; }
            VehicleParams display = _current == null ? VehicleParams.Defaults() : VehicleParams.FromCsv(_current.ToCsv());
            display.ShowBody = _body.Checked; display.ShowEnvelope = _envelope.Checked; display.ShowFrontTrack = _track.Checked;
            display.ShowWheels = _wheels.Checked; display.ShowGhost = _ghost.Checked; display.ShowRadii = _radii.Checked;
            display.ShowAllNodeVehicles = _nodes.SelectedIndex == 1;
            Result = FireVehicleCatalog.BuildVehicle(Value("width"), Value("height"), Value("wheelbase"), Value("front"),
                Value("rear"), Value("track"), Value("steer"), UnitScale, display);
            ResultHeightM = Value("height");
            SelectedProfileName = Selected.Name + (_modified ? "（参数已修改）" : "");
            SourceSummary = Selected.SourceTitle + "；" + Selected.SourceNotes + "；" + (Selected.IsManufacturerReference ? "公开尺寸与估算运动参数混合，未认证" : "通用/用户参数，未核验") +
                (_modified ? "；当前参数已编辑，不应当作原厂数值" : "") + (string.IsNullOrEmpty(Selected.SourceUrl) ? "" : "；" + Selected.SourceUrl);
            DialogResult = DialogResult.OK; Close();
        }

        private void OpenSource(object sender, LinkLabelLinkClickedEventArgs e)
        {
            if (Selected == null) return;
            Uri uri;
            if (!Uri.TryCreate(Selected.SourceUrl, UriKind.Absolute, out uri) || uri.Scheme != Uri.UriSchemeHttps) return;
            try { Process.Start(new ProcessStartInfo(uri.AbsoluteUri) { UseShellExecute = true }); }
            catch (Exception ex) { MessageBox.Show(this, "无法打开浏览器：" + ex.Message + "\r\n" + uri.AbsoluteUri, "资料链接", MessageBoxButtons.OK, MessageBoxIcon.Information); }
        }

        private void DrawPreview(object sender, PaintEventArgs e)
        {
            string message = null;
            if (Selected == null || !Selected.Supported)
                message = "该结构暂不支持精确建模\n不能以两轴前桥模型代替";
            else
            {
                string error;
                if (!ValidateInput(out error)) message = "输入有效参数后显示车辆侧视图";
            }
            FireVehicleSideView.Draw(e.Graphics, _preview.ClientRectangle, Value("front"),
                Value("wheelbase"), Value("rear"), Value("height"), message);
        }

        private sealed class PreviewPanel : Panel
        {
            public PreviewPanel() { DoubleBuffered = true; ResizeRedraw = true; }
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing) _tips.Dispose();
            base.Dispose(disposing);
        }
    }
}
