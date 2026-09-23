using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.Windows.Forms;

namespace TruckTurn
{
    /// <summary>CARRIERDRIVE 独立参数窗口，不读写旧货车 VehicleParams。</summary>
    public sealed class CarrierSizeForm : Form
    {
        readonly ComboBox cboArchitecture = Combo();
        readonly ComboBox cboDataStatus = Combo();
        readonly ComboBox cboRadiusReference = Combo();
        readonly ComboBox cboContainer = Combo();
        readonly ComboBox cboUnits = Combo();
        readonly TextBox txtPreset = Box();
        // 为兼容旧图纸/旧参数结构而保留；当前界面不再显示，避免误以为输入型号会自动识别车型。
        readonly TextBox txtManufacturer = Box();
        readonly TextBox txtModel = Box();
        readonly TextBox txtLength = Box();
        readonly TextBox txtWidth = Box();
        readonly TextBox txtWheelbase = Box();
        readonly TextBox txtTrack = Box();
        readonly TextBox txtWheelLength = Box();
        readonly TextBox txtWheelWidth = Box();
        readonly TextBox txtMaxSteer = Box();
        readonly TextBox txtSteerRate = Box();
        readonly TextBox txtMinRadius = Box();
        readonly TextBox txtSafety = Box();
        readonly TextBox txtContainerLength = Box();
        readonly TextBox txtContainerWidth = Box();
        readonly TextBox txtContainerHeight = Box();
        readonly TextBox txtContainerOffsetX = Box();
        readonly TextBox txtContainerOffsetY = Box();
        readonly TextBox txtContainerSlew = Box();

        readonly CheckBox chkFront = Check("前桥转向");
        readonly CheckBox chkRear = Check("后桥转向");
        readonly CheckBox chkCounter = Check("反相四轮");
        readonly CheckBox chkCrab = Check("蟹行");
        readonly CheckBox chkLateral = Check("纯横移");
        readonly CheckBox chkPivot = Check("原地回转（需明确确认）");

        readonly CheckBox chkPath = Check("参考点轨迹");
        readonly CheckBox chkEquipmentEnvelope = Check("设备包络");
        readonly CheckBox chkContainerEnvelope = Check("集装箱包络");
        readonly CheckBox chkSafetyEnvelope = Check("安全包络");
        readonly CheckBox chkWheelTracks = Check("四轮轨迹");
        readonly CheckBox chkBody = Check("车体与车轮");
        readonly ComboBox cboNodeVehicleDisplay = Combo();
        readonly DoubleBufferedPanel pnlPreview = new DoubleBufferedPanel();

        readonly double drawingUnitScale;
        bool loading;
        public CarrierParams Result { get; private set; }

        public CarrierSizeForm(CarrierParams current, double detectedDrawingUnitScale)
        {
            drawingUnitScale = detectedDrawingUnitScale;
            Text = "集装箱跨运车选型与轨迹参数";
            StartPosition = FormStartPosition.CenterScreen;
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            ClientSize = new Size(1120, 720);
            Font = new Font("Microsoft YaHei UI", 9F);

            var tabs = new TabControl { Dock = DockStyle.Fill };
            tabs.TabPages.Add(BuildVehicleTab());
            tabs.TabPages.Add(BuildContainerTab());
            tabs.TabPages.Add(BuildOutputTab());

            var note = new Label
            {
                Dock = DockStyle.Top,
                Height = 48,
                Padding = new Padding(10, 6, 10, 4),
                ForeColor = Color.DarkRed,
                Text = "轨迹仅按下方数字参数计算；方案名称只用于识别。当前内置参数是通用运动学模板，不代表厂家认证；原地回转默认关闭。"
            };

            var buttons = new FlowLayoutPanel
            {
                Dock = DockStyle.Bottom,
                Height = 48,
                FlowDirection = FlowDirection.RightToLeft,
                Padding = new Padding(8)
            };
            var ok = new Button { Text = "确定", Width = 90, DialogResult = DialogResult.None };
            var cancel = new Button { Text = "取消", Width = 90, DialogResult = DialogResult.Cancel };
            ok.Click += delegate { BuildResult(); };
            buttons.Controls.Add(ok);
            buttons.Controls.Add(cancel);
            AcceptButton = ok;
            CancelButton = cancel;

            var content = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                ColumnCount = 2,
                RowCount = 1,
                Padding = new Padding(8, 4, 8, 4)
            };
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 54F));
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 46F));
            content.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            content.Controls.Add(tabs, 0, 0);
            content.Controls.Add(BuildPreviewPane(), 1, 0);

            Controls.Add(content);
            Controls.Add(note);
            Controls.Add(buttons);

            FillChoices();
            WirePreviewEvents();
            LoadCurrent(current ?? CarrierParams.GenericFourWheelIndependent());
        }

        Control BuildPreviewPane()
        {
            var group = new GroupBox
            {
                Text = "参数化侧立面预览",
                Dock = DockStyle.Fill,
                Padding = new Padding(10),
                Margin = new Padding(10, 4, 2, 4)
            };
            pnlPreview.Dock = DockStyle.Fill;
            pnlPreview.BackColor = Color.FromArgb(250, 251, 252);
            pnlPreview.Paint += delegate(object sender, PaintEventArgs e)
            {
                DrawCarrierSideProfile(e.Graphics);
            };
            group.Controls.Add(pnlPreview);
            return group;
        }

        TabPage BuildVehicleTab()
        {
            var page = new TabPage("设备与转向");
            var table = FormTable();
            AddRow(table, "方案名称", txtPreset);
            AddRow(table, "数据状态", cboDataStatus);
            AddRow(table, "运动学架构", cboArchitecture);
            AddRow(table, "整机长度 m", txtLength);
            AddRow(table, "整机宽度 m", txtWidth);
            AddRow(table, "轴距 m", txtWheelbase);
            AddRow(table, "轮距 m", txtTrack);
            AddRow(table, "轮胎长度 m", txtWheelLength);
            AddRow(table, "轮胎宽度 m", txtWheelWidth);
            AddRow(table, "各轮最大转角 °", txtMaxSteer);
            AddRow(table, "最大转向速度 °/s", txtSteerRate);
            AddRow(table, "最小转弯半径 m", txtMinRadius);
            AddRow(table, "半径测量基准", cboRadiusReference);

            var modes = new FlowLayoutPanel { Dock = DockStyle.Fill, AutoSize = true };
            modes.Controls.AddRange(new Control[]
            {
                chkFront, chkRear, chkCounter, chkCrab, chkLateral, chkPivot
            });
            AddRow(table, "允许的转向模式", modes);
            page.Controls.Add(table);
            return page;
        }

        TabPage BuildContainerTab()
        {
            var page = new TabPage("集装箱与安全裕量");
            var table = FormTable();
            AddRow(table, "集装箱选型", cboContainer);
            AddRow(table, "自定义箱长 m", txtContainerLength);
            AddRow(table, "自定义箱宽 m", txtContainerWidth);
            AddRow(table, "自定义箱高 m", txtContainerHeight);
            AddRow(table, "箱体纵向偏置 m", txtContainerOffsetX);
            AddRow(table, "箱体横向偏置 m", txtContainerOffsetY);
            AddRow(table, "箱体相对转角 °", txtContainerSlew);
            AddRow(table, "安全裕量 m", txtSafety);
            AddRow(table, "图纸单位", cboUnits);

            var help = new Label
            {
                AutoSize = true,
                MaximumSize = new Size(610, 0),
                ForeColor = Color.DimGray,
                Text = "集装箱二维包络使用外部长宽；高度随设备参数保存在图纸中，不生成文字标注。安全裕量是规划假设，不替代设备厂家或现场安全标准。"
            };
            AddRow(table, "说明", help);
            page.Controls.Add(table);
            return page;
        }

        TabPage BuildOutputTab()
        {
            var page = new TabPage("CAD 输出");
            var panel = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                Padding = new Padding(24),
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false
            };
            panel.Controls.AddRange(new Control[]
            {
                chkPath, chkEquipmentEnvelope, chkContainerEnvelope,
                chkSafetyEnvelope, chkWheelTracks, chkBody
            });
            panel.Controls.Add(new Label
            {
                Text = "车辆姿态显示", AutoSize = true,
                Margin = new Padding(3, 14, 3, 2)
            });
            cboNodeVehicleDisplay.Width = 240;
            panel.Controls.Add(cboNodeVehicleDisplay);
            page.Controls.Add(panel);
            return page;
        }

        void FillChoices()
        {
            cboArchitecture.Items.AddRange(new object[] { "四轮联动转向", "四轮独立转向" });
            cboDataStatus.Items.AddRange(new object[] { "通用参数（未认证）", "用户自定义（未认证）" });
            cboRadiusReference.Items.AddRange(new object[]
            {
                "车体参考点", "内侧轮迹", "外侧轮迹", "整机外廓"
            });
            cboContainer.Items.AddRange(new object[]
            {
                "无箱", "20GP", "40GP", "40HC", "45HC", "自定义", "2×20"
            });
            cboUnits.Items.Add("米（1m = 1图形单位）");
            cboUnits.Items.Add("毫米（1m = 1000图形单位）");
            if (drawingUnitScale > 0.0)
                cboUnits.Items.Add("自动（按本图 INSUNITS）");
            cboNodeVehicleDisplay.Items.AddRange(new object[] { "仅首尾", "全部节点" });

            cboArchitecture.SelectedIndexChanged += delegate
            {
                if (!loading) ApplyArchitectureDefaults();
                RedrawPreview();
            };
            cboContainer.SelectedIndexChanged += delegate
            {
                UpdateContainerInputs();
                RedrawPreview();
            };
        }

        void WirePreviewEvents()
        {
            TextBox[] boxes =
            {
                txtLength, txtWidth, txtWheelbase, txtTrack,
                txtWheelLength, txtWheelWidth, txtSafety,
                txtContainerLength, txtContainerWidth, txtContainerHeight,
                txtContainerOffsetX, txtContainerOffsetY, txtContainerSlew
            };
            for (int i = 0; i < boxes.Length; i++)
                boxes[i].TextChanged += delegate
                {
                    if (!loading) RedrawPreview();
                };
        }

        void LoadCurrent(CarrierParams p)
        {
            loading = true;
            txtPreset.Text = p.PresetName;
            txtManufacturer.Text = p.Manufacturer;
            txtModel.Text = p.Model;
            cboDataStatus.SelectedIndex = p.DataStatus == CarrierDataStatus.UserDefined ? 1 : 0;
            cboArchitecture.SelectedIndex = p.Architecture == CarrierArchitecture.FourWheelLinked ? 0 : 1;
            txtLength.Text = Number(p.OverallLength);
            txtWidth.Text = Number(p.OverallWidth);
            txtWheelbase.Text = Number(p.Wheelbase);
            txtTrack.Text = Number(p.TrackWidth);
            txtWheelLength.Text = Number(p.WheelLength);
            txtWheelWidth.Text = Number(p.WheelWidth);
            txtMaxSteer.Text = Number(p.FrontLeft.MaxSteerAngleDeg);
            txtSteerRate.Text = Number(p.MaxSteerRateDegPerSecond);
            txtMinRadius.Text = Number(p.MinimumReferenceRadius);
            cboRadiusReference.SelectedIndex = (int)p.TurningRadiusReference;

            chkFront.Checked = p.Supports(CarrierSteeringMode.FrontOnly);
            chkRear.Checked = p.Supports(CarrierSteeringMode.RearOnly);
            chkCounter.Checked = p.Supports(CarrierSteeringMode.CounterPhaseFourWheel);
            chkCrab.Checked = p.Supports(CarrierSteeringMode.Crab);
            chkLateral.Checked = p.Supports(CarrierSteeringMode.Lateral);
            chkPivot.Checked = p.Supports(CarrierSteeringMode.Pivot);

            cboContainer.SelectedIndex = (int)p.ContainerPreset;
            txtContainerLength.Text = Number(p.CustomContainerLength);
            txtContainerWidth.Text = Number(p.CustomContainerWidth);
            txtContainerHeight.Text = Number(p.CustomContainerHeight);
            txtContainerOffsetX.Text = Number(p.ContainerOffsetX);
            txtContainerOffsetY.Text = Number(p.ContainerOffsetY);
            txtContainerSlew.Text = Number(p.ContainerSlewAngleDeg);
            txtSafety.Text = Number(p.SafetyMargin);
            if (drawingUnitScale > 0.0 && Math.Abs(p.UnitScale - drawingUnitScale) < 1e-6)
                cboUnits.SelectedIndex = 2;
            else
                cboUnits.SelectedIndex = p.UnitScale >= 999.5 ? 1 : 0;

            chkPath.Checked = p.ShowReferencePath;
            chkEquipmentEnvelope.Checked = p.ShowEquipmentEnvelope;
            chkContainerEnvelope.Checked = p.ShowContainerEnvelope;
            chkSafetyEnvelope.Checked = p.ShowSafetyEnvelope;
            chkWheelTracks.Checked = p.ShowWheelTracks;
            chkBody.Checked = p.ShowBody;
            cboNodeVehicleDisplay.SelectedIndex = p.ShowAllNodeVehicles ? 1 : 0;
            loading = false;
            UpdateContainerInputs();
            RedrawPreview();
        }

        void ApplyArchitectureDefaults()
        {
            CarrierParams p = cboArchitecture.SelectedIndex == 0
                ? CarrierParams.GenericFourWheelLinked()
                : CarrierParams.GenericFourWheelIndependent();
            chkFront.Checked = p.Supports(CarrierSteeringMode.FrontOnly);
            chkRear.Checked = p.Supports(CarrierSteeringMode.RearOnly);
            chkCounter.Checked = p.Supports(CarrierSteeringMode.CounterPhaseFourWheel);
            chkCrab.Checked = p.Supports(CarrierSteeringMode.Crab);
            chkLateral.Checked = p.Supports(CarrierSteeringMode.Lateral);
            chkPivot.Checked = false;
            txtMaxSteer.Text = Number(p.FrontLeft.MaxSteerAngleDeg);
            RedrawPreview();
        }

        void UpdateContainerInputs()
        {
            bool custom = cboContainer.SelectedIndex == (int)CarrierContainerPreset.Custom;
            txtContainerLength.Enabled = custom;
            txtContainerWidth.Enabled = custom;
            txtContainerHeight.Enabled = custom;
        }

        void BuildResult()
        {
            try
            {
                var p = new CarrierParams();
                p.PresetName = string.IsNullOrWhiteSpace(txtPreset.Text)
                    ? "用户跨运车参数" : txtPreset.Text.Trim();
                p.Manufacturer = txtManufacturer.Text.Trim();
                p.Model = txtModel.Text.Trim();
                p.DataStatus = cboDataStatus.SelectedIndex == 1
                    ? CarrierDataStatus.UserDefined : CarrierDataStatus.Generic;
                p.Architecture = cboArchitecture.SelectedIndex == 0
                    ? CarrierArchitecture.FourWheelLinked : CarrierArchitecture.FourWheelIndependent;
                p.OverallLength = Read(txtLength, "整机长度");
                p.OverallWidth = Read(txtWidth, "整机宽度");
                p.Wheelbase = Read(txtWheelbase, "轴距");
                p.TrackWidth = Read(txtTrack, "轮距");
                p.WheelLength = Read(txtWheelLength, "轮胎长度");
                p.WheelWidth = Read(txtWheelWidth, "轮胎宽度");
                double maxSteer = Read(txtMaxSteer, "最大转角");
                p.FrontLeft.MaxSteerAngleDeg = maxSteer;
                p.FrontRight.MaxSteerAngleDeg = maxSteer;
                p.RearLeft.MaxSteerAngleDeg = maxSteer;
                p.RearRight.MaxSteerAngleDeg = maxSteer;
                p.MaxSteerRateDegPerSecond = Read(txtSteerRate, "最大转向速度");
                p.MinimumReferenceRadius = Read(txtMinRadius, "最小转弯半径");
                p.TurningRadiusReference = (CarrierTurningRadiusReference)Math.Max(0, cboRadiusReference.SelectedIndex);

                p.SteeringCapabilities = CarrierSteeringCapabilities.Longitudinal;
                if (chkFront.Checked) p.SteeringCapabilities |= CarrierSteeringCapabilities.FrontOnly;
                if (chkRear.Checked) p.SteeringCapabilities |= CarrierSteeringCapabilities.RearOnly;
                if (chkCounter.Checked) p.SteeringCapabilities |= CarrierSteeringCapabilities.CounterPhaseFourWheel;
                if (chkCrab.Checked) p.SteeringCapabilities |= CarrierSteeringCapabilities.Crab;
                if (chkLateral.Checked) p.SteeringCapabilities |= CarrierSteeringCapabilities.Lateral;
                if (chkPivot.Checked) p.SteeringCapabilities |= CarrierSteeringCapabilities.Pivot;
                if (chkLateral.Checked && maxSteer < 89.5)
                    throw new InvalidOperationException("启用纯横移时，各轮最大转角必须至少为 89.5°。");
                if (chkPivot.Checked && p.Architecture != CarrierArchitecture.FourWheelIndependent)
                    throw new InvalidOperationException("原地回转只允许四轮独立转向架构。");

                p.ContainerPreset = (CarrierContainerPreset)Math.Max(0, cboContainer.SelectedIndex);
                p.CustomContainerLength = Read(txtContainerLength, "自定义箱长");
                p.CustomContainerWidth = Read(txtContainerWidth, "自定义箱宽");
                p.CustomContainerHeight = Read(txtContainerHeight, "自定义箱高");
                p.ContainerOffsetX = Read(txtContainerOffsetX, "箱体纵向偏置", false);
                p.ContainerOffsetY = Read(txtContainerOffsetY, "箱体横向偏置", false);
                p.ContainerSlewAngleDeg = Read(txtContainerSlew, "箱体相对转角", false);
                p.SafetyMargin = Read(txtSafety, "安全裕量", false);
                if (p.SafetyMargin < 0.0) throw new InvalidOperationException("安全裕量不能为负数。");
                p.UnitScale = cboUnits.SelectedIndex == 1
                    ? 1000.0
                    : (cboUnits.SelectedIndex == 2 && drawingUnitScale > 0.0 ? drawingUnitScale : 1.0);

                p.ShowReferencePath = chkPath.Checked;
                p.ShowEquipmentEnvelope = chkEquipmentEnvelope.Checked;
                p.ShowContainerEnvelope = chkContainerEnvelope.Checked;
                p.ShowSafetyEnvelope = chkSafetyEnvelope.Checked;
                p.ShowWheelTracks = chkWheelTracks.Checked;
                p.ShowBody = chkBody.Checked;
                p.ShowAllNodeVehicles = cboNodeVehicleDisplay.SelectedIndex == 1;

                var errors = p.Validate();
                if (errors.Count > 0)
                    throw new InvalidOperationException(string.Join("\n", errors));
                Result = p;
                DialogResult = DialogResult.OK;
                Close();
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, ex.Message, "参数有误",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
        }

        // ================= 跨运车参数化侧立面预览 =================

        void RedrawPreview()
        {
            if (pnlPreview != null) pnlPreview.Invalidate();
        }

        void DrawCarrierSideProfile(Graphics g)
        {
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;
            g.Clear(Color.FromArgb(250, 251, 252));

            double length, width, wheelbase, track, wheelLength, wheelWidth, safety;
            if (!TryPreviewNumber(txtLength, out length) ||
                !TryPreviewNumber(txtWidth, out width) ||
                !TryPreviewNumber(txtWheelbase, out wheelbase) ||
                !TryPreviewNumber(txtTrack, out track) ||
                !TryPreviewNumber(txtWheelLength, out wheelLength) ||
                !TryPreviewNumber(txtWheelWidth, out wheelWidth) ||
                !TryPreviewNumber(txtSafety, out safety, true) ||
                length <= 0.1 || width <= 0.1 || wheelbase <= 0.1)
            {
                DrawPreviewHint(g, "请输入有效的整机与轮位尺寸");
                return;
            }

            CarrierContainerDimensions box = PreviewContainerDimensions();
            double offsetX = PreviewNumber(txtContainerOffsetX, 0.0);
            double offsetY = PreviewNumber(txtContainerOffsetY, 0.0);
            double slew = PreviewNumber(txtContainerSlew, 0.0) * Math.PI / 180.0;
            double boxSideLength = box.Length <= 0.0 ? 0.0 :
                Math.Abs(box.Length * Math.Cos(slew)) + Math.Abs(box.Width * Math.Sin(slew));

            double minX = -length * 0.5;
            double maxX = length * 0.5;
            if (boxSideLength > 0.0)
            {
                minX = Math.Min(minX, offsetX - boxSideLength * 0.5);
                maxX = Math.Max(maxX, offsetX + boxSideLength * 0.5);
            }

            double topHeight = Math.Max(5.2, box.Height > 0.0 ? box.Height + 2.35 : 5.2);
            const int sidePad = 28;
            float groundY = pnlPreview.Height - 78;
            if (groundY < 170) groundY = pnlPreview.Height - 42;
            double scaleW = Math.Max(20.0, pnlPreview.Width - sidePad * 2.0) /
                            Math.Max(1.0, maxX - minX + 1.2);
            double scaleH = Math.Max(80.0, groundY - 92.0) / (topHeight + 0.45);
            double scale = Math.Max(4.0, Math.Min(scaleW, scaleH));
            double midX = (minX + maxX) * 0.5;
            float centerPx = pnlPreview.Width * 0.5f;
            float X(double x) { return centerPx + (float)((x - midX) * scale); }
            float Y(double y) { return groundY - (float)(y * scale); }

            using (var titleFont = new Font(Font.FontFamily, 11.5f, FontStyle.Bold))
            using (var metaFont = new Font(Font.FontFamily, 8.25f, FontStyle.Regular))
            using (var titleBrush = new SolidBrush(Color.FromArgb(45, 55, 64)))
            using (var metaBrush = new SolidBrush(Color.FromArgb(105, 114, 122)))
            {
                g.DrawString("集装箱跨运车  ·  侧立面", titleFont, titleBrush, 18, 16);
                string boxName = cboContainer.SelectedItem == null
                    ? "无载荷" : cboContainer.SelectedItem.ToString();
                g.DrawString(string.Format(
                    "{0:F1} × {1:F1} m   轮距 {2:F1} m   箱型 {3}",
                    length, width, track, boxName), metaFont, metaBrush, 19, 39);
            }

            // 安全裕量只作为克制的虚线框提示，不干扰设备结构线。
            if (safety > 1e-6)
            {
                using (var safePen = new Pen(Color.FromArgb(150, 177, 184, 190), 1f))
                {
                    safePen.DashStyle = DashStyle.Dash;
                    float sx1 = X(minX - safety);
                    float sx2 = X(maxX + safety);
                    float sy1 = Y(topHeight + safety);
                    float sy2 = Y(0.0) + (float)(safety * scale);
                    g.DrawRectangle(safePen, sx1, sy1, sx2 - sx1, sy2 - sy1);
                }
            }

            // 集装箱：淡暖灰填充，仅保留少量结构分格。
            if (boxSideLength > 0.0 && box.Height > 0.0)
            {
                double boxBottom = 0.92;
                float bx1 = X(offsetX - boxSideLength * 0.5);
                float bx2 = X(offsetX + boxSideLength * 0.5);
                float by1 = Y(boxBottom + box.Height);
                float by2 = Y(boxBottom);
                using (var fill = new SolidBrush(Color.FromArgb(244, 241, 232)))
                using (var pen = new Pen(Color.FromArgb(129, 104, 69), 1.35f))
                using (var seam = new Pen(Color.FromArgb(205, 193, 171), 0.85f))
                {
                    g.FillRectangle(fill, bx1, by1, bx2 - bx1, by2 - by1);
                    g.DrawRectangle(pen, bx1, by1, bx2 - bx1, by2 - by1);
                    for (int k = 1; k < 6; k++)
                    {
                        float xx = bx1 + (bx2 - bx1) * k / 6f;
                        g.DrawLine(seam, xx, by1 + 2, xx, by2 - 2);
                    }
                }
            }

            float bodyLeft = X(-length * 0.5);
            float bodyRight = X(length * 0.5);
            float frontAxle = X(-wheelbase * 0.5);
            float rearAxle = X(wheelbase * 0.5);
            float topY = Y(topHeight);
            float beamH = Math.Max(8f, (float)(0.32 * scale));
            float legW = Math.Max(8f, (float)(0.34 * scale));

            using (var steelFill = new SolidBrush(Color.FromArgb(226, 233, 238)))
            using (var steelDark = new SolidBrush(Color.FromArgb(73, 91, 104)))
            using (var outline = new Pen(Color.FromArgb(55, 72, 84), 1.65f))
            using (var brace = new Pen(Color.FromArgb(100, 117, 129), 1.25f))
            {
                var beam = new RectangleF(bodyLeft, topY, bodyRight - bodyLeft, beamH);
                g.FillRectangle(steelFill, beam);
                g.DrawRectangle(outline, beam.X, beam.Y, beam.Width, beam.Height);

                float legTop = topY + beamH;
                float legBottom = Y(0.72);
                foreach (float axleX in new[] { frontAxle, rearAxle })
                {
                    var leg = new RectangleF(
                        axleX - legW * 0.5f, legTop, legW, legBottom - legTop);
                    g.FillRectangle(steelFill, leg);
                    g.DrawRectangle(outline, leg.X, leg.Y, leg.Width, leg.Height);
                }

                // 两根大尺度斜撑足以说明门架受力，不堆叠装饰性线条。
                g.DrawLine(brace, frontAxle + legW * 0.5f, legTop + beamH * 0.4f,
                    X(-length * 0.10), Y(2.1));
                g.DrawLine(brace, rearAxle - legW * 0.5f, legTop + beamH * 0.4f,
                    X(length * 0.10), Y(2.1));

                // 底部纵梁与吊具。
                float railY = Y(1.02);
                g.FillRectangle(steelDark, bodyLeft + 0.18f * (float)scale,
                    railY, bodyRight - bodyLeft - 0.36f * (float)scale,
                    Math.Max(4f, 0.15f * (float)scale));
                float spreaderY = box.Height > 0.0 ? Y(1.18 + box.Height) : Y(2.0);
                float spreaderX1 = boxSideLength > 0.0
                    ? X(offsetX - Math.Min(boxSideLength, length * 0.82) * 0.5)
                    : X(-length * 0.32);
                float spreaderX2 = boxSideLength > 0.0
                    ? X(offsetX + Math.Min(boxSideLength, length * 0.82) * 0.5)
                    : X(length * 0.32);
                g.DrawLine(outline, spreaderX1, spreaderY, spreaderX2, spreaderY);
                g.DrawLine(brace, X(-length * 0.18), topY + beamH, spreaderX1, spreaderY);
                g.DrawLine(brace, X(length * 0.18), topY + beamH, spreaderX2, spreaderY);

                // 简化驾驶室，保留体量与玻璃两个信息层级。
                float cabW = Math.Max(18f, (float)(0.78 * scale));
                float cabX = frontAxle + legW * 0.60f;
                float cabTop = Y(2.45), cabBottom = Y(1.18);
                using (var cab = new SolidBrush(Color.FromArgb(238, 242, 245)))
                using (var glass = new SolidBrush(Color.FromArgb(195, 215, 226)))
                {
                    g.FillRectangle(cab, cabX, cabTop, cabW, cabBottom - cabTop);
                    g.DrawRectangle(outline, cabX, cabTop, cabW, cabBottom - cabTop);
                    g.FillRectangle(glass, cabX + 3, cabTop + 4, cabW - 6,
                        Math.Max(5, (cabBottom - cabTop) * 0.38f));
                }
            }

            double wheelRadiusM = Math.Max(0.38, Math.Min(0.72, wheelLength * 0.42));
            DrawCarrierPreviewWheel(g, frontAxle, groundY, wheelRadiusM, scale);
            DrawCarrierPreviewWheel(g, rearAxle, groundY, wheelRadiusM, scale);

            using (var ground = new Pen(Color.FromArgb(164, 171, 177), 1f))
                g.DrawLine(ground, X(minX - 0.35), groundY, X(maxX + 0.35), groundY);

            using (var dimFont = new Font(Font.FontFamily, 8.0f, FontStyle.Regular))
            {
                DrawPreviewDimension(g, dimFont, bodyLeft, bodyRight, 66,
                    "整机 " + length.ToString("0.00") + " m");
                DrawPreviewDimension(g, dimFont, frontAxle, rearAxle, groundY + 25,
                    "轴距 " + wheelbase.ToString("0.00") + " m");
            }

            using (var footFont = new Font(Font.FontFamily, 7.8f, FontStyle.Regular))
            using (var footBrush = new SolidBrush(Color.FromArgb(116, 124, 131)))
            {
                string foot = string.Format(
                    "轮胎 {0:0.00}×{1:0.00} m   ·   箱体横偏 {2:+0.00;-0.00;0.00} m",
                    wheelLength, wheelWidth, offsetY);
                g.DrawString(foot, footFont, footBrush, 18, pnlPreview.Height - 25);
            }
        }

        void DrawCarrierPreviewWheel(
            Graphics g, float cx, float groundY, double radiusMeters, double scale)
        {
            float r = Math.Max(5f, (float)(radiusMeters * scale));
            float cy = groundY - r;
            using (var tire = new SolidBrush(Color.FromArgb(55, 63, 69)))
            using (var rim = new SolidBrush(Color.FromArgb(205, 211, 216)))
            using (var outline = new Pen(Color.FromArgb(43, 50, 56), 1.1f))
            {
                g.FillEllipse(tire, cx - r, cy - r, r * 2, r * 2);
                g.DrawEllipse(outline, cx - r, cy - r, r * 2, r * 2);
                float rr = r * 0.46f;
                g.FillEllipse(rim, cx - rr, cy - rr, rr * 2, rr * 2);
                g.DrawEllipse(outline, cx - rr, cy - rr, rr * 2, rr * 2);
            }
        }

        void DrawPreviewDimension(
            Graphics g, Font font, float x1, float x2, float y, string text)
        {
            if (x2 < x1) { float swap = x1; x1 = x2; x2 = swap; }
            if (x2 - x1 < 10) return;
            using (var pen = new Pen(Color.FromArgb(132, 141, 149), 0.9f))
            using (var brush = new SolidBrush(Color.FromArgb(82, 91, 98)))
            {
                g.DrawLine(pen, x1, y, x2, y);
                g.DrawLine(pen, x1, y - 4, x1, y + 4);
                g.DrawLine(pen, x2, y - 4, x2, y + 4);
                g.DrawLine(pen, x1, y, x1 + 5, y - 2);
                g.DrawLine(pen, x1, y, x1 + 5, y + 2);
                g.DrawLine(pen, x2, y, x2 - 5, y - 2);
                g.DrawLine(pen, x2, y, x2 - 5, y + 2);
                SizeF size = g.MeasureString(text, font);
                g.DrawString(text, font, brush, (x1 + x2 - size.Width) * 0.5f,
                    y - size.Height - 2);
            }
        }

        void DrawPreviewHint(Graphics g, string text)
        {
            using (var font = new Font(Font.FontFamily, 10F, FontStyle.Regular))
            using (var brush = new SolidBrush(Color.FromArgb(145, 153, 160)))
            using (var format = new StringFormat
            {
                Alignment = StringAlignment.Center,
                LineAlignment = StringAlignment.Center
            })
                g.DrawString(text, font, brush,
                    new RectangleF(0, 0, pnlPreview.Width, pnlPreview.Height), format);
        }

        CarrierContainerDimensions PreviewContainerDimensions()
        {
            switch ((CarrierContainerPreset)Math.Max(0, cboContainer.SelectedIndex))
            {
                case CarrierContainerPreset.None:
                    return new CarrierContainerDimensions(0.0, 0.0, 0.0);
                case CarrierContainerPreset.Iso20Gp:
                    return new CarrierContainerDimensions(6.058, 2.438, 2.591);
                case CarrierContainerPreset.Iso40Gp:
                    return new CarrierContainerDimensions(12.192, 2.438, 2.591);
                case CarrierContainerPreset.Iso40Hc:
                    return new CarrierContainerDimensions(12.192, 2.438, 2.896);
                case CarrierContainerPreset.Iso45Hc:
                    return new CarrierContainerDimensions(13.716, 2.438, 2.896);
                case CarrierContainerPreset.Twin20:
                    return new CarrierContainerDimensions(12.192, 2.438, 2.591);
                default:
                    return new CarrierContainerDimensions(
                        PreviewNumber(txtContainerLength, 0.0),
                        PreviewNumber(txtContainerWidth, 0.0),
                        PreviewNumber(txtContainerHeight, 0.0));
            }
        }

        static bool TryPreviewNumber(TextBox box, out double value, bool allowZero = false)
        {
            bool ok = double.TryParse(box.Text, NumberStyles.Float,
                          CultureInfo.CurrentCulture, out value) ||
                      double.TryParse(box.Text, NumberStyles.Float,
                          CultureInfo.InvariantCulture, out value);
            return ok && CarrierMath.Finite(value) && (allowZero ? value >= 0.0 : value > 0.0);
        }

        static double PreviewNumber(TextBox box, double fallback)
        {
            double value;
            return (double.TryParse(box.Text, NumberStyles.Float,
                        CultureInfo.CurrentCulture, out value) ||
                    double.TryParse(box.Text, NumberStyles.Float,
                        CultureInfo.InvariantCulture, out value)) && CarrierMath.Finite(value)
                ? value : fallback;
        }

        static TableLayoutPanel FormTable()
        {
            var table = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                Padding = new Padding(18),
                ColumnCount = 2,
                AutoSize = false
            };
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 190));
            table.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            return table;
        }

        static void AddRow(TableLayoutPanel table, string label, Control control)
        {
            int row = table.RowCount++;
            table.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            table.Controls.Add(new Label
            {
                Text = label,
                AutoSize = true,
                Margin = new Padding(3, 8, 8, 8)
            }, 0, row);
            control.Margin = new Padding(3, 4, 3, 4);
            control.Anchor = AnchorStyles.Left | AnchorStyles.Right;
            table.Controls.Add(control, 1, row);
        }

        static ComboBox Combo()
        {
            return new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 300 };
        }

        static TextBox Box() { return new TextBox { Width = 300 }; }
        static CheckBox Check(string text) { return new CheckBox { Text = text, AutoSize = true }; }
        static string Number(double value) { return value.ToString("0.###", CultureInfo.InvariantCulture); }

        static double Read(TextBox box, string name, bool positive = true)
        {
            double value;
            if (!double.TryParse(box.Text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) &&
                !double.TryParse(box.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                throw new InvalidOperationException(name + "不是有效数字。");
            if (!CarrierMath.Finite(value) || (positive && value <= 0.0))
                throw new InvalidOperationException(name + (positive ? "必须大于 0。" : "数值无效。"));
            return value;
        }

        sealed class DoubleBufferedPanel : Panel
        {
            public DoubleBufferedPanel()
            {
                DoubleBuffered = true;
                ResizeRedraw = true;
            }
        }
    }
}
