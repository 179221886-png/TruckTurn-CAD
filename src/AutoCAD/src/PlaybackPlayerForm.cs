using System;
using System.Drawing;
using System.Windows.Forms;

namespace TruckTurn
{
    /// <summary>UI-only transport. All CAD work is owned by the main-thread playback session.</summary>
    internal sealed class PlaybackPlayerForm : Form
    {
        readonly TrackBar progress = new TrackBar();
        readonly Label time = new Label();
        readonly Label status = new Label();
        readonly ComboBox speed = new ComboBox();
        readonly CheckBox loop = new CheckBox();
        bool updating;
        readonly double duration;
        public event Action PlayRequested;
        public event Action PauseRequested;
        public event Action StopRequested;
        public event Action<double> SeekRequested;
        public event Action<double> SpeedChanged;
        public bool Loop { get { return loop.Checked; } }

        public PlaybackPlayerForm(string name, double durationSeconds)
        {
            duration = durationSeconds;
            Text = "轨迹动画回放";
            ClientSize = new Size(490, 196);
            FormBorderStyle = FormBorderStyle.FixedToolWindow;
            MaximizeBox = false;
            MinimizeBox = false;
            ShowInTaskbar = false;
            StartPosition = FormStartPosition.CenterScreen;
            Font = new Font("Microsoft YaHei UI", 9F);
            BackColor = Color.FromArgb(247, 249, 251);

            var title = new Label { Left = 16, Top = 13, Width = 458, Height = 24,
                Text = string.IsNullOrEmpty(name) ? "已录制轨迹" : name, AutoEllipsis = true,
                Font = new Font(Font, FontStyle.Bold) };
            Controls.Add(title);
            progress.SetBounds(11, 43, 464, 38);
            progress.Minimum = 0;
            progress.Maximum = 10000;
            progress.TickStyle = TickStyle.None;
            progress.SmallChange = 100;
            progress.LargeChange = 500;
            progress.ValueChanged += delegate
            {
                if (!updating && SeekRequested != null)
                    SeekRequested(duration * progress.Value / 10000.0);
            };
            Controls.Add(progress);
            time.SetBounds(16, 82, 458, 21);
            Controls.Add(time);

            var play = Button("播放", 16, delegate { if (PlayRequested != null) PlayRequested(); });
            var pause = Button("暂停", 94, delegate { if (PauseRequested != null) PauseRequested(); });
            var stop = Button("停止", 172, delegate { if (StopRequested != null) StopRequested(); });
            Controls.Add(play); Controls.Add(pause); Controls.Add(stop);
            speed.SetBounds(262, 112, 83, 28);
            speed.DropDownStyle = ComboBoxStyle.DropDownList;
            speed.Items.AddRange(new object[] { "0.5×", "1×", "2×", "4×" });
            speed.SelectedIndex = 1;
            speed.SelectedIndexChanged += delegate
            {
                if (SpeedChanged != null)
                    SpeedChanged(new[] { 0.5, 1.0, 2.0, 4.0 }[speed.SelectedIndex]);
            };
            Controls.Add(speed);
            loop.SetBounds(364, 114, 110, 25);
            loop.Text = "循环播放";
            Controls.Add(loop);
            status.SetBounds(16, 157, 458, 26);
            status.ForeColor = Color.FromArgb(82, 95, 112);
            status.Text = "点击播放；不改动原图及包络。";
            Controls.Add(status);
            SetPosition(0, "就绪 · 示意速度，非实际行驶计时");
        }

        static Button Button(string text, int left, Action clicked)
        {
            var button = new Button { Text = text, Left = left, Top = 110, Width = 70, Height = 30,
                UseVisualStyleBackColor = true };
            button.Click += delegate { clicked(); };
            return button;
        }

        public void SetPosition(double seconds, string message)
        {
            if (IsDisposed) return;
            updating = true;
            progress.Value = duration <= 0 ? 0 : (int)Math.Round(
                Math.Max(0, Math.Min(1, seconds / duration)) * progress.Maximum);
            time.Text = FormatTime(seconds) + " / " + FormatTime(duration);
            if (message != null) status.Text = message;
            updating = false;
        }

        static string FormatTime(double seconds)
        {
            var span = TimeSpan.FromSeconds(Math.Max(0, seconds));
            return ((int)span.TotalMinutes).ToString("00") + ":" + span.Seconds.ToString("00")
                + "." + (span.Milliseconds / 100).ToString();
        }
    }
}
