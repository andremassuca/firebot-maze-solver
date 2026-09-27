// FIREBOT App: painel de controlo e telemetria do robô FIREBOT (Arduino Mega pela porta série)
// Fala com o firmware firebot_telemetria.ino (115200 baud).
// Compilar (sem instalar nada, usa o compilador que vem com o Windows):
//   compilar.bat
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Text;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Management;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;
using System.Windows.Forms;

namespace FirebotApp
{
    static class Cores
    {
        public static readonly Color Fundo = Color.FromArgb(6, 10, 9);
        public static readonly Color Painel = Color.FromArgb(13, 21, 18);
        public static readonly Color Radar = Color.FromArgb(3, 9, 6);
        public static readonly Color Linha = Color.FromArgb(32, 60, 50);
        public static readonly Color Texto = Color.FromArgb(214, 236, 225);
        public static readonly Color Fraco = Color.FromArgb(120, 154, 140);
        public static readonly Color Verde = Color.FromArgb(51, 255, 136);
        public static readonly Color VerdeEsc = Color.FromArgb(31, 140, 80);
        public static readonly Color Amarelo = Color.FromArgb(255, 210, 74);
        public static readonly Color Vermelho = Color.FromArgb(255, 77, 77);
        public static readonly Color Laranja = Color.FromArgb(255, 150, 50);
        public static readonly Color Azul = Color.FromArgb(40, 90, 170);
        public static readonly Color Botao = Color.FromArgb(18, 38, 29);
    }

    static class Fontes
    {
        public static readonly Font Titulo = new Font("Consolas", 9f, FontStyle.Bold);
        public static readonly Font Mono = new Font("Consolas", 9f);
        public static readonly Font MonoPeq = new Font("Consolas", 8f);
        public static readonly Font MonoGrande = new Font("Consolas", 13f, FontStyle.Bold);
        public static readonly Font Enorme = new Font("Consolas", 24f, FontStyle.Bold);
        public static readonly Font Estado = new Font("Segoe UI", 15f, FontStyle.Bold);
        public static readonly Font Normal = new Font("Segoe UI", 9.5f);
    }

    // Fontes das vistas desenhadas: em píxeis lógicos (a vista aplica a escala do ecrã)
    static class Fv
    {
        public static readonly Font Titulo = new Font("Consolas", 12f, FontStyle.Bold, GraphicsUnit.Pixel);
        public static readonly Font Mono = new Font("Consolas", 12.5f, GraphicsUnit.Pixel);
        public static readonly Font MonoPeq = new Font("Consolas", 11f, GraphicsUnit.Pixel);
        public static readonly Font MonoGrande = new Font("Consolas", 17f, FontStyle.Bold, GraphicsUnit.Pixel);
        public static readonly Font Enorme = new Font("Consolas", 32f, FontStyle.Bold, GraphicsUnit.Pixel);
        public static readonly Font Estado = new Font("Segoe UI", 20f, FontStyle.Bold, GraphicsUnit.Pixel);
    }

    static class Escala { public static float F = 1f; }

    struct Amostra { public DateTime T; public int FL, FR, FC, FI; }

    // Último estado recebido do robô
    class Telemetria
    {
        public int FL, FR, FC, FA, FD, FI, FB, FT = 100, ML, MR, Fan;
        public double Bat, BatRatio = 3.0, BatA, BatRatioA = 3.0, Hdg, Pit, Rol, Gz, Gb;
        public bool BatOk, BatAOk, ChamaOk, MpuOk, CentralOk, ChamaSim, Farol;
        public int Sirene;
        public string Estado = "-", Who = "0";
        public DateTime Quando = DateTime.MinValue;
        public int Pacotes;
        public readonly List<Amostra> Hist = new List<Amostra>();

        public bool Recente { get { return (DateTime.Now - Quando).TotalMilliseconds < 1000; } }
        public bool Chama { get { return Recente && ChamaOk && FI > FT; } }

        static readonly Regex Par = new Regex(@"([A-Z]+):(\S+)");

        public bool Aplicar(string linha)
        {
            MatchCollection ms = Par.Matches(linha);
            if (ms.Count < 5) return false;
            foreach (Match m in ms)
            {
                string k = m.Groups[1].Value, v = m.Groups[2].Value;
                switch (k)
                {
                    case "FL": FL = I(v); break;
                    case "FR": FR = I(v); break;
                    case "FC": FC = I(v); break;
                    case "FCOK": CentralOk = v == "1"; break;
                    case "FA": FA = I(v); break;
                    case "FD": FD = I(v); break;
                    case "FI": FI = I(v); break;
                    case "FB": FB = I(v); break;
                    case "FT": FT = I(v); break;
                    case "BAT": Bat = D(v); break;
                    case "BR": BatRatio = D(v); break;
                    case "BOK": BatOk = v == "1"; break;
                    case "BATA": BatA = D(v); break;
                    case "BRA": BatRatioA = D(v); break;
                    case "BAOK": BatAOk = v == "1"; break;
                    case "FOK": ChamaOk = v == "1"; break;
                    case "ST": Estado = v; break;
                    case "MPU": MpuOk = v == "1"; break;
                    case "WHO": Who = v; break;
                    case "HDG": Hdg = D(v); break;
                    case "PIT": Pit = D(v); break;
                    case "ROL": Rol = D(v); break;
                    case "GZ": Gz = D(v); break;
                    case "GB": Gb = D(v); break;
                    case "ML": ML = I(v); break;
                    case "MR": MR = I(v); break;
                    case "FAN": Fan = I(v); break;
                    case "SIM": ChamaSim = v == "1"; break;
                    case "FAR": Farol = v == "1"; break;
                    case "SIR": Sirene = I(v); break;
                }
            }
            Quando = DateTime.Now;
            Pacotes++;
            Hist.Add(new Amostra { T = Quando, FL = FL, FR = FR, FC = CentralOk ? FC : -1, FI = ChamaOk ? FI : -1 });
            if (Hist.Count > 700) Hist.RemoveRange(0, 100);
            return true;
        }

        static int I(string s) { int r; int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out r); return r; }
        static double D(string s) { double r; double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out r); return r; }
    }

    // Definições guardadas em firebot_app.ini, ao lado do .exe
    class Config
    {
        public double Alcance = 100, Angulo = 28, Afastamento = 5, Cone = 15, VMax = 40; // ângulo medido nas fotos
        public bool InverterGiro = false;
        public string Porta = "", NomeBle = "BT24";

        string Ficheiro { get { return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "firebot_app.ini"); } }

        public void Carregar()
        {
            try
            {
                if (!File.Exists(Ficheiro)) return;
                foreach (string l in File.ReadAllLines(Ficheiro))
                {
                    int i = l.IndexOf('=');
                    if (i < 0) continue;
                    string k = l.Substring(0, i).Trim(), v = l.Substring(i + 1).Trim();
                    double d;
                    bool num = double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out d);
                    switch (k)
                    {
                        case "alcance": if (num) Alcance = d; break;
                        case "angulo": if (num) Angulo = d; break;
                        case "afastamento": if (num) Afastamento = d; break;
                        case "cone": if (num) Cone = d; break;
                        case "vmax": if (num) VMax = d; break;
                        case "inverter": InverterGiro = v == "1"; break;
                        case "porta": Porta = v; break;
                        case "ble": NomeBle = v; break;
                    }
                }
            }
            catch { }
        }

        public void Guardar()
        {
            try
            {
                File.WriteAllLines(Ficheiro, new string[] {
                    "alcance=" + F(Alcance), "angulo=" + F(Angulo), "afastamento=" + F(Afastamento),
                    "cone=" + F(Cone), "vmax=" + F(VMax), "inverter=" + (InverterGiro ? "1" : "0"), "porta=" + Porta, "ble=" + NomeBle });
            }
            catch { }
        }

        static string F(double d) { return d.ToString(CultureInfo.InvariantCulture); }
    }

    // Posição e direção estimadas (sem encoders): integra os comandos dos motores
    class Movimento
    {
        public double X, Y, H, FaseL, FaseR, FaseFan;
        public bool HeadingDoMpu;
        public readonly List<PointF> Trajeto = new List<PointF>();

        public void Limpar() { X = 0; Y = 0; Trajeto.Clear(); Trajeto.Add(new PointF(0, 0)); }

        public void Atualizar(Telemetria t, Config cfg, double dt)
        {
            const double largura = 15.0; // cm entre rodas
            double vL = 0, vR = 0;
            if (t.Recente) { vL = t.ML / 255.0 * cfg.VMax; vR = t.MR / 255.0 * cfg.VMax; }
            HeadingDoMpu = t.Recente && t.MpuOk;
            if (HeadingDoMpu) H = cfg.InverterGiro ? 360 - t.Hdg : t.Hdg;
            else H += (vL - vR) / largura * dt * 180 / Math.PI; // rodar para a direita = positivo
            H = ((H % 360) + 360) % 360;
            double v = (vL + vR) / 2, hr = H * Math.PI / 180;
            X += v * Math.Sin(hr) * dt;
            Y += v * Math.Cos(hr) * dt;
            FaseL += vL * dt; FaseR += vR * dt;
            if (t.Recente) FaseFan += t.Fan / 255.0 * 1440 * dt;
            PointF ult = Trajeto.Count > 0 ? Trajeto[Trajeto.Count - 1] : new PointF(float.NaN, 0);
            if (Trajeto.Count == 0 || Math.Abs(ult.X - X) + Math.Abs(ult.Y - Y) > 0.5)
            {
                Trajeto.Add(new PointF((float)X, (float)Y));
                if (Trajeto.Count > 4000) Trajeto.RemoveRange(0, 1000);
            }
        }
    }

    static class Desenho
    {
        public static void Texto(Graphics g, string s, Font f, Color c, float x, float y)
        {
            using (Brush b = new SolidBrush(c)) g.DrawString(s, f, b, x, y);
        }

        public static void TextoCentro(Graphics g, string s, Font f, Color c, float x, float y)
        {
            SizeF sz = g.MeasureString(s, f);
            Texto(g, s, f, c, x - sz.Width / 2, y - sz.Height / 2);
        }

        public static GraphicsPath Arredondado(RectangleF r, float raio)
        {
            GraphicsPath p = new GraphicsPath();
            float d = Math.Max(1, Math.Min(raio * 2, Math.Min(r.Width, r.Height)));
            p.AddArc(r.X, r.Y, d, d, 180, 90);
            p.AddArc(r.Right - d, r.Y, d, d, 270, 90);
            p.AddArc(r.Right - d, r.Bottom - d, d, d, 0, 90);
            p.AddArc(r.X, r.Bottom - d, d, d, 90, 90);
            p.CloseFigure();
            return p;
        }

        public static Color Alfa(Color c, int a) { return Color.FromArgb(Math.Max(0, Math.Min(255, a)), c); }

        public static Color CorRoda(int v)
        {
            if (v > 0) return Cores.Verde;
            if (v < 0) return Cores.Vermelho;
            return Color.FromArgb(70, 80, 76);
        }

        // Robô visto de cima (réplica do FIREBOT das fotos), centrado na origem, frente para cima (-y).
        // sc = píxeis por cm. Tudo o que muda (rodas, ventoinha, chama, MPU, sonares) é animado.
        public static void Robo(Graphics g, float sc, Telemetria t, Config cfg, Movimento mv)
        {
            bool rec = t.Recente;
            int tick = Environment.TickCount;

            // 1) sombra, desloca-se com a inclinação do MPU
            float sx = 1.0f, sy = 1.4f;
            if (rec && t.MpuOk) { sx += (float)Math.Max(-3, Math.Min(3, t.Rol / 8)); sy += (float)Math.Max(-3, Math.Min(3, t.Pit / 8)); }
            for (int i = 3; i >= 1; i--)
                using (GraphicsPath p = Arredondado(new RectangleF((-11.8f + sx - i * 0.45f) * sc, (-16.8f + sy - i * 0.45f) * sc, (23.6f + i * 0.9f) * sc, (30f + i * 0.9f) * sc), (3 + i * 0.5f) * sc))
                using (Brush b = new SolidBrush(Color.FromArgb(30, 0, 0, 0))) g.FillPath(b, p);

            // 2) chassis branco por baixo
            RectangleF chassis = new RectangleF(-7.2f * sc, -9.5f * sc, 14.4f * sc, 22.5f * sc);
            using (GraphicsPath p = Arredondado(chassis, 1.2f * sc))
            {
                using (Brush b = new LinearGradientBrush(chassis, Color.FromArgb(236, 236, 230), Color.FromArgb(180, 180, 174), 0f)) g.FillPath(b, p);
                using (Pen pn = new Pen(Color.FromArgb(140, 140, 135), 1f)) g.DrawPath(pn, p);
            }

            // 3) motores TT amarelos e 4) rodas
            for (int lado = -1; lado <= 1; lado += 2)
            {
                RectangleF mot = new RectangleF(lado < 0 ? -8.6f * sc : 6.2f * sc, 5.0f * sc, 2.4f * sc, 6.2f * sc);
                using (Brush b = new LinearGradientBrush(mot, Color.FromArgb(250, 205, 40), Color.FromArgb(200, 150, 20), 0f)) g.FillRectangle(b, mot);
                int v = rec ? (lado < 0 ? t.ML : t.MR) : 0;
                Roda(g, sc, lado < 0 ? -11.6f : 8.6f, v, lado < 0 ? mv.FaseL : mv.FaseR);
            }

            // 5) placa de acrílico âmbar com a frente em arco
            using (GraphicsPath placa = new GraphicsPath())
            {
                placa.AddLine(-8.8f * sc, 13.2f * sc, -8.8f * sc, -3.4f * sc);
                placa.AddLine(-8.8f * sc, -3.4f * sc, -11.3f * sc, -3.4f * sc);
                placa.AddBezier(-11.3f * sc, -3.4f * sc, -10.6f * sc, -9.8f * sc, -5.2f * sc, -11.8f * sc, 0, -11.8f * sc);
                placa.AddBezier(0, -11.8f * sc, 5.2f * sc, -11.8f * sc, 10.6f * sc, -9.8f * sc, 11.3f * sc, -3.4f * sc);
                placa.AddLine(11.3f * sc, -3.4f * sc, 8.8f * sc, -3.4f * sc);
                placa.AddLine(8.8f * sc, -3.4f * sc, 8.8f * sc, 13.2f * sc);
                placa.CloseFigure();
                RectangleF bb = placa.GetBounds();
                using (Brush b = new LinearGradientBrush(bb, Color.FromArgb(232, 236, 184, 70), Color.FromArgb(232, 186, 132, 32), 60f)) g.FillPath(b, placa);
                using (Pen pn = new Pen(Color.FromArgb(150, 110, 25), 1.2f)) g.DrawPath(pn, placa);
                // brilho do acrílico
                using (Pen pn = new Pen(Color.FromArgb(70, 255, 245, 200), 0.5f * sc))
                    g.DrawBezier(pn, -9.8f * sc, -4.4f * sc, -9f * sc, -9f * sc, -5f * sc, -10.9f * sc, -1f * sc, -10.9f * sc);
            }
            using (Brush furo = new SolidBrush(Color.FromArgb(150, 45, 35, 20)))
            using (Pen ranhura = new Pen(Color.FromArgb(150, 45, 35, 20), 0.7f * sc) { StartCap = LineCap.Round, EndCap = LineCap.Round })
            {
                g.DrawLine(ranhura, -9.4f * sc, -5.2f * sc, -5.2f * sc, -8.6f * sc);
                g.DrawLine(ranhura, 9.4f * sc, -5.2f * sc, 5.2f * sc, -8.6f * sc);
                g.FillEllipse(furo, -6.4f * sc, -7.2f * sc, 2.8f * sc, 1.4f * sc);
                g.FillEllipse(furo, 3.6f * sc, -7.2f * sc, 2.8f * sc, 1.4f * sc);
                // cruzes por onde se vêem as pilhas
                foreach (float cx in new float[] { -4f, 4f })
                {
                    g.FillRectangle(furo, (cx - 1.4f) * sc, 0.5f * sc, 2.8f * sc, 0.9f * sc);
                    g.FillRectangle(furo, (cx - 0.45f) * sc, -0.5f * sc, 0.9f * sc, 2.9f * sc);
                }
                foreach (PointF f in new PointF[] { new PointF(-5.8f, -2f), new PointF(5.8f, -2f), new PointF(-5.8f, 4f), new PointF(5.8f, 4f), new PointF(0, 2.6f), new PointF(-2.4f, 3.6f), new PointF(2.4f, 3.6f) })
                    g.FillEllipse(furo, (f.X - 0.35f) * sc, (f.Y - 0.35f) * sc, 0.7f * sc, 0.7f * sc);
            }

            // cabo colorido da frente até ao Mega
            Color[] fios = { Color.Brown, Color.Red, Color.Orange, Color.Gold, Color.LimeGreen, Color.DodgerBlue, Color.MediumPurple, Color.Gray };
            for (int i = 0; i < fios.Length; i++)
                using (Pen pn = new Pen(fios[i], Math.Max(1f, 0.22f * sc)))
                {
                    float x = (-0.8f + i * 0.23f) * sc;
                    g.DrawBezier(pn, x, -8.5f * sc, x, -2f * sc, x + 0.3f * sc, 2f * sc, x - 0.6f * sc, 7.2f * sc);
                }

            // 6) eletrónica na traseira: Mega, breadboard e MPU
            RectangleF mega = new RectangleF(-5.1f * sc, 7.2f * sc, 10.2f * sc, 5.3f * sc);
            using (Brush b = new LinearGradientBrush(mega, Color.FromArgb(28, 92, 165), Color.FromArgb(16, 60, 120), 90f)) g.FillRectangle(b, mega);
            using (Pen pn = new Pen(Color.FromArgb(90, 150, 220), 1f)) g.DrawRectangle(pn, mega.X, mega.Y, mega.Width, mega.Height);
            using (Brush b = new SolidBrush(Color.FromArgb(20, 20, 20)))
            {
                g.FillRectangle(b, -4.6f * sc, 7.4f * sc, 9.2f * sc, 0.45f * sc);
                g.FillRectangle(b, 4.3f * sc, 7.9f * sc, 0.45f * sc, 4.2f * sc);
            }
            using (Brush b = new SolidBrush(Color.FromArgb(190, 190, 195))) g.FillRectangle(b, -6.0f * sc, 8.0f * sc, 1.4f * sc, 1.3f * sc); // USB
            RectangleF bread = new RectangleF(-3.2f * sc, 8.1f * sc, 5.6f * sc, 3.8f * sc);
            using (Brush b = new SolidBrush(Color.FromArgb(240, 240, 236))) g.FillRectangle(b, bread);
            if (sc > 4)
                using (Brush b = new SolidBrush(Color.FromArgb(150, 150, 150)))
                    for (float yy = 8.5f; yy < 11.8f; yy += 0.5f)
                        for (float xx = -2.9f; xx < 2.2f; xx += 0.5f)
                            g.FillRectangle(b, xx * sc, yy * sc, Math.Max(1, 0.12f * sc), Math.Max(1, 0.12f * sc));
            RectangleF mpu = new RectangleF(0.2f * sc, 8.5f * sc, 2.0f * sc, 1.7f * sc);
            using (Brush b = new SolidBrush(Color.FromArgb(110, 60, 150))) g.FillRectangle(b, mpu);
            if (rec && t.MpuOk) Brilho(g, 1.9f * sc, 8.8f * sc, 0.9f * sc, Color.FromArgb(80, 255, 90));
            using (Brush b = new SolidBrush(rec && t.MpuOk ? Color.FromArgb(120, 255, 120) : Color.FromArgb(40, 60, 40)))
                g.FillEllipse(b, 1.7f * sc, 8.6f * sc, 0.4f * sc, 0.4f * sc);

            // 7) ventoinha (gira com a potência enviada)
            Ventoinha(g, sc, 0, -5.6f, 4.2f, rec ? t.Fan : 0, mv.FaseFan);

            // 8) suporte preto impresso em 3D na frente
            PointF[] suporte = {
                new PointF(-2.6f * sc, -10.4f * sc), new PointF(-9.9f * sc, -13.6f * sc), new PointF(-8.3f * sc, -17.6f * sc),
                new PointF(0, -14.2f * sc), new PointF(8.3f * sc, -17.6f * sc), new PointF(9.9f * sc, -13.6f * sc), new PointF(2.6f * sc, -10.4f * sc) };
            using (Brush b = new SolidBrush(Color.FromArgb(30, 30, 33))) g.FillPolygon(b, suporte);
            using (Pen pn = new Pen(Color.FromArgb(70, 70, 76), 1f)) g.DrawPolygon(pn, suporte);

            // 9) sonares HC-SR04 em ângulo, com as "ondas" a sair quando há leitura
            float af = (float)cfg.Afastamento, ang = (float)cfg.Angulo;
            Sonar(g, sc, -af - 0.3f, -14.6f, -ang, rec && t.FL > 0, tick);
            Sonar(g, sc, af + 0.3f, -14.6f, ang, rec && t.FR > 0, tick + 400);

            // 9b) sonar central, por baixo do suporte, a olhar em frente
            if (rec && t.CentralOk) Sonar(g, sc, 0, -17.4f, 0, t.FC > 0, tick + 200);

            // 10) KY-026 ao meio, com o LED de IR a brilhar quando há chama
            RectangleF ky = new RectangleF(-0.95f * sc, -15.6f * sc, 1.9f * sc, 4.6f * sc);
            using (Brush b = new SolidBrush(Color.FromArgb(205, 35, 35))) g.FillRectangle(b, ky);
            using (Brush b = new SolidBrush(Color.FromArgb(40, 80, 200))) g.FillRectangle(b, -0.6f * sc, -13.6f * sc, 1.0f * sc, 1.0f * sc);
            using (Brush b = new SolidBrush(Color.FromArgb(230, 190, 60)))
                for (int i = 0; i < 4; i++) g.FillRectangle(b, (-0.75f + i * 0.45f) * sc, -11.6f * sc, 0.2f * sc, 0.5f * sc);
            if (t.Chama)
            {
                float pulso = 3.2f + 0.8f * (float)Math.Sin(tick / 120.0);
                Brilho(g, 0, -16.4f * sc, pulso * sc, Color.FromArgb(255, 150, 40));
            }
            using (Brush b = new SolidBrush(t.Chama ? Color.FromArgb(255, 200, 90) : Color.FromArgb(18, 18, 20)))
                g.FillEllipse(b, -0.6f * sc, -17.0f * sc, 1.2f * sc, 1.4f * sc);

            // seta discreta da frente
            using (Pen pn = new Pen(Color.FromArgb(120, 20, 60, 30), Math.Max(1.5f, 0.35f * sc)) { StartCap = LineCap.Round, EndCap = LineCap.Round })
            {
                g.DrawLine(pn, -1.2f * sc, 5.6f * sc, 0, 4.4f * sc);
                g.DrawLine(pn, 1.2f * sc, 5.6f * sc, 0, 4.4f * sc);
            }
        }

        public static void Brilho(Graphics g, float x, float y, float r, Color c)
        {
            if (r < 1) return;
            using (GraphicsPath p = new GraphicsPath())
            {
                p.AddEllipse(x - r, y - r, 2 * r, 2 * r);
                using (PathGradientBrush b = new PathGradientBrush(p))
                {
                    b.CenterColor = Color.FromArgb(200, c);
                    b.SurroundColors = new Color[] { Color.FromArgb(0, c) };
                    g.FillPath(b, p);
                }
            }
        }

        // Pneu visto de cima: faixa de rodagem com o piso a andar e tom verde/vermelho conforme o sentido
        static void Roda(Graphics g, float sc, float x, int v, double fase)
        {
            RectangleF r = new RectangleF(x * sc, 4.6f * sc, 3.0f * sc, 6.8f * sc);
            using (GraphicsPath p = Arredondado(r, 0.9f * sc))
            {
                using (LinearGradientBrush b = new LinearGradientBrush(r, Color.Black, Color.Black, 90f))
                {
                    ColorBlend cb = new ColorBlend();
                    cb.Colors = new Color[] { Color.FromArgb(10, 10, 10), Color.FromArgb(58, 58, 58), Color.FromArgb(10, 10, 10) };
                    cb.Positions = new float[] { 0f, 0.5f, 1f };
                    b.InterpolationColors = cb;
                    g.FillPath(b, p);
                }
                Region antes = g.Clip;
                g.SetClip(p, CombineMode.Intersect);
                float passo = 1.1f;
                float off = (float)(((-fase % passo) + passo) % passo);
                using (Pen pn = new Pen(Color.FromArgb(95, 95, 95), Math.Max(1f, 0.22f * sc)))
                    for (float y = 4.6f - passo + off; y < 11.6f; y += passo)
                    {
                        g.DrawLine(pn, r.Left, (y + 0.35f) * sc, r.Left + r.Width / 2, y * sc);
                        g.DrawLine(pn, r.Left + r.Width / 2, y * sc, r.Right, (y + 0.35f) * sc);
                    }
                if (v != 0)
                    using (Brush b = new SolidBrush(Alfa(CorRoda(v), 55))) g.FillPath(b, p);
                g.Clip = antes;
                using (Pen pn = new Pen(v == 0 ? Color.FromArgb(70, 70, 70) : CorRoda(v), 1.2f)) g.DrawPath(pn, p);
            }
        }

        // HC-SR04: placa azul, cristal e dois transdutores prateados. ang em graus para fora (negativo = esquerda).
        static void Sonar(Graphics g, float sc, float x, float y, float ang, bool ativo, int tick)
        {
            GraphicsState st = g.Save();
            g.TranslateTransform(x * sc, y * sc);
            g.RotateTransform(ang);
            if (ativo)
            {
                for (int k = 0; k < 2; k++)
                {
                    float f = ((tick / 900f) + k * 0.5f) % 1f;
                    float rr = (1.5f + f * 6f) * sc;
                    using (Pen pn = new Pen(Color.FromArgb((int)(150 * (1 - f)), 80, 255, 150), Math.Max(1f, 0.25f * sc)))
                        g.DrawArc(pn, -rr, -1.4f * sc - rr, 2 * rr, 2 * rr, 240, 60);
                }
            }
            RectangleF placa = new RectangleF(-2.25f * sc, -1.0f * sc, 4.5f * sc, 2.0f * sc);
            using (Brush b = new LinearGradientBrush(placa, Color.FromArgb(40, 95, 190), Color.FromArgb(22, 55, 125), 90f)) g.FillRectangle(b, placa);
            using (Brush b = new SolidBrush(Color.FromArgb(210, 210, 215))) g.FillRectangle(b, -0.45f * sc, -0.2f * sc, 0.9f * sc, 0.5f * sc);
            foreach (float cx in new float[] { -1.2f, 1.2f })
            {
                RectangleF tr = new RectangleF((cx - 0.8f) * sc, -1.85f * sc, 1.6f * sc, 1.6f * sc);
                using (Brush b = new LinearGradientBrush(tr, Color.FromArgb(235, 235, 240), Color.FromArgb(150, 150, 158), 45f)) g.FillEllipse(b, tr);
                RectangleF dentro = new RectangleF((cx - 0.5f) * sc, -1.55f * sc, 1.0f * sc, 1.0f * sc);
                using (Brush b = new SolidBrush(Color.FromArgb(60, 60, 64))) g.FillEllipse(b, dentro);
                using (Pen pn = new Pen(Color.FromArgb(110, 110, 116), Math.Max(0.5f, 0.08f * sc)))
                    g.DrawEllipse(pn, (cx - 0.3f) * sc, -1.35f * sc, 0.6f * sc, 0.6f * sc);
            }
            g.Restore(st);
        }

        // Ventoinha vista de cima: moldura quadrada e 5 pás que rodam com a potência
        public static void Ventoinha(Graphics g, float sc, float x, float y, float lado, int pot, double fase)
        {
            RectangleF r = new RectangleF((x - lado / 2) * sc, (y - lado / 2) * sc, lado * sc, lado * sc);
            using (GraphicsPath p = Arredondado(r, 0.5f * sc))
            {
                using (Brush b = new SolidBrush(Color.FromArgb(24, 24, 26))) g.FillPath(b, p);
                using (Pen pn = new Pen(pot > 0 ? Cores.Verde : Color.FromArgb(70, 70, 74), 1f)) g.DrawPath(pn, p);
            }
            float cx = x * sc, cy = y * sc, rr = lado * 0.44f * sc;
            GraphicsState st = g.Save();
            g.TranslateTransform(cx, cy);
            g.RotateTransform((float)(fase % 360));
            using (Brush b = new SolidBrush(pot > 0 ? Color.FromArgb(90, 95, 100) : Color.FromArgb(60, 62, 66)))
                for (int i = 0; i < 5; i++)
                {
                    g.RotateTransform(72);
                    g.FillPie(b, -rr, -rr, 2 * rr, 2 * rr, 0, 42);
                }
            g.Restore(st);
            using (Brush b = new SolidBrush(Color.FromArgb(35, 35, 38))) g.FillEllipse(b, cx - rr * 0.35f, cy - rr * 0.35f, rr * 0.7f, rr * 0.7f);
            if (pot > 0)
                using (Pen pn = new Pen(Color.FromArgb(Math.Min(200, 60 + pot / 2), 120, 200, 255), Math.Max(1f, 0.2f * sc)))
                    for (int i = -1; i <= 1; i++)
                    {
                        float ax = cx + i * lado * 0.3f * sc, ay = cy - lado * 0.62f * sc;
                        float onda = (float)((fase / 60.0) % 1.0) * sc;
                        g.DrawLine(pn, ax, ay - onda, ax, ay - onda - 1.2f * sc);
                    }
        }
    }

    class Vista : Control
    {
        protected Telemetria t; protected Config cfg; protected Movimento mv;
        public float Zoom = 1f;                                  // o modo apresentação desenha tudo maior
        protected float Ef { get { return Escala.F * Zoom; } }
        public Vista(Telemetria t, Config cfg, Movimento mv)
        {
            this.t = t; this.cfg = cfg; this.mv = mv;
            SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint |
                     ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
            Dock = DockStyle.Fill;
            BackColor = Cores.Painel;
        }
        protected Graphics Preparar(PaintEventArgs e, Color fundo)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;
            g.Clear(fundo);
            g.ScaleTransform(Ef, Ef);
            return g;
        }
    }

    // ---------------- Radar ----------------
    class RadarView : Vista
    {
        float ox, oy, s;
        public RadarView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        PointF P(double x, double y) { return new PointF((float)(ox + x * s), (float)(oy - y * s)); }
        PointF Polar(double x0, double y0, double ang, double r)
        {
            double a = ang * Math.PI / 180;
            return P(x0 + r * Math.Cos(a), y0 + r * Math.Sin(a));
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Radar);
            double alc = Math.Max(20, cfg.Alcance);
            float W = Width / Ef, H = Height / Ef;
            float hg = H >= 360 ? Math.Max(80, Math.Min(H * 0.26f, 190)) : 0; // histórico só se o radar continuar com espaço
            float Hr = H - hg;
            s = (float)Math.Min((Hr - 92) / (alc + 27), (W - 120) / (2 * alc));
            if (s <= 0.05f) return;
            ox = W / 2; oy = (float)(76 + alc * s);

            double yc = -1.5;
            bool central = t.Recente && t.CentralOk;
            if (!central) ZonaCega(g, yc, alc);
            else Cone(g, "FC", t.FC, 90, 0, yc - 0.5, alc);
            Cone(g, "FL", t.FL, 90 + cfg.Angulo, -cfg.Afastamento, yc, alc);
            Cone(g, "FR", t.FR, 90 - cfg.Angulo, cfg.Afastamento, yc, alc);

            // grelha por cima dos cones
            using (Pen p = new Pen(Desenho.Alfa(Cores.VerdeEsc, 150), 1.2f))
            {
                for (int i = 1; i <= 4; i++)
                {
                    float r = (float)(alc * i / 4 * s);
                    g.DrawArc(p, ox - r, oy - r, 2 * r, 2 * r, 180, 180);
                    Desenho.Texto(g, string.Format("{0:0} cm", alc * i / 4), Fv.MonoPeq, Cores.Fraco, ox + r + 3, oy + 2);
                }
                float R = (float)(alc * s);
                g.DrawLine(p, ox - R - 12, oy, ox + R + 12, oy);
                for (int a = 30; a <= 150; a += 30)
                {
                    PointF q = Polar(0, 0, a, alc);
                    g.DrawLine(p, ox, oy, q.X, q.Y);
                    PointF lq = Polar(0, 0, a, alc + 16 / s);
                    Desenho.TextoCentro(g, (a - 90) == 0 ? "0°" : string.Format("{0}°", 90 - a), Fv.MonoPeq, Cores.Fraco, lq.X, lq.Y);
                }
            }

            // robô (frente na origem)
            GraphicsState st = g.Save();
            g.TranslateTransform(ox, oy + 13.1f * s);
            Desenho.Robo(g, s, t, cfg, mv);
            g.Restore(st);

            // legendas
            Leitura(g, "FL (esquerdo)", t.FL, 12, 8, false);
            Leitura(g, "FR (direito)", t.FR, W - 12, 8, true);
            if (t.Recente && t.CentralOk)
            {
                string vc = t.FC <= 0 ? "sem eco" : t.FC + " cm";
                Desenho.TextoCentro(g, "FC (centro, em baixo)", Fv.Mono, Cores.Fraco, W / 2, 16);
                Desenho.TextoCentro(g, vc, Fv.MonoGrande, t.FC > 0 ? Cores.Amarelo : Cores.Fraco, W / 2, 36);
            }
            else if (t.Recente)
            {
                // o robô diz que o sonar central não responde: dizê-lo em vez de só mostrar a zona cega
                Desenho.TextoCentro(g, "FC (centro): sem sinal", Fv.Mono, Cores.Laranja, W / 2, 16);
                Desenho.TextoCentro(g, "ver fios: 5V, GND, Trig D26, Echo D27", Fv.MonoPeq, Cores.Fraco, W / 2, 36);
            }
            // aviso de obstáculo perto: ocupa o lugar da linha informativa
            int perto = int.MaxValue;
            if (t.Recente) { if (t.FL > 0) perto = Math.Min(perto, t.FL); if (t.FR > 0) perto = Math.Min(perto, t.FR); if (t.CentralOk && t.FC > 0) perto = Math.Min(perto, t.FC); }
            if (perto >= 20)
            {
                string info = t.Recente ? string.Format("sonares a {0:0}° para fora, afastados {1:0} cm do centro", cfg.Angulo, cfg.Afastamento) + (Zoom > 1 ? "" : "   |   duplo clique para ampliar") : "sem dados do robô";
                Desenho.TextoCentro(g, info, Fv.MonoPeq, Cores.Fraco, W / 2, Hr - 10);
            }
            else
            {
                // em baixo, à esquerda do robô: não tapa os ângulos nem as leituras
                float bw = Math.Min(240, W / 2 - 60), cx = W / 2 - 55 - bw / 2, cy = Math.Max(oy + 30, Hr - 22);
                RectangleF av = new RectangleF(cx - bw / 2, cy - 14, bw, 28);
                bool pisca = (Environment.TickCount / 300) % 2 == 0;
                using (GraphicsPath p = Desenho.Arredondado(av, 6))
                {
                    using (Brush b = new SolidBrush(Desenho.Alfa(Cores.Vermelho, pisca ? 90 : 50))) g.FillPath(b, p);
                    using (Pen pn = new Pen(Cores.Vermelho, 1.5f)) g.DrawPath(pn, p);
                }
                Desenho.TextoCentro(g, string.Format("OBSTÁCULO A {0} cm", perto), bw < 220 ? Fv.Mono : Fv.MonoGrande, Cores.Vermelho, cx, cy);
            }

            if (hg > 0) Historico(g, new RectangleF(14, Hr + 8, W - 28, hg - 16));
        }

        // Entre as bordas interiores dos dois cones fica uma faixa que nenhum sonar vê
        void ZonaCega(Graphics g, double yc, double alc)
        {
            double folga = cfg.Angulo - cfg.Cone; // graus entre a borda interior do cone e a frente
            if (folga <= 0) return;
            PointF o1 = Polar(-cfg.Afastamento, yc, 90 + cfg.Angulo, 1.0), o2 = Polar(cfg.Afastamento, yc, 90 - cfg.Angulo, 1.0);
            double a1 = 90 + folga, a2 = 90 - folga;
            // até onde a faixa cabe dentro do radar
            double d = alc / Math.Cos(folga * Math.PI / 180);
            PointF p1 = Polar(-cfg.Afastamento, yc + 1, a1, d), p2 = Polar(cfg.Afastamento, yc + 1, a2, d);
            PointF[] zona = { o1, p1, p2, o2 };
            using (Region clip = new Region(new RectangleF(0, 0, Width, oy)))
            {
                Region antes = g.Clip;
                g.SetClip(clip, CombineMode.Intersect);
                using (HatchBrush hb = new HatchBrush(HatchStyle.WideUpwardDiagonal, Desenho.Alfa(Cores.Amarelo, 45), Color.Transparent))
                    g.FillPolygon(hb, zona);
                using (Pen pn = new Pen(Desenho.Alfa(Cores.Amarelo, 110), 1f) { DashStyle = DashStyle.Dash })
                { g.DrawLine(pn, o1, p1); g.DrawLine(pn, o2, p2); }
                g.Clip = antes;
            }
            double larg50 = 2 * cfg.Afastamento + 2 * 50 * Math.Tan(folga * Math.PI / 180);
            PointF l = P(0, alc * 0.86);   // junto à borda: as leituras dos sonares ficam mais perto do robô
            Desenho.TextoCentro(g, "ZONA CEGA", Fv.Mono, Desenho.Alfa(Cores.Amarelo, 220), l.X, l.Y);
            Desenho.TextoCentro(g, string.Format("{0:0} cm de largura a 50 cm", larg50), Fv.MonoPeq, Desenho.Alfa(Cores.Amarelo, 200), l.X, l.Y + 16);
        }

        void Historico(Graphics g, RectangleF r)
        {
            Color cFL = Cores.Verde, cFR = Color.FromArgb(90, 200, 255), cFC = Cores.Amarelo;
            using (Brush b = new SolidBrush(Color.FromArgb(8, 18, 12))) g.FillRectangle(b, r);
            using (Pen pn = new Pen(Cores.Linha)) g.DrawRectangle(pn, r.X, r.Y, r.Width, r.Height);
            // faixa de cima só para o título e a legenda; coluna da esquerda só para os números do eixo
            const float topo = 22, eixo = 30;
            RectangleF a = new RectangleF(r.X + eixo, r.Y + topo, r.Width - eixo - 6, r.Height - topo - 6);
            if (a.Height < 20 || a.Width < 40) return;
            using (Pen pn = new Pen(Cores.Linha)) g.DrawLine(pn, r.X, a.Y, r.Right, a.Y);
            Desenho.Texto(g, "Histórico (últimos 20 s)", Fv.MonoPeq, Cores.Fraco, r.X + 6, r.Y + 5);
            Desenho.Texto(g, "━ FL", Fv.MonoPeq, cFL, r.Right - 150, r.Y + 5);
            Desenho.Texto(g, "━ FR", Fv.MonoPeq, cFR, r.Right - 100, r.Y + 5);
            Desenho.Texto(g, "━ FC", Fv.MonoPeq, cFC, r.Right - 50, r.Y + 5);

            double alc = Math.Max(20, cfg.Alcance), janela = 20; // segundos
            using (Pen pn = new Pen(Color.FromArgb(22, 44, 34)))
                for (int i = 0; i <= 4; i++)
                {
                    float y = a.Bottom - a.Height * i / 4f;
                    if (i > 0 && i < 4) g.DrawLine(pn, a.X, y, a.Right, y);
                    bool mostra = a.Height >= 70 ? i > 0 : (i == 2 || i == 4);
                    if (mostra) Desenho.Texto(g, string.Format("{0:0}", alc * i / 4), Fv.MonoPeq, Cores.Fraco, r.X + 3, i == 4 ? y + 1 : y - 6);
                }
            DateTime agora = DateTime.Now;
            GraphicsState st = g.Save();
            g.SetClip(a);                                   // nenhuma linha sai da área do gráfico
            for (int k = 0; k < 3; k++)
            {
                List<PointF> pts = new List<PointF>();
                using (Pen pn = new Pen(k == 0 ? cFL : (k == 1 ? cFR : cFC), 1.8f))
                {
                    foreach (Amostra am in t.Hist)
                    {
                        double idade = (agora - am.T).TotalSeconds;
                        if (idade > janela) continue;
                        int v = k == 0 ? am.FL : (k == 1 ? am.FR : am.FC);
                        if (v <= 0) { if (pts.Count > 1) g.DrawLines(pn, pts.ToArray()); pts.Clear(); continue; }
                        float x = a.Right - (float)(idade / janela * a.Width);
                        float y = a.Bottom - (float)(Math.Min(v, alc) / alc * (a.Height - 2)) - 1;
                        pts.Add(new PointF(x, y));
                    }
                    if (pts.Count > 1) g.DrawLines(pn, pts.ToArray());
                }
            }
            g.Restore(st);
        }

        void Cone(Graphics g, string nome, int cm, double ang, double xc, double yc, double alc)
        {
            PointF o = Polar(xc, yc, ang, 1.0);
            float half = (float)cfg.Cone;
            float ini = (float)(-ang - half), varr = 2 * half;
            float R = (float)(alc * s);
            if (!t.Recente)
            {
                using (Pen p = new Pen(Color.FromArgb(40, 70, 60), 1f) { DashStyle = DashStyle.Dash })
                    g.DrawPie(p, o.X - R, o.Y - R, 2 * R, 2 * R, ini, varr);
                return;
            }
            if (cm <= 0)
            {
                using (Brush b = new SolidBrush(Desenho.Alfa(Cores.Verde, 22)))
                    g.FillPie(b, o.X - R, o.Y - R, 2 * R, 2 * R, ini, varr);
                PointF l = Polar(xc, yc, ang, alc * 0.55);
                Desenho.TextoCentro(g, nome + " sem eco", Fv.Mono, Cores.Fraco, l.X, l.Y);
                return;
            }
            float rd = (float)(Math.Min(cm, alc) * s);
            if (cm <= alc)
            {
                using (GraphicsPath anel = new GraphicsPath(FillMode.Alternate))
                using (Brush b = new SolidBrush(Desenho.Alfa(Cores.Vermelho, 50)))
                {
                    anel.AddPie(o.X - R, o.Y - R, 2 * R, 2 * R, ini, varr);
                    if (rd > 1) anel.AddPie(o.X - rd, o.Y - rd, 2 * rd, 2 * rd, ini, varr);
                    g.FillPath(b, anel);
                }
            }
            if (rd > 1)
            {
                using (Brush b = new SolidBrush(Desenho.Alfa(Cores.Verde, cm <= alc ? 80 : 35)))
                    g.FillPie(b, o.X - rd, o.Y - rd, 2 * rd, 2 * rd, ini, varr);
                if (cm <= alc)
                    using (Pen p = new Pen(Cores.Verde, 4f))
                        g.DrawArc(p, o.X - rd, o.Y - rd, 2 * rd, 2 * rd, ini, varr);
                float f = (Environment.TickCount % 900) / 900f, rp = f * rd;
                if (rp > 2)
                    using (Pen p = new Pen(Desenho.Alfa(Cores.Verde, (int)(160 * (1 - f))), 2f))
                        g.DrawArc(p, o.X - rp, o.Y - rp, 2 * rp, 2 * rp, ini, varr);
            }
            if (cm <= alc)
            {
                PointF lt = Polar(xc, yc, ang, Math.Max(cm, 25) + 14 / s);   // muito perto: rótulo afastado, para os três não se sobreporem
                Desenho.TextoCentro(g, string.Format("{0} cm", cm), Fv.MonoGrande, Cores.Verde, lt.X, lt.Y);
            }
            else
            {
                PointF lt = Polar(xc, yc, ang, alc * 0.78);
                Desenho.TextoCentro(g, string.Format("{0} cm", cm), Fv.Mono, Cores.Texto, lt.X, lt.Y - 8);
                Desenho.TextoCentro(g, "livre", Fv.MonoPeq, Cores.Fraco, lt.X, lt.Y + 8);
            }
        }

        void Leitura(Graphics g, string nome, int cm, float x, float y, bool direita)
        {
            string v = !t.Recente ? "sem dados" : (cm <= 0 ? "sem eco" : cm + " cm");
            Font f = Fv.MonoGrande;
            SizeF a = g.MeasureString(nome, Fv.Mono), b = g.MeasureString(v, f);
            float xa = direita ? x - a.Width : x, xb = direita ? x - b.Width : x;
            Desenho.Texto(g, nome, Fv.Mono, Cores.Fraco, xa, y);
            Desenho.Texto(g, v, f, t.Recente && cm > 0 ? Cores.Verde : Cores.Fraco, xb, y + 16);
        }
    }

    // ---------------- Robô (MPU) ----------------
    class RoboView : Vista
    {
        public RoboView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Painel);
            float W = Width / Ef, H = Height / Ef;
            float lado = Math.Min(W * 0.56f, H) - 6;
            if (lado < 40) return;
            float cx = lado / 2 + 2, cy = H / 2, R = lado / 2 - 26;

            using (Pen p = new Pen(Cores.Linha, 2f)) g.DrawEllipse(p, cx - R, cy - R, 2 * R, 2 * R);
            using (Pen p = new Pen(Cores.VerdeEsc, 1.5f))
            {
                for (int a = 0; a < 360; a += 10)
                {
                    double r = a * Math.PI / 180;
                    float r1 = a % 30 == 0 ? R - 9 : R - 4;
                    g.DrawLine(p, cx + (float)Math.Sin(r) * r1, cy - (float)Math.Cos(r) * r1,
                                  cx + (float)Math.Sin(r) * R, cy - (float)Math.Cos(r) * R);
                    if (a % 90 == 0)
                        Desenho.TextoCentro(g, a + "°", Fv.MonoPeq, Cores.Fraco,
                            cx + (float)Math.Sin(r) * (R + 15), cy - (float)Math.Cos(r) * (R + 13));
                }
            }
            // marcador da direção
            double hr = mv.H * Math.PI / 180;
            using (Brush b = new SolidBrush(Cores.Verde))
            {
                PointF ponta = new PointF(cx + (float)Math.Sin(hr) * (R - 2), cy - (float)Math.Cos(hr) * (R - 2));
                g.FillEllipse(b, ponta.X - 5, ponta.Y - 5, 10, 10);
            }

            GraphicsState st = g.Save();
            g.TranslateTransform(cx, cy);
            g.RotateTransform((float)mv.H);
            Desenho.Robo(g, R / 21.5f, t, cfg, mv);
            g.Restore(st);

            // lado direito: valores e inclinação
            float x0 = lado + 10, w = W - x0 - 8;
            if (w < 60) return;
            Desenho.Texto(g, "Direção", Fv.Mono, Cores.Fraco, x0, 6);
            Desenho.Texto(g, string.Format("{0:0}°", mv.H), Fv.Enorme, Cores.Verde, x0 - 4, 20);
            Desenho.Texto(g, mv.HeadingDoMpu ? "do giroscópio (MPU)" : "estimada pelos motores", Fv.MonoPeq, Cores.Fraco, x0, 62);

            float y = 88;
            Barra(g, "Inclinação frente/trás", t.Pit, x0, y, w); y += 44;
            Barra(g, "Inclinação lateral", t.Rol, x0, y, w); y += 44;
            Desenho.Texto(g, string.Format("Rotação: {0:0.0} °/s", t.Gz), Fv.Mono, Cores.Texto, x0, y); y += 20;
            string mpu = !t.Recente ? "sem dados" : (t.MpuOk ? "MPU ligado (WHO_AM_I 0x" + t.Who + ")" : "MPU não ligado (SDA 20 / SCL 21)");
            Desenho.Texto(g, mpu, Fv.MonoPeq, t.MpuOk ? Cores.Verde : Cores.Amarelo, x0, y);
        }

        void Barra(Graphics g, string nome, double graus, float x, float y, float w)
        {
            bool ok = t.Recente && t.MpuOk;
            Desenho.Texto(g, string.Format("{0}: {1}", nome, ok ? graus.ToString("0.0") + "°" : "-"), Fv.MonoPeq, Cores.Fraco, x, y);
            RectangleF r = new RectangleF(x, y + 16, w, 12);
            using (Brush b = new SolidBrush(Color.FromArgb(22, 34, 29))) g.FillRectangle(b, r);
            float meio = r.X + r.Width / 2;
            using (Pen p = new Pen(Cores.Linha)) g.DrawLine(p, meio, r.Y - 2, meio, r.Bottom + 2);
            if (!ok) return;
            float v = (float)Math.Max(-45, Math.Min(45, graus)) / 45f * r.Width / 2;
            Color c = Math.Abs(graus) > 15 ? Cores.Amarelo : Cores.Verde;
            using (Brush b = new SolidBrush(c))
                g.FillRectangle(b, v >= 0 ? meio : meio + v, r.Y + 2, Math.Abs(v), r.Height - 4);
        }
    }

    // ---------------- Trajeto ----------------
    class TrajetoView : Vista
    {
        public TrajetoView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Radar);
            float W = Width / Ef, H = Height / Ef;
            if (W < 40 || H < 40) return;
            double minX = mv.X, maxX = mv.X, minY = mv.Y, maxY = mv.Y;
            foreach (PointF p in mv.Trajeto)
            {
                minX = Math.Min(minX, p.X); maxX = Math.Max(maxX, p.X);
                minY = Math.Min(minY, p.Y); maxY = Math.Max(maxY, p.Y);
            }
            double span = Math.Max(120, Math.Max(maxX - minX, maxY - minY) + 40);
            double cxm = (minX + maxX) / 2, cym = (minY + maxY) / 2;
            float sc = (float)(Math.Min(W, H - 20) / span);
            Func<double, double, PointF> P = (x, y) => new PointF((float)(W / 2 + (x - cxm) * sc), (float)((H + 10) / 2 - (y - cym) * sc));

            using (Pen p = new Pen(Color.FromArgb(18, 40, 30)))
            {
                double passo = span > 400 ? 50 : 20;
                for (double x = Math.Floor((cxm - span) / passo) * passo; x <= cxm + span; x += passo)
                { PointF a = P(x, cym - span), b = P(x, cym + span); g.DrawLine(p, a, b); }
                for (double y = Math.Floor((cym - span) / passo) * passo; y <= cym + span; y += passo)
                { PointF a = P(cxm - span, y), b = P(cxm + span, y); g.DrawLine(p, a, b); }
            }
            if (mv.Trajeto.Count > 1)
            {
                PointF[] pts = new PointF[mv.Trajeto.Count];
                for (int i = 0; i < pts.Length; i++) pts[i] = P(mv.Trajeto[i].X, mv.Trajeto[i].Y);
                using (Pen p = new Pen(Desenho.Alfa(Cores.Verde, 170), 2f)) g.DrawLines(p, pts);
            }
            PointF inicio = P(0, 0);
            using (Pen p = new Pen(Cores.Fraco)) g.DrawEllipse(p, inicio.X - 4, inicio.Y - 4, 8, 8);

            PointF c = P(mv.X, mv.Y);
            GraphicsState st = g.Save();
            g.TranslateTransform(c.X, c.Y);
            g.RotateTransform((float)mv.H);
            using (Brush b = new SolidBrush(Cores.Verde))
                g.FillPolygon(b, new PointF[] { new PointF(0, -11), new PointF(-7, 7), new PointF(7, 7) });
            g.Restore(st);

            using (Brush fundo = new SolidBrush(Cores.Painel)) { g.FillRectangle(fundo, 0, 0, W, 20); g.FillRectangle(fundo, 0, H - 20, W, 20); }
            Desenho.Texto(g, "Estimativa (sem encoders), só de referência", Fv.MonoPeq, Cores.Fraco, 6, 4);
            Desenho.Texto(g, string.Format("x {0:0} cm   y {1:0} cm   (quadrícula {2} cm)", mv.X, mv.Y, span > 400 ? 50 : 20), Fv.MonoPeq, Cores.Fraco, 6, H - 16);
        }
    }

    // ---------------- Chama ----------------
    class ChamaView : Vista
    {
        public ChamaView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Painel);
            float W = Width / Ef, H = Height / Ef;
            string estado; Color cor;
            if (!t.Recente) { estado = "Sem dados"; cor = Cores.Fraco; }
            else if (t.Farol && !t.Chama) { estado = "VELA: FAROL IR"; cor = Cores.Laranja; }
            else if (!t.ChamaOk) { estado = "KY-026 não ligado (A0)"; cor = Cores.Amarelo; }
            else if (t.Chama) { estado = t.ChamaSim ? "CHAMA SIMULADA" : "CHAMA DETETADA"; cor = Cores.Laranja; }
            else { estado = "Sem chama"; cor = Cores.Verde; }

            bool compacto = H < 290;
            RectangleF caixa = new RectangleF(4, 4, W - 8, compacto ? 40 : 58);
            using (GraphicsPath p = Desenho.Arredondado(caixa, 8))
            {
                using (Brush b = new SolidBrush(Desenho.Alfa(cor, t.Chama ? 70 : 25))) g.FillPath(b, p);
                using (Pen pn = new Pen(cor, 1.5f)) g.DrawPath(pn, p);
            }
            Desenho.TextoCentro(g, estado, compacto ? Fv.MonoGrande : Fv.Estado, cor, W / 2, compacto ? 24 : 33);

            // barra de intensidade
            float y = compacto ? 52 : 78;
            double max = Math.Min(1023, Math.Max(200, t.FT * 2.5));
            Desenho.Texto(g, string.Format("Intensidade (diferença para o ambiente): {0}", t.Recente && t.ChamaOk ? t.FI.ToString() : "-"), Fv.MonoPeq, Cores.Fraco, 6, y);
            RectangleF r = new RectangleF(6, y + 18, W - 12, compacto ? 14 : 22);
            using (Brush b = new SolidBrush(Color.FromArgb(22, 34, 29))) g.FillRectangle(b, r);
            if (t.Recente && t.ChamaOk)
            {
                float v = (float)(Math.Min(t.FI, max) / max * r.Width);
                using (Brush b = new LinearGradientBrush(r, Cores.VerdeEsc, Cores.Laranja, 0f))
                    g.FillRectangle(b, r.X, r.Y, v, r.Height);
            }
            float xl = r.X + (float)(Math.Min(t.FT, max) / max * r.Width);
            using (Pen p = new Pen(Cores.Vermelho, 2f)) g.DrawLine(p, xl, r.Y - 4, xl, r.Bottom + 4);
            Desenho.Texto(g, "limiar " + t.FT, Fv.MonoPeq, Cores.Vermelho, Math.Min(xl + 3, W - 70), r.Bottom + 3);

            y = r.Bottom + (compacto ? 20 : 24);
            string[] linhas = {
                string.Format("Analógico (A0):  {0}", Val(t.FA)),
                string.Format("Base ambiente:   {0}", Val(t.FB)),
                string.Format("Digital (D28):   {0}", t.Recente && t.ChamaOk ? (t.FD == 1 ? "1 (HIGH)" : "0 (LOW)") : "-"),
            };
            // com pouco espaço, os três valores ficam numa linha só, para o gráfico ter sempre lugar
            if (H - y >= 54 + 22 + 70) { foreach (string l in linhas) { Desenho.Texto(g, l, Fv.Mono, Cores.Texto, 6, y); y += 18; } }
            else
            {
                string curta = string.Format("A0 {0}  ·  base {1}  ·  D28 {2}", Val(t.FA), Val(t.FB), t.Recente && t.ChamaOk ? t.FD.ToString() : "-");
                if (H - y >= 16 + 22 + 50) { Desenho.Texto(g, curta, Fv.MonoPeq, Cores.Texto, 6, y); y += 16; }
                else y -= 10;   // muito apertado: sem a linha de valores, o gráfico sobe
            }

            // intensidade nos últimos 30 s
            RectangleF gr = new RectangleF(6, y + 20, W - 12, H - y - 26);
            if (gr.Height < 24) return;   // sem espaço: nem título nem gráfico, para não ficar um título solto
            Desenho.Texto(g, "Intensidade, últimos 30 s", Fv.MonoPeq, Cores.Fraco, 6, y + 3);
            using (Brush b = new SolidBrush(Color.FromArgb(8, 18, 12))) g.FillRectangle(b, gr);
            using (Pen pn = new Pen(Cores.Linha)) g.DrawRectangle(pn, gr.X, gr.Y, gr.Width, gr.Height);
            float yl = gr.Bottom - (float)(Math.Min(t.FT, max) / max * gr.Height);
            using (Pen pn = new Pen(Desenho.Alfa(Cores.Vermelho, 160), 1f) { DashStyle = DashStyle.Dash }) g.DrawLine(pn, gr.X, yl, gr.Right, yl);
            DateTime agora = DateTime.Now;
            GraphicsState stc = g.Save();
            g.SetClip(gr);
            List<PointF> pts = new List<PointF>();
            foreach (Amostra a in t.Hist)
            {
                double idade = (agora - a.T).TotalSeconds;
                if (idade > 30 || a.FI < 0) continue;
                pts.Add(new PointF(gr.Right - (float)(idade / 30 * gr.Width), gr.Bottom - (float)(Math.Min(a.FI, max) / max * gr.Height)));
            }
            if (pts.Count > 1)
            {
                using (Pen pn = new Pen(Cores.Laranja, 1.8f)) g.DrawLines(pn, pts.ToArray());
            }
            g.Restore(stc);
        }

        string Val(int v) { return t.Recente && t.ChamaOk ? v.ToString() : "-"; }
    }

    // ---------------- Bateria ----------------
    class BateriaView : Vista
    {
        public BateriaView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Painel);
            float W = Width / Ef;
            Linha(g, "Bateria dos motores (6x AA)", "A11", t.BatOk, t.Bat, true, 0, W);
            Linha(g, "Bateria do Arduino (6x AA)", "A12", t.BatAOk, t.BatA, true, 52, W);
        }

        // Fontes A e B: 6x AA alcalinas cada, 6,0 V vazia a 9,3 V cheia
        void Linha(Graphics g, string nome, string pino, bool ok, double v, bool percentagem, float y, float W)
        {
            if (!t.Recente) { Desenho.Texto(g, nome + ": sem dados", Fv.Mono, Cores.Fraco, 4, y + 6); return; }
            if (!ok)
            {
                Desenho.Texto(g, nome + ": divisor não ligado (" + pino + ")", Fv.Mono, Cores.Amarelo, 4, y + 4);
                Desenho.Texto(g, "(+) → 20k → " + pino + " → 10k → GND", Fv.MonoPeq, Cores.Fraco, 4, y + 22);
                return;
            }
            if (v < 1.0)
            {
                Desenho.Texto(g, nome, Fv.MonoPeq, Cores.Fraco, 4, y);
                Desenho.Texto(g, "desligada (interruptor?)", Fv.MonoGrande, Cores.Fraco, 2, y + 14);
                return;
            }
            if (v < 5.3)
            {
                Desenho.Texto(g, nome, Fv.MonoPeq, Cores.Fraco, 4, y);
                Desenho.Texto(g, "sem bateria: energia do USB", Fv.MonoGrande, Cores.Fraco, 2, y + 14);
                return;
            }
            double pct = Math.Max(0, Math.Min(1, (v - 6.0) / (9.3 - 6.0)));
            Color c = !percentagem ? Cores.Verde : (pct > 0.5 ? Cores.Verde : (pct > 0.2 ? Cores.Amarelo : Cores.Vermelho));
            Desenho.Texto(g, nome, Fv.MonoPeq, Cores.Fraco, 4, y);
            Desenho.Texto(g, string.Format("{0:0.00} V", v), Fv.MonoGrande, c, 2, y + 14);
            if (!percentagem) return;
            RectangleF r = new RectangleF(110, y + 16, Math.Max(40, W - 170), 20);
            using (Pen p = new Pen(c, 1.5f)) g.DrawRectangle(p, r.X, r.Y, r.Width, r.Height);
            using (Brush b = new SolidBrush(c)) g.FillRectangle(b, r.Right, r.Y + 5, 4, 10);
            using (Brush b = new SolidBrush(Desenho.Alfa(c, 180))) g.FillRectangle(b, r.X + 2, r.Y + 2, (float)((r.Width - 4) * pct), r.Height - 4);
            Desenho.Texto(g, string.Format("{0:0}%", pct * 100), Fv.Mono, Cores.Texto, r.Right + 8, r.Y + 2);
        }
    }

    // ---------------- Ventoinha ----------------
    class VentoinhaView : Vista
    {
        public VentoinhaView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Painel);
            float W = Width / Ef, H = Height / Ef;
            float lado = Math.Min(H - 10, 120);
            if (lado < 30) return;
            int pot = t.Recente ? t.Fan : 0;
            float sc = lado / 10f;
            GraphicsState st = g.Save();
            g.TranslateTransform(10 + lado / 2, H / 2);
            Desenho.Ventoinha(g, sc, 0, 0, 9f, pot, mv.FaseFan);
            g.Restore(st);
            float x = lado + 28;
            Desenho.Texto(g, pot > 0 ? "A SOPRAR" : "Parada", Fv.MonoGrande, pot > 0 ? Cores.Verde : Cores.Fraco, x, H / 2 - 26);
            Desenho.Texto(g, string.Format("PWM {0} / 255  ({1:0}%)", pot, pot / 2.55), Fv.Mono, Cores.Texto, x, H / 2);
            Desenho.Texto(g, "XY-MOS no D44, bateria do Arduino", Fv.MonoPeq, Cores.Fraco, x, H / 2 + 20);
        }
    }

    // ---------------- Motores (barras) ----------------
    class MotoresView : Vista
    {
        public MotoresView(Telemetria t, Config cfg, Movimento mv) : base(t, cfg, mv) { }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = Preparar(e, Cores.Painel);
            float W = Width / Ef, H = Height / Ef;
            BarraMotor(g, "ESQ", t.Recente ? t.ML : 0, 6, (W - 18) / 2, H);
            BarraMotor(g, "DIR", t.Recente ? t.MR : 0, 12 + (W - 18) / 2, (W - 18) / 2, H);
        }

        void BarraMotor(Graphics g, string nome, int v, float x, float w, float H)
        {
            RectangleF r = new RectangleF(x, 20, w, H - 44);
            using (Brush b = new SolidBrush(Color.FromArgb(22, 34, 29))) g.FillRectangle(b, r);
            float meio = r.Y + r.Height / 2;
            using (Pen p = new Pen(Cores.Linha)) g.DrawLine(p, r.X - 2, meio, r.Right + 2, meio);
            float h = Math.Abs(v) / 255f * r.Height / 2;
            using (Brush b = new SolidBrush(Desenho.CorRoda(v)))
                g.FillRectangle(b, r.X + 3, v >= 0 ? meio - h : meio, r.Width - 6, h);
            Desenho.TextoCentro(g, nome, Fv.Mono, Cores.Fraco, x + w / 2, 9);
            Desenho.TextoCentro(g, v.ToString(), Fv.MonoGrande, Desenho.CorRoda(v), x + w / 2, H - 12);
        }
    }

    // ---------------- Fases ----------------
    class FasesView : Control
    {
        Telemetria t;
        static readonly string[,] Fases = {
            { "IDLE", "Parado" }, { "CALIBRAR", "Calibração" }, { "MANUAL", "Controlo manual" },
            { "BUSCA", "Procurar chama" }, { "APROXIMAR", "Aproximar" }, { "EXTINGUIR", "Extinguir" }, { "RETORNAR", "Regressar" } };

        public FasesView(Telemetria t)
        {
            this.t = t;
            SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
            Dock = DockStyle.Top;
            BackColor = Cores.Fundo;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;
            g.Clear(Cores.Fundo);
            g.ScaleTransform(Escala.F, Escala.F);
            float Width = this.Width / Escala.F, Height = this.Height / Escala.F;
            int n = Fases.GetLength(0);
            float margem = 12, seta = 18;
            float w = (Width - 2 * margem - (n - 1) * seta) / n, h = Height - 14;
            for (int i = 0; i < n; i++)
            {
                float x = margem + i * (w + seta);
                bool ativa = t.Recente && t.Estado == Fases[i, 0];
                bool futura = i >= 3;
                RectangleF r = new RectangleF(x, 7, w, h);
                Color borda = ativa ? Cores.Verde : (futura ? Color.FromArgb(40, 55, 50) : Cores.Linha);
                using (GraphicsPath p = Desenho.Arredondado(r, 8))
                {
                    using (Brush b = new SolidBrush(ativa ? Color.FromArgb(20, 70, 40) : Cores.Painel)) g.FillPath(b, p);
                    using (Pen pn = new Pen(borda, ativa ? 2f : 1f)) g.DrawPath(pn, p);
                }
                Color ct = ativa ? Cores.Verde : (futura ? Color.FromArgb(80, 100, 92) : Cores.Texto);
                Desenho.TextoCentro(g, Fases[i, 0], Fv.Titulo, ct, x + w / 2, 7 + h * 0.36f);
                Desenho.TextoCentro(g, futura ? Fases[i, 1] + " (missão)" : Fases[i, 1], Fv.MonoPeq, futura ? Color.FromArgb(70, 90, 82) : Cores.Fraco, x + w / 2, 7 + h * 0.72f);
                if (i < n - 1)
                    Desenho.TextoCentro(g, i == 2 ? "|" : "›", Fv.MonoGrande, Cores.Linha, x + w + seta / 2, 7 + h / 2);
            }
        }
    }

    // Cartão com título (painel escuro)
    class Cartao : Panel
    {
        public string Titulo;
        public bool Ampliado;
        public Cartao(string titulo)
        {
            Titulo = titulo;
            Padding = new Padding((int)(10 * Escala.F), (int)(30 * Escala.F), (int)(10 * Escala.F), (int)(10 * Escala.F));
            BackColor = Cores.Painel;
            ForeColor = Cores.Texto;
            Margin = new Padding(5);
            Dock = DockStyle.Fill;
            DoubleBuffered = true;
            AutoScroll = true;
        }
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            e.Graphics.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;
            string tit = Titulo.ToUpperInvariant() + (Ampliado ? "   (Esc ou duplo clique para voltar)" : "");
            Desenho.Texto(e.Graphics, tit, Fontes.Titulo, Ampliado ? Cores.Verde : Cores.Fraco, 9 * Escala.F, 8 * Escala.F);
            using (Pen p = new Pen(Ampliado ? Cores.VerdeEsc : Cores.Linha)) e.Graphics.DrawRectangle(p, 0, 0, Width - 1, Height - 1);
            // ícone de ampliar/reduzir (cantos) no canto superior direito
            float k = 5 * Escala.F, x = Width - 22 * Escala.F, y = 9 * Escala.F, l = 11 * Escala.F;
            using (Pen p = new Pen(Cores.Fraco, 1.4f))
            {
                if (!Ampliado)
                {
                    e.Graphics.DrawLines(p, new PointF[] { new PointF(x, y + k), new PointF(x, y), new PointF(x + k, y) });
                    e.Graphics.DrawLines(p, new PointF[] { new PointF(x + l - k, y + l), new PointF(x + l, y + l), new PointF(x + l, y + l - k) });
                }
                else
                {
                    e.Graphics.DrawLines(p, new PointF[] { new PointF(x + k, y), new PointF(x + k, y + k), new PointF(x, y + k) });
                    e.Graphics.DrawLines(p, new PointF[] { new PointF(x + l, y + l - k), new PointF(x + l - k, y + l - k), new PointF(x + l - k, y + l) });
                }
            }
        }
    }

    class PortaItem
    {
        public string Nome, Descricao;
        public override string ToString() { return string.IsNullOrEmpty(Descricao) ? Nome : Descricao; }
    }

    // ---------------- Janela principal ----------------
    class Principal : Form
    {
        readonly Telemetria t = new Telemetria();
        readonly Config cfg = new Config();
        readonly Movimento mv = new Movimento();

        SerialPort porta;
        LigacaoBle ble;
        bool aLigarBle = false;
        DateTime ultimaTentativaBle = DateTime.MinValue;
        ToolTip dicas;
        ToolTip Dicas { get { if (dicas == null) dicas = new ToolTip { AutoPopDelay = 12000, InitialDelay = 400 }; return dicas; } }
        bool LigadoUsb { get { return porta != null && porta.IsOpen; } }
        bool LigadoBle { get { return ble != null && ble.Ligado; } }
        bool Ligado { get { return LigadoUsb || LigadoBle; } }
        readonly StringBuilder rx = new StringBuilder();
        TableLayoutPanel grelha;
        Cartao ampliado = null;
        readonly Dictionary<Control, int[]> posicoes = new Dictionary<Control, int[]>();
        bool desligadoPeloUtilizador = false;
        DateTime ultimaTentativa = DateTime.MinValue;

        ComboBox cbPortas; Button btLigar; Label lbEstado; CheckBox ckDemo; Label lbBat;
        RadarView radar; VentoinhaView ventoinhaView; RoboView robo; TrajetoView trajeto; ChamaView chama; BateriaView bateria; MotoresView motoresView; FasesView fases;
        TextBox consola, entrada; CheckBox ckTelem;
        TrackBar tbVel, tbFan; CheckBox ckFan; Label lbFan, lbVel;
        NumericUpDown nuLimiar, nuFator, nuFatorA, nuAngulo, nuAfast, nuAlcance, nuVMax; CheckBox ckInverter;

        readonly bool[] tecla = new bool[4]; // cima, baixo, esquerda, direita
        readonly bool[] botao = new bool[4];
        bool conduzia = false;
        DateTime ultimoTick = DateTime.Now;
        float esc = 1f;

        // demo
        DateTime demoSimInicio = DateTime.MinValue;
        bool demo = false; double demoT = 0; int demoML, demoMR, demoFan; DateTime demoCmd = DateTime.MinValue; double demoH = 0;

        public Principal()
        {
            cfg.Carregar();
            mv.Limpar();
            using (Graphics g = CreateGraphics()) esc = g.DpiX / 96f;
            Escala.F = esc;

            Text = "FIREBOT";
            BackColor = Cores.Fundo;
            ForeColor = Cores.Texto;
            Font = Fontes.Normal;
            StartPosition = FormStartPosition.CenterScreen;
            Size = new Size(Px(1500), Px(960));
            MinimumSize = new Size(Px(1100), Px(720));
            KeyPreview = true;
            try { Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath); } catch { Icon = CriarIcone(); }

            grelha = CriarGrelha();
            PrepararAmpliar();
            fases = new FasesView(t) { Height = Px(58) };
            Control topo = CriarTopo();
            Controls.Add(grelha);
            Controls.Add(fases);
            Controls.Add(topo);

            Timer ui = new Timer { Interval = 40 };
            ui.Tick += TickUi;
            ui.Start();
            Timer conducao = new Timer { Interval = 150 };
            conducao.Tick += TickConducao;
            conducao.Start();
            Timer demoTimer = new Timer { Interval = 100 };
            demoTimer.Tick += TickDemo;
            demoTimer.Start();

            KeyUp += (s, e) => Tecla(e.KeyCode, false);
            Deactivate += (s, e) => { for (int i = 0; i < 4; i++) { tecla[i] = false; botao[i] = false; } };
            FormClosing += (s, e) => { Enviar("STOP"); Desligar(null); cfg.Guardar(); };
            if (argLarg > 0 && argAlt > 0) { StartPosition = FormStartPosition.Manual; Location = new Point(0, 0); MinimumSize = new Size(1, 1); Size = new Size(Px(argLarg), Px(argAlt)); }
            Shown += (s, e) =>
            {
                AtualizarPortas();
                if (argBle)
                {
                    for (int i = 0; i < cbPortas.Items.Count; i++) if (((PortaItem)cbPortas.Items[i]).Nome == "BLE") cbPortas.SelectedIndex = i;
                    Ligar();
                }
                else if (argDemo) ckDemo.Checked = true; else LigarAutomatico();
                if (Array.IndexOf(Environment.GetCommandLineArgs(), "--simular") >= 0) Enviar("SIMCHAMA");
                if (argApresentacao) { AlternarApresentacao(); if (argSlide > 0) apresentacao.Mudar(argSlide); }
                if (argCaptura != null)
                {
                    Timer tc = new Timer { Interval = 4000 };
                    tc.Tick += (s2, e2) => { tc.Stop(); try { if (apresentacao != null) apresentacao.GuardarImagem(argCaptura); else using (Bitmap b = new Bitmap(Width, Height)) { DrawToBitmap(b, new Rectangle(Point.Empty, Size)); b.Save(argCaptura); } } catch { } Close(); };
                    tc.Start();
                }
                if (argAmpliar != null)
                    foreach (Control c in grelha.Controls)
                    {
                        Cartao k = c as Cartao;
                        if (k != null && k.Titulo.ToLowerInvariant().StartsWith(argAmpliar)) { Ampliar(k); break; }
                    }
            };
            Log("FIREBOT App pronta. Liga o Mega por USB: a app liga-se sozinha à porta do Arduino.");
        }

        int Px(int v) { return (int)Math.Round(v * esc); }

        // ---------- ampliar um painel com duplo clique ----------
        void PrepararAmpliar()
        {
            foreach (Control c in grelha.Controls)
            {
                TableLayoutPanelCellPosition pos = grelha.GetPositionFromControl(c);
                posicoes[c] = new int[] { pos.Column, pos.Row, grelha.GetColumnSpan(c), grelha.GetRowSpan(c) };
                Cartao cartao = c as Cartao;
                if (cartao != null) LigarDuploClique(cartao, cartao);
            }
        }

        void LigarDuploClique(Control c, Cartao cartao)
        {
            if (c is ButtonBase || c is TextBoxBase || c is TrackBar || c is UpDownBase || c is ComboBox) return;
            c.DoubleClick += (s, e) => Ampliar(cartao);
            foreach (Control f in c.Controls) LigarDuploClique(f, cartao);
        }

        void Ampliar(Cartao c)
        {
            grelha.SuspendLayout();
            if (ampliado == null && c != null)
            {
                foreach (Control x in grelha.Controls) x.Visible = x == c;
                grelha.SetCellPosition(c, new TableLayoutPanelCellPosition(0, 0));
                grelha.SetColumnSpan(c, 3);
                grelha.SetRowSpan(c, 3);
                c.Ampliado = true;
                ampliado = c;
            }
            else
            {
                foreach (KeyValuePair<Control, int[]> kv in posicoes)
                {
                    grelha.SetCellPosition(kv.Key, new TableLayoutPanelCellPosition(kv.Value[0], kv.Value[1]));
                    grelha.SetColumnSpan(kv.Key, kv.Value[2]);
                    grelha.SetRowSpan(kv.Key, kv.Value[3]);
                    kv.Key.Visible = true;
                    Cartao k = kv.Key as Cartao;
                    if (k != null) k.Ampliado = false;
                }
                ampliado = null;
            }
            grelha.ResumeLayout(true);
            grelha.Invalidate(true);
        }

        // ---------- construção da interface ----------
        Button Botao(string texto, EventHandler h)
        {
            Button b = new Button { Text = texto, FlatStyle = FlatStyle.Flat, BackColor = Cores.Botao, ForeColor = Cores.Verde, AutoSize = true, Margin = new Padding(3), Padding = new Padding(6, 2, 6, 2), Cursor = Cursors.Hand };
            b.FlatAppearance.BorderColor = Cores.VerdeEsc;
            b.FlatAppearance.MouseOverBackColor = Color.FromArgb(26, 56, 40);
            if (h != null) b.Click += h;
            return b;
        }

        NumericUpDown Numero(decimal min, decimal max, decimal val, int casas, decimal passo)
        {
            return new NumericUpDown { Minimum = min, Maximum = max, Value = Math.Max(min, Math.Min(max, val)), DecimalPlaces = casas, Increment = passo, Width = Px(70), BackColor = Color.FromArgb(10, 18, 15), ForeColor = Cores.Texto, BorderStyle = BorderStyle.FixedSingle, Margin = new Padding(3, 5, 3, 3) };
        }

        Label Rotulo(string texto)
        {
            return new Label { Text = texto, AutoSize = true, ForeColor = Cores.Fraco, Margin = new Padding(3, 8, 3, 3) };
        }

        Control CriarTopo()
        {
            FlowLayoutPanel p = new FlowLayoutPanel { Dock = DockStyle.Top, Height = Px(50), BackColor = Color.FromArgb(8, 16, 13), Padding = new Padding(Px(10), Px(8), Px(10), 0), WrapContents = false };
            Label titulo = new Label { Text = "FIREBOT", AutoSize = true, Font = new Font("Consolas", 16f, FontStyle.Bold), ForeColor = Cores.Verde, Margin = new Padding(0, 2, Px(18), 0) };
            cbPortas = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = Px(230), BackColor = Color.FromArgb(10, 18, 15), ForeColor = Cores.Texto, FlatStyle = FlatStyle.Flat, Margin = new Padding(3, 6, 3, 3) };
            btLigar = Botao("Ligar", (s, e) => { if (aLigarBle) return; if (Ligado) { desligadoPeloUtilizador = true; Desligar("Desligado."); } else { desligadoPeloUtilizador = false; Ligar(); } });
            lbEstado = new Label { Text = "●  Desligado", AutoSize = false, Width = Px(360), Height = Px(24), AutoEllipsis = true, ForeColor = Cores.Fraco, Margin = new Padding(Px(10), Px(10), Px(10), 0) };
            ckDemo = new CheckBox { Text = "Demo", AutoSize = true, ForeColor = Cores.Fraco, Margin = new Padding(Px(10), Px(9), 3, 0) };
            ckDemo.CheckedChanged += (s, e) => { demo = ckDemo.Checked; if (demo && Ligado) { desligadoPeloUtilizador = true; Desligar("Desligado para o modo demo."); } Log(demo ? "Modo demo ligado (valores simulados)." : "Modo demo desligado."); };
            lbBat = new Label { Text = "", AutoSize = false, Width = Px(230), Height = Px(24), ForeColor = Cores.Fraco, Margin = new Padding(Px(20), Px(10), Px(10), 0) };
            Button parar = Botao("■  PARAR  (espaço)", (s, e) => PararTudo());
            parar.ForeColor = Cores.Vermelho;
            parar.BackColor = Color.FromArgb(40, 14, 14);
            parar.FlatAppearance.BorderColor = Color.FromArgb(140, 45, 45);
            parar.Margin = new Padding(Px(6), 3, 3, 3);

            Button atualizar = Botao("↻", (s, e) => AtualizarPortas());
            Button ajuda = Botao("Ajuda (F1)", (s, e) => MostrarAjuda());
            Button apresentar = Botao("Apresentação (F11)", (s, e) => AlternarApresentacao());
            Dicas.SetToolTip(apresentar, "Ecrã inteiro para o público: radar, robô, chama e o QR do guia. Com dois ecrãs abre no segundo. F11 ou Esc fecha.");
            Dicas.SetToolTip(cbPortas, "Por onde falar com o robô: o Arduino por cabo USB, ou o Bluetooth BLE (BT24) sem cabo.");
            Dicas.SetToolTip(atualizar, "Voltar a procurar portas (depois de ligar um cabo, por exemplo).");
            Dicas.SetToolTip(btLigar, "Ligar ou desligar do robô. Por cabo é imediato; por Bluetooth a procura demora até 12 s.");
            Dicas.SetToolTip(ckDemo, "Modo demonstração: a app mostra valores simulados, sem precisar do robô.");
            Dicas.SetToolTip(parar, "Pára motores e ventoinha de imediato. Atalho: barra de espaço.");
            Dicas.SetToolTip(ajuda, "Atalhos de teclado e resumo de como usar a app.");
            cbPortas.Name = "portas"; btLigar.Name = "ligar"; ckDemo.Name = "demo"; parar.Name = "parar";
            p.Controls.AddRange(new Control[] { titulo, cbPortas, atualizar, btLigar, ckDemo, lbEstado, lbBat });
            // Ajuda, Apresentação e PARAR presos à direita: ficam sempre visíveis, mesmo em ecrãs pequenos
            FlowLayoutPanel dir = new FlowLayoutPanel { Dock = DockStyle.Right, AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink, WrapContents = false, BackColor = p.BackColor, Padding = new Padding(0, Px(8), Px(10), 0) };
            dir.Controls.AddRange(new Control[] { ajuda, apresentar, parar });
            Panel topo = new Panel { Dock = DockStyle.Top, Height = Px(50), BackColor = p.BackColor };
            p.Dock = DockStyle.Fill;
            topo.Controls.Add(p); topo.Controls.Add(dir);
            return topo;
        }

        TableLayoutPanel CriarGrelha()
        {
            TableLayoutPanel gr = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 3, Padding = new Padding(Px(6)), BackColor = Cores.Fundo };
            gr.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 42));
            gr.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 30));
            gr.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 28));
            gr.RowStyles.Add(new RowStyle(SizeType.Percent, 38));
            gr.RowStyles.Add(new RowStyle(SizeType.Percent, 32));
            gr.RowStyles.Add(new RowStyle(SizeType.Percent, 30));

            // radar
            Cartao cRadar = new Cartao("Radar: sonares frontais");
            radar = new RadarView(t, cfg, mv);
            cRadar.Controls.Add(radar);
            gr.Controls.Add(cRadar, 0, 0);
            gr.SetRowSpan(cRadar, 2);

            // robô
            Cartao cRobo = new Cartao("Robô: direção e inclinação (MPU)");
            robo = new RoboView(t, cfg, mv);
            cRobo.Controls.Add(robo);
            gr.Controls.Add(cRobo, 1, 0);

            // chama
            Cartao cChama = new Cartao("Sensor de chama (KY-026)");
            chama = new ChamaView(t, cfg, mv);
            cChama.Controls.Add(chama);
            gr.Controls.Add(cChama, 2, 0);

            // trajeto
            Cartao cTraj = new Cartao("Movimento e trajeto");
            trajeto = new TrajetoView(t, cfg, mv);
            cTraj.Controls.Add(trajeto);
            gr.Controls.Add(cTraj, 1, 1);

            gr.Controls.Add(CriarMotores(), 2, 1);
            gr.Controls.Add(CriarEnergia(), 0, 2);
            Control calib = CriarCalibracoes();
            gr.Controls.Add(calib, 1, 2);
            DicasCalibracao(calib);
            gr.Controls.Add(CriarConsola(), 2, 2);
            return gr;
        }

        Control CriarMotores()
        {
            Cartao c = new Cartao("Motores (setas / WASD)");
            motoresView = new MotoresView(t, cfg, mv) { Dock = DockStyle.Left, Width = Px(120) };

            TableLayoutPanel pad = new TableLayoutPanel { ColumnCount = 3, RowCount = 3, Dock = DockStyle.Top, Height = Px(132) };
            for (int i = 0; i < 3; i++) { pad.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3f)); pad.RowStyles.Add(new RowStyle(SizeType.Percent, 33.3f)); }
            pad.Controls.Add(BotaoConducao("▲", 0), 1, 0);
            pad.Controls.Add(BotaoConducao("◄", 2), 0, 1);
            Button stop = Botao("■", (s, e) => PararTudo());
            stop.Dock = DockStyle.Fill; stop.ForeColor = Cores.Vermelho; stop.AutoSize = false;
            pad.Controls.Add(stop, 1, 1);
            pad.Controls.Add(BotaoConducao("►", 3), 2, 1);
            pad.Controls.Add(BotaoConducao("▼", 1), 1, 2);

            lbVel = new Label { Text = "Velocidade: 150", AutoSize = true, ForeColor = Cores.Fraco, Dock = DockStyle.Top, Padding = new Padding(0, Px(6), 0, 0) };
            tbVel = new TrackBar { Minimum = 60, Maximum = 255, Value = 150, TickFrequency = 25, Dock = DockStyle.Top, BackColor = Cores.Painel };
            tbVel.ValueChanged += (s, e) => lbVel.Text = "Velocidade: " + tbVel.Value;
            Label aviso = new Label { Text = "Rodas no ar para testar.\nParam sozinhos sem ordens da app (0,5 s).", Dock = DockStyle.Top, Height = Px(44), ForeColor = Cores.Amarelo, Font = Fontes.MonoPeq };

            Panel dir = new Panel { Dock = DockStyle.Fill, Padding = new Padding(Px(10), 0, 0, 0) };
            dir.Controls.Add(aviso);
            dir.Controls.Add(tbVel);
            dir.Controls.Add(lbVel);
            dir.Controls.Add(pad);
            c.Controls.Add(dir);
            c.Controls.Add(motoresView);
            return c;
        }

        Button BotaoConducao(string texto, int idx)
        {
            Button b = Botao(texto, null);
            b.AutoSize = false; b.Dock = DockStyle.Fill; b.Font = new Font("Segoe UI", 13f);
            Dicas.SetToolTip(b, "Carrega e mantém para andar. Também podes usar as setas do teclado.");
            b.MouseDown += (s, e) => botao[idx] = true;
            b.MouseUp += (s, e) => botao[idx] = false;
            b.MouseLeave += (s, e) => botao[idx] = false;
            return b;
        }

        Control CriarEnergia()
        {
            Cartao c = new Cartao("Energia e ventoinha");
            bateria = new BateriaView(t, cfg, mv) { Dock = DockStyle.Top, Height = Px(100) };

            FlowLayoutPanel fan = new FlowLayoutPanel { Dock = DockStyle.Top, Height = Px(132), Padding = new Padding(0, Px(8), 0, 0) };
            ckFan = new CheckBox { Text = "Ventoinha ligada", AutoSize = true, ForeColor = Cores.Texto, Margin = new Padding(3, 8, 10, 3) };
            tbFan = new TrackBar { Minimum = 0, Maximum = 255, Value = 255, TickFrequency = 32, Width = Px(200), BackColor = Cores.Painel };
            lbFan = new Label { Text = "Potência 100%", AutoSize = true, ForeColor = Cores.Fraco, Margin = new Padding(3, 8, 3, 3) };
            ckFan.CheckedChanged += (s, e) => EnviarVentoinha();
            ckFan.Name = "ventoinha";
            tbFan.ValueChanged += (s, e) => { lbFan.Text = string.Format("Potência {0:0}%", tbFan.Value / 2.55); if (ckFan.Checked) EnviarVentoinha(); };
            Label pino = new Label { Text = "XY-MOS: TRIG/PWM no D44", AutoSize = true, ForeColor = Cores.Fraco, Font = Fontes.MonoPeq, Margin = new Padding(3, 4, 3, 3) };
            Button simular = Botao("Simular chama (tecla 0 do comando)", (s, e) => { if (!Ligado && !demo) { Log("Liga-te primeiro ao robô para simular a chama."); return; } Enviar("SIMCHAMA"); });
            simular.ForeColor = Cores.Laranja;
            Dicas.SetToolTip(simular, "Plano B sem vela: o robô cria uma chama sintética que dispara a sirene e a ventoinha. O ecrã e a app indicam que é simulada. Espaço pára tudo.");
            fan.Controls.AddRange(new Control[] { ckFan, tbFan, lbFan, pino, simular });
            fan.SetFlowBreak(lbFan, true);

            ventoinhaView = new VentoinhaView(t, cfg, mv);
            c.Controls.Add(ventoinhaView);
            c.Controls.Add(fan);
            c.Controls.Add(bateria);
            return c;
        }

        Control CriarCalibracoes()
        {
            Cartao c = new Cartao("Calibrações e posicionamento");
            TableLayoutPanel f = new TableLayoutPanel { Dock = DockStyle.Fill, AutoScroll = true, ColumnCount = 4 };
            for (int i = 0; i < 4; i++) f.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            FlowLayoutPanel botoes = new FlowLayoutPanel { AutoSize = true, Dock = DockStyle.Fill, Margin = new Padding(0, 0, 0, Px(6)) };
            botoes.Controls.Add(Botao("Calibrar giroscópio", (s, e) => { Enviar("CAL GYRO"); mv.H = 0; }));
            botoes.Controls.Add(Botao("Direção a 0°", (s, e) => { Enviar("HDG0"); mv.H = 0; }));
            botoes.Controls.Add(Botao("Calibrar IR ambiente", (s, e) => Enviar("CAL IR")));
            botoes.Controls.Add(Botao("Limpar trajeto", (s, e) => mv.Limpar()));
            f.Controls.Add(botoes, 0, 0);
            f.SetColumnSpan(botoes, 4);

            nuLimiar = Numero(1, 1023, 100, 0, 5);
            nuFator = Numero(0.5m, 20, 3, 2, 0.05m);
            nuAngulo = Numero(0, 80, (decimal)cfg.Angulo, 0, 1);
            nuAfast = Numero(0, 15, (decimal)cfg.Afastamento, 1, 0.5m);
            nuAlcance = Numero(20, 400, (decimal)cfg.Alcance, 0, 10);
            nuVMax = Numero(5, 200, (decimal)cfg.VMax, 0, 5);
            ckInverter = new CheckBox { Text = "Inverter giroscópio", Checked = cfg.InverterGiro, AutoSize = true, ForeColor = Cores.Texto, Margin = new Padding(3, 7, 3, 3) };

            FlowLayoutPanel limiar = new FlowLayoutPanel { AutoSize = true, Margin = new Padding(0) };
            limiar.Controls.Add(nuLimiar);
            limiar.Controls.Add(Botao("Enviar", (s, e) => Enviar("FTHR " + nuLimiar.Value)));
            FlowLayoutPanel fator = new FlowLayoutPanel { AutoSize = true, Margin = new Padding(0) };
            fator.Controls.Add(nuFator);
            fator.Controls.Add(Botao("Enviar", (s, e) => Enviar("BATR " + nuFator.Value.ToString(CultureInfo.InvariantCulture))));

            f.Controls.Add(Rotulo("Limiar chama"), 0, 1); f.Controls.Add(limiar, 1, 1);
            f.Controls.Add(Rotulo("Fator bat. motores"), 2, 1); f.Controls.Add(fator, 3, 1);
            f.Controls.Add(Rotulo("Ângulo sonares (°)"), 0, 2); f.Controls.Add(nuAngulo, 1, 2);
            f.Controls.Add(Rotulo("Afastamento (cm)"), 2, 2); f.Controls.Add(nuAfast, 3, 2);
            f.Controls.Add(Rotulo("Alcance radar (cm)"), 0, 3); f.Controls.Add(nuAlcance, 1, 3);
            f.Controls.Add(Rotulo("Vel. máxima (cm/s)"), 2, 3); f.Controls.Add(nuVMax, 3, 3);
            nuFatorA = Numero(0.5m, 20, 3, 2, 0.05m);
            FlowLayoutPanel fatorA = new FlowLayoutPanel { AutoSize = true, Margin = new Padding(0) };
            fatorA.Controls.Add(nuFatorA);
            fatorA.Controls.Add(Botao("Enviar", (s, e) => Enviar("BATRA " + nuFatorA.Value.ToString(CultureInfo.InvariantCulture))));
            f.Controls.Add(ckInverter, 0, 4); f.SetColumnSpan(ckInverter, 2);
            f.Controls.Add(Rotulo("Fator bat. Arduino"), 2, 4); f.Controls.Add(fatorA, 3, 4);

            EventHandler guardar = (s, e) =>
            {
                cfg.Angulo = (double)nuAngulo.Value; cfg.Afastamento = (double)nuAfast.Value;
                cfg.Alcance = (double)nuAlcance.Value; cfg.VMax = (double)nuVMax.Value;
                cfg.InverterGiro = ckInverter.Checked; cfg.Guardar();
            };
            nuAngulo.ValueChanged += guardar; nuAfast.ValueChanged += guardar; nuAlcance.ValueChanged += guardar;
            nuVMax.ValueChanged += guardar; ckInverter.CheckedChanged += guardar;
            c.Controls.Add(f);
            return c;
        }

        void DicasCalibracao(Control raiz)
        {
            foreach (Control x in raiz.Controls)
            {
                string d = null;
                switch (x.Text)
                {
                    case "Calibrar giroscópio": d = "Mede o desvio do giroscópio. Deixa o robô PARADO durante 1 s."; break;
                    case "Direção a 0°": d = "A direção atual passa a ser 0°."; break;
                    case "Calibrar IR ambiente": d = "Mede o infravermelho do ambiente. Faz com a vela APAGADA ou longe."; break;
                    case "Limpar trajeto": d = "Apaga o desenho do trajeto e volta a pôr o robô no centro."; break;
                }
                if (d != null) Dicas.SetToolTip(x, d);
                if (x.Controls.Count > 0) DicasCalibracao(x);
            }
        }

        void MostrarAjuda()
        {
            Form f = new Form { Text = "FIREBOT: ajuda rápida", BackColor = Cores.Painel, ForeColor = Cores.Texto, Size = new Size(Px(620), Px(560)), StartPosition = FormStartPosition.CenterParent, FormBorderStyle = FormBorderStyle.FixedDialog, MaximizeBox = false, MinimizeBox = false, Icon = Icon };
            TextBox t = new TextBox { Multiline = true, ReadOnly = true, Dock = DockStyle.Fill, BackColor = Cores.Painel, ForeColor = Cores.Texto, BorderStyle = BorderStyle.None, Font = new Font("Segoe UI", 10.5f), ScrollBars = ScrollBars.Vertical };
            t.Text = string.Join(Environment.NewLine, new string[] {
                "LIGAR AO ROBÔ",
                "  • Por cabo: liga o USB. A app liga-se sozinha ao Arduino.",
                "  • Sem cabo: escolhe \"Bluetooth BLE (BT24)\" e carrega em Ligar.",
                "    O robô tem de estar ligado pelas baterias. Chegam 2 leituras por segundo.",
                "",
                "CONDUZIR (rodas no ar para testar!)",
                "  • Setas ou W A S D: frente, trás, rodar à esquerda e à direita.",
                "  • Botões ▲ ▼ ◄ ►: carrega e mantém.",
                "  • Espaço ou PARAR TUDO: pára motores e ventoinha.",
                "  • Os motores param sozinhos se a app deixar de mandar ordens (0,5 s).",
                "",
                "APRESENTAÇÃO (F11)",
                "  • Ecrã inteiro para o público: radar, robô, chama e o QR do guia.",
                "  • Com um segundo ecrã (TV ou projetor) abre lá; o portátil fica com os controlos.",
                "  • ← → (ou Page Up/Down) mudam de slide; conduzir com W A S D; espaço pára tudo.",
                "  • Sem teclas durante 30 s, os slides passam sozinhos a cada 12 s. F11 ou Esc fecha.",
                "",
                "PAINÉIS",
                "  • Duplo clique num painel: ocupa a janela toda. Esc ou novo duplo clique: volta.",
                "  • Radar: verde até ao obstáculo, vermelho para lá dele. Zona cega a amarelo.",
                "  • Robô: direção pelo giroscópio e inclinação. A sombra mexe-se com a inclinação.",
                "  • Chama: nível acima do limiar (linha vermelha) = CHAMA DETETADA.",
                "",
                "CHAMA: SIRENE E VENTOINHA",
                "  • Ao detetar chama: sirene de bombeiros 3 s, depois a ventoinha (no máximo 8 s).",
                "  • Plano B sem vela: botão Simular chama ou tecla 0 do comando. Fica indicado como simulada.",
                "  • Espaço (ou POWER no comando) cala a sirene e pára tudo.",
                "",
                "COMANDO DE IR (aponta ao recetor)",
                "  • ▲ ▼ |<< >>| ou 2 8 4 6: conduzir.   POWER, FUNC/STOP, 5: parar.",
                "  • >||: liga/desliga a ventoinha.   VOL+ / VOL−: velocidade.",
                "  • 0: chama simulada.   EQ: liga/desliga o som do buzzer.",
                "",
                "LEDS DO ROBÔ",
                "  • Verde: parado e tudo bem.   Amarelo: a andar, a calibrar ou bateria a meio.",
                "  • Vermelho a piscar: chama.   Vermelho fixo: bateria baixa.",
                "  • A andar: aviso de distância. Verde > 50 cm, amarelo 25-50, vermelho < 25, a piscar < 12.",
                "    Com som (SOM 1), bips cada vez mais rápidos abaixo de 60 cm.",
                "",
                "CONSOLA",
                "  • Escreve comandos para o robô: PING, TESTE (LEDs e buzzer), STOP, FAN 255...",
            });
            f.Controls.Add(t);
            f.Shown += (s, e) => t.SelectionLength = 0;
            f.ShowDialog(this);
        }

        Control CriarConsola()
        {
            Cartao c = new Cartao("Consola");
            consola = new TextBox { Name = "consola", Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, Dock = DockStyle.Fill, BackColor = Color.FromArgb(4, 9, 7), ForeColor = Cores.Verde, Font = Fontes.MonoPeq, BorderStyle = BorderStyle.None, WordWrap = true };
            Panel baixo = new Panel { Dock = DockStyle.Bottom, Height = Px(40), Padding = new Padding(0, Px(4), 0, 0) };
            entrada = new TextBox { Dock = DockStyle.Fill, BackColor = Color.FromArgb(10, 18, 15), ForeColor = Cores.Texto, Font = Fontes.Mono, BorderStyle = BorderStyle.FixedSingle };
            entrada.Name = "entrada";
            entrada.KeyDown += (s, e) => { if (e.KeyCode == Keys.Enter) { EnviarEntrada(); e.SuppressKeyPress = true; } };
            Button env = Botao("Enviar", (s, e) => EnviarEntrada());
            env.Dock = DockStyle.Right; env.AutoSize = false; env.Width = Px(80); env.Margin = new Padding(0);
            ckTelem = new CheckBox { Text = "Mostrar telemetria", AutoSize = true, ForeColor = Cores.Fraco, Dock = DockStyle.Right, Padding = new Padding(Px(8), Px(4), 0, 0) };
            baixo.Controls.Add(entrada);
            baixo.Controls.Add(env);
            baixo.Controls.Add(ckTelem);
            c.Controls.Add(consola);
            c.Controls.Add(baixo);
            return c;
        }

        static Icon CriarIcone()
        {
            Bitmap b = new Bitmap(32, 32);
            using (Graphics g = Graphics.FromImage(b))
            {
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.Clear(Color.Transparent);
                using (Brush fundo = new SolidBrush(Color.FromArgb(10, 30, 20))) g.FillEllipse(fundo, 0, 0, 31, 31);
                using (GraphicsPath p = new GraphicsPath())
                {
                    p.AddBezier(16, 4, 26, 14, 24, 26, 16, 28);
                    p.AddBezier(16, 28, 8, 26, 6, 16, 16, 4);
                    using (Brush chamaB = new LinearGradientBrush(new Rectangle(0, 4, 32, 26), Color.FromArgb(255, 220, 60), Color.FromArgb(255, 70, 30), 90f))
                        g.FillPath(chamaB, p);
                }
            }
            return Icon.FromHandle(b.GetHicon());
        }

        // ---------- porta série ----------
        void AtualizarPortas()
        {
            string atual = cbPortas.SelectedItem != null ? ((PortaItem)cbPortas.SelectedItem).Nome : cfg.Porta;
            List<PortaItem> itens = new List<PortaItem>();
            Dictionary<string, string> nomes = new Dictionary<string, string>();
            try
            {
                using (ManagementObjectSearcher q = new ManagementObjectSearcher("SELECT Name FROM Win32_PnPEntity WHERE Name LIKE '%(COM%'"))
                    foreach (ManagementObject o in q.Get())
                    {
                        string n = (o["Name"] ?? "").ToString();
                        Match m = Regex.Match(n, @"\((COM\d+)\)");
                        if (m.Success) nomes[m.Groups[1].Value] = n;
                    }
            }
            catch { }
            foreach (string p in SerialPort.GetPortNames())
            {
                string d;
                nomes.TryGetValue(p, out d);
                itens.Add(new PortaItem { Nome = p, Descricao = d ?? p });
            }
            itens.Add(new PortaItem { Nome = "BLE", Descricao = "Bluetooth BLE (" + cfg.NomeBle + ")" });
            cbPortas.Items.Clear();
            int escolher = -1;
            for (int i = 0; i < itens.Count; i++) cbPortas.Items.Add(itens[i]);
            // o Arduino ligado por USB tem sempre prioridade; senão, a última porta usada
            for (int i = 0; i < itens.Count && escolher < 0; i++)
                if (itens[i].Descricao.IndexOf("Arduino", StringComparison.OrdinalIgnoreCase) >= 0) escolher = i;
            for (int i = 0; i < itens.Count && escolher < 0; i++)
                if (itens[i].Nome == atual) escolher = i;
            if (escolher < 0)
                for (int i = 0; i < itens.Count; i++)
                    if (itens[i].Descricao.IndexOf("Arduino", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        itens[i].Descricao.IndexOf("CH340", StringComparison.OrdinalIgnoreCase) >= 0 ||
                        itens[i].Descricao.IndexOf("CP210", StringComparison.OrdinalIgnoreCase) >= 0) { escolher = i; break; }
            if (escolher >= 0) cbPortas.SelectedIndex = escolher;
            else if (cbPortas.Items.Count > 0) cbPortas.SelectedIndex = 0;
        }

        bool PortaEhArduino()
        {
            PortaItem it = cbPortas.SelectedItem as PortaItem;
            return it != null && (it.Descricao.IndexOf("Arduino", StringComparison.OrdinalIgnoreCase) >= 0 || it.Nome == cfg.Porta);
        }

        void LigarAutomatico()
        {
            if (demo || desligadoPeloUtilizador || Ligado || aLigarBle) return;
            PortaItem it = cbPortas.SelectedItem as PortaItem;
            if (it == null) return;
            if (it.Nome == "BLE")
            {
                // Bluetooth: tenta sozinho se foi a última ligação, no máximo a cada 20 s
                if (cfg.Porta == "BLE" && (DateTime.Now - ultimaTentativaBle).TotalSeconds > 20) { ultimaTentativaBle = DateTime.Now; Ligar(); }
                return;
            }
            if (PortaEhArduino()) Ligar();
        }

        void Ligar()
        {
            PortaItem it = cbPortas.SelectedItem as PortaItem;
            if (it == null) { Log("Nenhuma porta COM encontrada. O Mega está ligado por USB?"); return; }
            if (demo) ckDemo.Checked = false;
            if (it.Nome == "BLE") { LigarBle(); return; }
            try
            {
                porta = new SerialPort(it.Nome, 115200) { DtrEnable = false, RtsEnable = false, Encoding = Encoding.ASCII, NewLine = "\n" };
                porta.DataReceived += (s, e) =>
                {
                    string txt;
                    try { txt = ((SerialPort)s).ReadExisting(); } catch { return; }
                    try { BeginInvoke(new Action<string>(Receber), txt); } catch { }
                };
                porta.Open();
                cfg.Porta = it.Nome; cfg.Guardar();
                btLigar.Text = "Desligar";
                lbEstado.Text = "Ligado a " + it.Nome;
                lbEstado.ForeColor = Cores.Verde;
                Log("Ligado a " + it);
            }
            catch (Exception ex)
            {
                porta = null;
                lbEstado.Text = "Erro ao ligar";
                lbEstado.ForeColor = Cores.Vermelho;
                Log("Não consegui abrir " + it.Nome + ": " + ex.Message + " (o Serial Monitor do Arduino IDE está aberto?)");
            }
        }

        async void LigarBle()
        {
            aLigarBle = true;
            btLigar.Enabled = false;
            lbEstado.Text = "A procurar " + cfg.NomeBle + " por Bluetooth...";
            lbEstado.ForeColor = Cores.Amarelo;
            Log("A procurar o " + cfg.NomeBle + " por Bluetooth (até 12 s)...");
            ble = new LigacaoBle();
            ble.Recebido += txt => { try { BeginInvoke(new Action<string>(Receber), txt); } catch { } };
            ble.Perdida += () => { try { BeginInvoke(new Action(() => Desligar("A ligação Bluetooth caiu (robô desligado ou longe demais?)."))); } catch { } };
            string erro;
            try { erro = await ble.Ligar(cfg.NomeBle); }
            catch (Exception ex) { erro = "Erro no Bluetooth: " + ex.Message; }
            aLigarBle = false;
            btLigar.Enabled = true;
            if (erro != null)
            {
                ble.Desligar(); ble = null;
                lbEstado.Text = "Bluetooth: não ligou"; lbEstado.ForeColor = Cores.Vermelho;
                Log(erro);
                return;
            }
            cfg.Porta = "BLE"; cfg.Guardar();
            btLigar.Text = "Desligar";
            Log("Ligado por Bluetooth ao " + cfg.NomeBle + ". Pelo Bluetooth chegam 2 leituras por segundo.");
        }

        void Desligar(string motivo)
        {
            if (ble != null) { ble.Desligar(); ble = null; }
            if (porta != null)
            {
                try { if (porta.IsOpen) porta.Close(); } catch { }
                porta.Dispose();
                porta = null;
            }
            if (btLigar != null) btLigar.Text = "Ligar";
            if (lbEstado != null) { lbEstado.Text = "Desligado"; lbEstado.ForeColor = Cores.Fraco; }
            if (motivo != null) Log(motivo);
        }

        void Receber(string txt)
        {
            rx.Append(txt);
            string tudo = rx.ToString();
            int i;
            while ((i = tudo.IndexOf('\n')) >= 0)
            {
                string l = tudo.Substring(0, i).Trim('\r', ' ');
                tudo = tudo.Substring(i + 1);
                ProcessarLinha(l);
            }
            rx.Clear();
            if (tudo.Length < 2000) rx.Append(tudo);
        }

        void ProcessarLinha(string l)
        {
            if (l.Length == 0) return;
            if (l.StartsWith("EV:")) { Log("robô: " + l.Substring(3)); return; }
            if (l == "PONG") { Log("robô: PONG"); return; }
            if (t.Aplicar(l)) { if (ckTelem.Checked) Log(l); }
            else if (l.IndexOf(':') < 0) Log("robô: " + l); // pedaços de telemetria cortados (ao ligar) não interessam
        }

        void Enviar(string cmd)
        {
            if (demo) { DemoComando(cmd); return; }
            if (!Ligado) { if (!cmd.StartsWith("M ")) Log("Não ligado: \"" + cmd + "\" não foi enviado."); return; }
            if (LigadoBle)
            {
                ble.Enviar(cmd + "\n");
                if (!cmd.StartsWith("M ")) Log("> " + cmd + "  (Bluetooth)");
                return;
            }
            try
            {
                porta.Write(cmd + "\n");
                if (!cmd.StartsWith("M ")) Log("> " + cmd);
            }
            catch (Exception ex) { Desligar("Ligação perdida: " + ex.Message); }
        }

        void EnviarEntrada()
        {
            string c = entrada.Text.Trim();
            if (c.Length == 0) return;
            Enviar(c.ToUpperInvariant());
            entrada.Clear();
        }

        void EnviarVentoinha()
        {
            if (ckFan.Checked && !Ligado && !demo) { ckFan.Checked = false; Log("Liga-te primeiro ao robô para usar a ventoinha."); return; }
            Enviar("FAN " + (ckFan.Checked ? tbFan.Value : 0));
        }

        void PararTudo()
        {
            for (int i = 0; i < 4; i++) { tecla[i] = false; botao[i] = false; }
            conduzia = false;
            ckFan.Checked = false;
            Enviar("STOP");
        }

        void Log(string s)
        {
            if (consola == null) return;
            if (consola.TextLength > 60000) consola.Text = consola.Text.Substring(consola.TextLength - 30000);
            consola.AppendText(DateTime.Now.ToString("HH:mm:ss ") + s + Environment.NewLine);
        }

        // ---------- teclado ----------
        bool AEscrever() { Control a = ActiveControl; while (a is ContainerControl && ((ContainerControl)a).ActiveControl != null) a = ((ContainerControl)a).ActiveControl; return a is TextBoxBase || a is NumericUpDown || a is UpDownBase; }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if (keyData == Keys.F1) { MostrarAjuda(); return true; }
            if (keyData == Keys.F11) { AlternarApresentacao(); return true; }
            if (keyData == Keys.Escape && ampliado != null) { Ampliar(null); return true; }
            if (keyData == Keys.Space && !(ActiveControl is TextBoxBase) && entrada != null && !entrada.Focused) { PararTudo(); return true; }
            if (!AEscrever())
            {
                if (keyData == Keys.Escape && ampliado != null) { Ampliar(null); return true; }
                if (Tecla(keyData, true)) return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        bool Tecla(Keys k, bool premida)
        {
            int i = -1;
            if (k == Keys.Up || k == Keys.W) i = 0;
            else if (k == Keys.Down || k == Keys.S) i = 1;
            else if (k == Keys.Left || k == Keys.A) i = 2;
            else if (k == Keys.Right || k == Keys.D) i = 3;
            if (i < 0) return false;
            if (!premida || !AEscrever()) tecla[i] = premida;
            return true;
        }

        void TickConducao(object s, EventArgs e)
        {
            bool c = tecla[0] || botao[0], b = tecla[1] || botao[1], esq = tecla[2] || botao[2], dir = tecla[3] || botao[3];
            int f = (c ? 1 : 0) - (b ? 1 : 0), v = (dir ? 1 : 0) - (esq ? 1 : 0);
            double vel = tbVel.Value;
            int l = 0, r = 0;
            if (f != 0)
            {
                l = (int)(f * vel * (v < 0 ? 0.35 : 1));
                r = (int)(f * vel * (v > 0 ? 0.35 : 1));
            }
            else if (v != 0)
            {
                l = (int)(v * vel * 0.8);
                r = (int)(-v * vel * 0.8);
            }
            if (l != 0 || r != 0) { Enviar(string.Format("M {0} {1}", l, r)); conduzia = true; }
            else if (conduzia) { Enviar("M 0 0"); conduzia = false; }
        }

        // ---------- ciclo da interface ----------
        int contaVerificacao = 0, pacotesAntes = 0;
        double ritmo = 0;
        DateTime inicioRitmo = DateTime.Now;
        void TickUi(object s, EventArgs e)
        {
            DateTime agora = DateTime.Now;
            double dt = Math.Min(0.2, (agora - ultimoTick).TotalSeconds);
            ultimoTick = agora;
            mv.Atualizar(t, cfg, dt);

            if (++contaVerificacao >= 30) // ~1,2 s
            {
                contaVerificacao = 0;
                if (porta != null && Array.IndexOf(SerialPort.GetPortNames(), porta.PortName) < 0)
                    Desligar("A porta " + porta.PortName + " desapareceu (cabo USB desligado ou curto-circuito?).");
                if (!Ligado && !aLigarBle && !demo && (agora - ultimaTentativa).TotalSeconds > 3)
                {
                    ultimaTentativa = agora;
                    AtualizarPortas();
                    if (!desligadoPeloUtilizador) LigarAutomatico();
                }
            }

            if (Ligado)
            {
                string onde = LigadoBle ? "Bluetooth (" + cfg.NomeBle + ")" : "USB (" + porta.PortName + ")";
                lbEstado.Text = t.Recente ? "●  Ligado por " + onde : "●  Ligado por " + onde + ", à espera de dados...";
                lbEstado.ForeColor = t.Recente ? Cores.Verde : Cores.Amarelo;
            }
            else if (demo) { lbEstado.Text = "●  Modo demo (valores simulados)"; lbEstado.ForeColor = Cores.Amarelo; }
            else if (aLigarBle) { lbEstado.Text = "●  A procurar " + cfg.NomeBle + " por Bluetooth..."; lbEstado.ForeColor = Cores.Amarelo; }
            else { lbEstado.Text = "●  Desligado: escolhe a ligação e carrega em Ligar"; lbEstado.ForeColor = Cores.Fraco; }

            if ((agora - inicioRitmo).TotalSeconds >= 1)
            {
                ritmo = (t.Pacotes - pacotesAntes) / (agora - inicioRitmo).TotalSeconds;
                pacotesAntes = t.Pacotes; inicioRitmo = agora;
            }
            if (t.Recente && (Ligado || demo)) lbEstado.Text += string.Format("  ·  {0:0} leituras/s", ritmo);
            if (t.Recente && t.ChamaSim) { lbEstado.Text += "  ·  CHAMA SIMULADA"; lbEstado.ForeColor = Cores.Laranja; }
            string bat = "";
            if (t.Recente && t.BatOk && t.Bat > 5.3) bat += string.Format("Motores {0:0.0} V", t.Bat);
            if (t.Recente && t.BatAOk && t.BatA > 5.3) bat += (bat.Length > 0 ? "   " : "") + string.Format("Arduino {0:0.0} V", t.BatA);
            lbBat.Text = bat;
            double menor = Math.Min(t.Bat > 5.3 ? t.Bat : 99, t.BatA > 5.3 ? t.BatA : 99);
            lbBat.ForeColor = menor > 7.5 ? Cores.Verde : (menor > 6.6 ? Cores.Amarelo : Cores.Vermelho);

            radar.Invalidate(); robo.Invalidate(); trajeto.Invalidate(); chama.Invalidate();
            bateria.Invalidate(); motoresView.Invalidate(); fases.Invalidate(); ventoinhaView.Invalidate();
        }

        // ---------- modo apresentação ----------
        Apresentacao apresentacao;
        void AlternarApresentacao()
        {
            if (apresentacao != null && !apresentacao.IsDisposed) { apresentacao.Close(); apresentacao = null; return; }
            apresentacao = new Apresentacao(t, cfg, mv,
                () => demo,
                () => Ligado && t.Recente ? (LigadoBle ? "Ligado ao robô por Bluetooth" : "Ligado ao robô por cabo USB") : (demo ? "" : "À espera do robô..."),
                (k, premida) => { if (k == Keys.Space && premida) { PararTudo(); return true; } return Tecla(k, premida); });
            Screen[] ecras = Screen.AllScreens;
            Screen alvo = Screen.FromControl(this);
            if (ecras.Length > 1) foreach (Screen e in ecras) if (!e.Equals(alvo)) { alvo = e; break; }
            apresentacao.FormClosed += (s, e) => { apresentacao = null; Activate(); };
            if (argLarg > 0 && argAlt > 0) apresentacao.AbrirTamanho(Px(argLarg), Px(argAlt)); else apresentacao.Abrir(alvo);
        }

        // ---------- modo demo ----------
        void DemoComando(string cmd)
        {
            int a, b;
            string[] p = cmd.Split(' ');
            if (p.Length == 3 && p[0] == "M" && int.TryParse(p[1], out a) && int.TryParse(p[2], out b)) { demoML = a; demoMR = b; demoCmd = DateTime.Now; return; }
            if (p.Length == 2 && p[0] == "FAN" && int.TryParse(p[1], out a)) { demoFan = a; Log("demo: ventoinha " + a); return; }
            if (cmd == "STOP") { demoML = demoMR = demoFan = 0; Log("demo: STOP"); return; }
            if (cmd == "SIMCHAMA") { demoSimInicio = DateTime.Now; Log("demo: chama simulada"); return; }
            if (cmd == "HDG0" || cmd == "CAL GYRO") demoH = 0;
            Log("demo: " + cmd);
        }

        void TickDemo(object s, EventArgs e)
        {
            if (!demo) return;
            demoT += 0.1;
            if ((DateTime.Now - demoCmd).TotalMilliseconds > 500)
            {
                // sem ordens do utilizador: o robô simulado passeia sozinho
                bool parado = (DateTime.Now - demoCmd).TotalSeconds < 3;
                demoML = parado ? 0 : (int)(130 + 70 * Math.Sin(demoT * 0.4));
                demoMR = parado ? 0 : (int)(130 - 70 * Math.Sin(demoT * 0.4 + 0.8));
            }
            demoH += (demoML - demoMR) / 255.0 * 40 / 15.0 * 0.1 * 180 / Math.PI;
            demoH = ((demoH % 360) + 360) % 360;
            int fl = (int)(60 + 35 * Math.Sin(demoT * 0.9));
            int fr = Math.Sin(demoT * 0.5) > 0.75 ? 0 : (int)(45 + 25 * Math.Sin(demoT * 1.3 + 1));
            int fi = (int)Math.Max(0, 60 + 90 * Math.Sin(demoT * 0.35));
            double ds = (DateTime.Now - demoSimInicio).TotalSeconds;
            bool demoSim = ds < 14;
            if (demoSim) fi = (int)Math.Max(0, (ds < 1.5 ? ds / 1.5 * 250 : (ds < 10 ? 250 : 250 - (ds - 10) * 70)) + 30 * Math.Sin(demoT * 7));
            string linha = string.Format(CultureInfo.InvariantCulture,
                "FL:{0} FR:{1} FC:{14} FCOK:1 FA:{2} FD:{3} FI:{4} FB:380 FT:100 BAT:{5:0.00} BR:3.00 BOK:1 BATA:7.62 BRA:3.00 BAOK:1 FOK:1 ST:{6} MPU:1 WHO:71 HDG:{7:0.0} PIT:{8:0.0} ROL:{9:0.0} GZ:{10:0.0} GB:0.10 ML:{11} MR:{12} FAN:{13} SIM:{15}",
                fl, fr, 380 - fi, fi > 100 ? 1 : 0, fi, 8.1 - 0.02 * Math.Sin(demoT), (demoML != 0 || demoMR != 0) ? "MANUAL" : "IDLE",
                demoH, 3 * Math.Sin(demoT * 0.7), 2 * Math.Cos(demoT * 0.5), (demoML - demoMR) / 255.0 * 150, demoML, demoMR, demoSim && ds > 3 && ds < 11 ? 255 : demoFan, (int)(40 + 25 * Math.Sin(demoT * 0.6 + 2)), demoSim ? 1 : 0);
            ProcessarLinha(linha);
        }

        [DllImport("user32.dll")] static extern bool SetProcessDPIAware();

        static bool argDemo = false;
        static string argAmpliar = null;
        static bool argBle = false;
        static bool argApresentacao = false;
        static string argCaptura = null;
        static int argSlide = 0;
        static int argLarg = 0, argAlt = 0;

        [STAThread]
        static void Main(string[] args)
        {
            argDemo = Array.IndexOf(args, "--demo") >= 0;
            argBle = Array.IndexOf(args, "--ble") >= 0;
            argApresentacao = Array.IndexOf(args, "--apresentacao") >= 0;
            foreach (string a in args) { if (a.StartsWith("--captura=")) argCaptura = a.Substring(10); if (a.StartsWith("--slide=")) int.TryParse(a.Substring(8), out argSlide); if (a.StartsWith("--tamanho=")) { string[] d = a.Substring(10).Split('x'); if (d.Length == 2) { int.TryParse(d[0], out argLarg); int.TryParse(d[1], out argAlt); } } }
            foreach (string a in args) if (a.StartsWith("--ampliar=")) argAmpliar = a.Substring(10).ToLowerInvariant();
            try { SetProcessDPIAware(); } catch { }
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new Principal());
        }
    }

    // Janela de ecrã inteiro para o público. As setas ← → (ou Page Up/Down de um comando de apresentação)
    // passam por: vista geral, radar, robô, chama e QR do guia. W A S D conduzem, espaço pára tudo.
    class Apresentacao : Form
    {
        readonly Func<bool> emDemo; readonly Func<string> estado; readonly Func<Keys, bool, bool> tecla;
        readonly Image qr, noite; readonly Label faixa; readonly PainelDB topo; readonly Telemetria tel;
        readonly List<Control> slides = new List<Control>();
        readonly List<Vista[]> vistasDoSlide = new List<Vista[]>();
        readonly string[] nomes = { "Vista geral", "Radar dos sonares", "Direção e inclinação", "Sensor de chama", "Guia e desafio" };
        int atual = 0;
        string ultimoEstado = null;
        readonly List<KeyValuePair<Vista, float>> zooms = new List<KeyValuePair<Vista, float>>();
        DateTime ultimaTecla = DateTime.Now, ultimaRotacao = DateTime.Now;
        const string Url = "firebot-noite.netlify.app";

        // painel desenhado em memória: sem piscar ao redesenhar
        class PainelDB : Panel
        {
            public PainelDB() { SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true); }
        }

        public Apresentacao(Telemetria t, Config cfg, Movimento mv, Func<bool> emDemo, Func<string> estado, Func<Keys, bool, bool> tecla)
        {
            this.emDemo = emDemo; this.estado = estado; this.tecla = tecla; tel = t;
            Text = "FIREBOT: apresentação";
            FormBorderStyle = FormBorderStyle.None;
            BackColor = Cores.Fundo; ForeColor = Cores.Texto;
            KeyPreview = true;
            SetStyle(ControlStyles.OptimizedDoubleBuffer | ControlStyles.AllPaintingInWmPaint, true);
            try { Icon = Icon.ExtractAssociatedIcon(Application.ExecutablePath); } catch { }
            try { string f = Path.Combine(Path.GetDirectoryName(Application.ExecutablePath), "qr_firebot.png"); if (File.Exists(f)) qr = Image.FromFile(f); } catch { qr = null; }
            try { string f = Path.Combine(Path.GetDirectoryName(Application.ExecutablePath), "noite_logo.png"); if (File.Exists(f)) noite = Image.FromFile(f); } catch { noite = null; }

            topo = new PainelDB { Dock = DockStyle.Top, Height = Px(96), BackColor = Color.FromArgb(8, 16, 13) };
            topo.Paint += PintarTopo;
            faixa = new Label { Dock = DockStyle.Top, Height = Px(46), TextAlign = ContentAlignment.MiddleCenter, BackColor = Cores.Amarelo, ForeColor = Color.FromArgb(30, 24, 0),
                                Font = new Font("Segoe UI", 16f, FontStyle.Bold), Text = "DEMONSTRAÇÃO COM VALORES SIMULADOS: o robô não está ligado", Visible = false };
            Panel palco = new Panel { Dock = DockStyle.Fill, BackColor = Cores.Fundo };

            // 1) vista geral: radar à esquerda; robô, chama e QR à direita
            RadarView radar = new RadarView(t, cfg, mv) { Zoom = 1.7f };
            RoboView robo = new RoboView(t, cfg, mv) { Zoom = 1.25f };
            ChamaView chama = new ChamaView(t, cfg, mv) { Zoom = 1.2f };
            Panel geral = new Panel { Dock = DockStyle.Fill, BackColor = Cores.Fundo };
            Panel lado = new Panel { Dock = DockStyle.Right, Width = Px(560), BackColor = Cores.Fundo, Padding = new Padding(Px(8)) };
            PainelDB painelQr = new PainelDB { Dock = DockStyle.Bottom, Height = Px(300), BackColor = Cores.Painel };
            painelQr.Paint += (s, e) => PintarQr(e.Graphics, painelQr.ClientRectangle, false);
            Panel pRobo = Moldura(robo); pRobo.Dock = DockStyle.Top; pRobo.Height = Px(330);
            Panel pChama = Moldura(chama); pChama.Dock = DockStyle.Fill;
            lado.Controls.Add(pChama); lado.Controls.Add(painelQr); lado.Controls.Add(pRobo);
            Panel pRadar = Moldura(radar); pRadar.Dock = DockStyle.Fill;
            geral.Controls.Add(pRadar); geral.Controls.Add(lado);
            Juntar(geral, radar, robo, chama);

            // 2) a 4) um painel de cada vez, em grande
            RadarView radarG = new RadarView(t, cfg, mv) { Zoom = 2.2f };
            Juntar(Moldura(radarG), radarG);
            RoboView roboG = new RoboView(t, cfg, mv) { Zoom = 2.4f };
            Juntar(Moldura(roboG), roboG);
            ChamaView chamaG = new ChamaView(t, cfg, mv) { Zoom = 2.2f };
            Juntar(Moldura(chamaG), chamaG);

            // 5) o QR em ecrã inteiro
            PainelDB qrG = new PainelDB { Dock = DockStyle.Fill, BackColor = Cores.Painel };
            qrG.Paint += (s, e) => PintarQr(e.Graphics, qrG.ClientRectangle, true);
            Juntar(qrG);

            foreach (Control c in slides) { c.Dock = DockStyle.Fill; c.Visible = false; palco.Controls.Add(c); }
            slides[0].Visible = true;
            Controls.Add(palco); Controls.Add(faixa); Controls.Add(topo);

            Timer tm = new Timer { Interval = 40 };
            tm.Tick += (s, e) =>
            {
                bool d = emDemo(), sim = tel.Recente && tel.ChamaSim;
                string txt = d ? "DEMONSTRAÇÃO COM VALORES SIMULADOS: o robô não está ligado" : "CHAMA SIMULADA: teste da sequência de alarme, sem vela";
                if (faixa.Text != txt) faixa.Text = txt;
                Color cor = d ? Cores.Amarelo : Cores.Laranja;
                if (faixa.BackColor != cor) faixa.BackColor = cor;
                if (faixa.Visible != (d || sim)) faixa.Visible = d || sim;
                foreach (Vista v in vistasDoSlide[atual]) v.Invalidate();
                DateTime agora = DateTime.Now;
                if ((agora - ultimaTecla).TotalSeconds > 30 && (agora - ultimaRotacao).TotalSeconds > 12) { ultimaRotacao = agora; Mudar(1); }
                string st = estado();
                if (st != ultimoEstado) { ultimoEstado = st; topo.Invalidate(); }
            };
            tm.Start();
            FormClosed += (s, e) => { tm.Stop(); if (qr != null) qr.Dispose(); if (noite != null) noite.Dispose(); };
            KeyUp += (s, e) => tecla(e.KeyCode, false);
            Resize += (s, e) =>
            {
                AjustarZoom();
                lado.Width = Math.Max(Px(420), (int)(ClientSize.Width * 0.37));
                pRobo.Height = Math.Max(Px(210), (int)(ClientSize.Height * 0.32));
                painelQr.Height = Math.Max(Px(190), (int)(ClientSize.Height * 0.28));
            };
        }

        void Juntar(Control slide, params Vista[] vs) { slides.Add(slide); vistasDoSlide.Add(vs); foreach (Vista v in vs) zooms.Add(new KeyValuePair<Vista, float>(v, v.Zoom)); }

        void AjustarZoom()
        {
            float z = Math.Min(ClientSize.Width / (1500f * Escala.F), ClientSize.Height / (1000f * Escala.F));   // tamanhos afinados num ecrã de 1500x1000 (lógicos)
            z = Math.Max(0.55f, Math.Min(1.6f, z));
            foreach (KeyValuePair<Vista, float> kv in zooms) kv.Key.Zoom = Math.Max(1f, kv.Value * z);
        }

        public void Mudar(int passo)
        {
            int novo = (atual + passo + slides.Count) % slides.Count;
            if (novo == atual) return;
            SuspendLayout();
            slides[novo].Visible = true;
            slides[atual].Visible = false;
            atual = novo;
            ResumeLayout(true);
            topo.Invalidate();
        }

        static int Px(int v) { return (int)Math.Round(v * Escala.F); }

        Panel Moldura(Control c)
        {
            Panel p = new Panel { Padding = new Padding(Px(6)), BackColor = Cores.Fundo };
            c.Dock = DockStyle.Fill; p.Controls.Add(c);
            return p;
        }

        public void AbrirTamanho(int l, int a)
        {
            StartPosition = FormStartPosition.Manual;
            Bounds = new Rectangle(0, 0, l, a);
            Show();
        }

        public void Abrir(Screen ecra)
        {
            StartPosition = FormStartPosition.Manual;
            Bounds = ecra.Bounds;
            Show();
            Activate();
        }

        // --captura=ficheiro.png: guarda uma imagem da janela (para verificar o aspeto sem olhar para o ecrã)
        public void GuardarImagem(string f)
        {
            using (Bitmap b = new Bitmap(ClientSize.Width, ClientSize.Height)) { DrawToBitmap(b, new Rectangle(Point.Empty, ClientSize)); b.Save(f); }
        }

        void PintarTopo(object s, PaintEventArgs e)
        {
            Graphics g = e.Graphics; g.SmoothingMode = SmoothingMode.AntiAlias; g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;
            Control c = (Control)s;
            g.Clear(c.BackColor);
            using (Font f1 = new Font("Consolas", 34f, FontStyle.Bold))
            using (Font f2 = new Font("Segoe UI", 13f))
            using (Font f3 = new Font("Segoe UI", 13f, FontStyle.Bold))
            using (StringFormat corta = new StringFormat(StringFormatFlags.NoWrap) { Trimming = StringTrimming.EllipsisCharacter })
            {
                // à direita: logótipo da Noite
                float direita = c.Width - Px(24);
                if (noite != null)
                {
                    float lh = c.Height - Px(20), lw = lh * noite.Width / noite.Height;
                    g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                    g.DrawImage(noite, direita - lw, Px(10), lw, lh);
                    direita -= lw + Px(24);
                }
                // linha 1 à direita: estado da ligação; linha 2 à direita: slide atual e pontos
                string st = estado();
                float xEstado = direita;
                if (!string.IsNullOrEmpty(st))
                {
                    SizeF m = g.MeasureString(st, f3);
                    xEstado = direita - m.Width;
                    using (SolidBrush b = new SolidBrush(st.StartsWith("Ligado") ? Cores.Verde : Cores.Amarelo)) g.DrawString(st, f3, b, xEstado, Px(16));
                }
                string nome = nomes[atual] + "   ← →";
                SizeF mn = g.MeasureString(nome, f2);
                float xSlide = direita - mn.Width - slides.Count * Px(16) - Px(12);
                using (SolidBrush b = new SolidBrush(Cores.Fraco)) g.DrawString(nome, f2, b, xSlide, Px(52));
                for (int i = 0; i < slides.Count; i++)
                {
                    float cx = direita - (slides.Count - i) * Px(16) + Px(8), cy = Px(52) + mn.Height / 2;
                    using (SolidBrush b = new SolidBrush(i == atual ? Cores.Laranja : Cores.Linha)) g.FillEllipse(b, cx - Px(5), cy - Px(5), Px(10), Px(10));
                }
                // à esquerda: FIREBOT e as duas linhas, cortadas antes do que está à direita
                float x = Px(24);
                SizeF a = g.MeasureString("FIRE", f1);
                using (SolidBrush b = new SolidBrush(Color.White)) g.DrawString("FIRE", f1, b, x, Px(14));
                using (SolidBrush b = new SolidBrush(Cores.Laranja)) g.DrawString("BOT", f1, b, x + a.Width - Px(14), Px(14));
                x += a.Width * 2 + Px(10);
                float l1 = Math.Max(0, xEstado - Px(30) - x), l2 = Math.Max(0, xSlide - Px(30) - x);
                using (SolidBrush b = new SolidBrush(Cores.Texto)) g.DrawString("Robô bombeiro com três sonares, giroscópio e sensor de chama por infravermelho.", f2, b, new RectangleF(x, Px(18), l1, Px(30)), corta);
                using (SolidBrush b = new SolidBrush(Cores.Fraco)) g.DrawString("Universidade Lusófona · Faculdade de Engenharia · Engenharia Biomédica", f2, b, new RectangleF(x, Px(50), l2, Px(30)), corta);
            }
        }

        void PintarQr(Graphics g, Rectangle area, bool grande)
        {
            g.SmoothingMode = SmoothingMode.AntiAlias; g.TextRenderingHint = TextRenderingHint.ClearTypeGridFit;
            g.Clear(Cores.Painel);
            int q = grande ? Math.Min((int)(area.Height * 0.78), (int)(area.Width * 0.45)) : Math.Min(area.Height - Px(40), (int)(area.Width * 0.5));
            int x = grande ? (int)(area.Width * 0.08) : Px(20), y = (area.Height - q) / 2;
            if (qr != null)
            {
                g.InterpolationMode = InterpolationMode.NearestNeighbor;
                g.PixelOffsetMode = PixelOffsetMode.Half;
                g.FillRectangle(Brushes.White, x, y, q, q);
                g.DrawImage(qr, x, y, q, q);
            }
            float tx = x + (qr != null ? q + Px(grande ? 60 : 18) : 0), largura = area.Width - tx - Px(16);
            if (largura < Px(60)) return;
            string t1 = "Aponta a câmara do telemóvel", t2 = "Explora o robô peça a peça e, no fim, faz o desafio.";
            // letra à medida: começa grande e encolhe até os três textos caberem na altura do QR
            float k = grande ? 1.9f : Math.Max(0.6f, Math.Min(1.2f, area.Height / (float)Px(300)));
            for (int tent = 0; tent < 12; tent++, k *= 0.9f)
            {
                using (Font f1 = new Font("Segoe UI", 19f * k, FontStyle.Bold))
                using (Font f2 = new Font("Segoe UI", 12.5f * k))
                using (Font f3 = new Font("Consolas", (grande ? 11.5f : 9.5f) * k, FontStyle.Bold))
                {
                    SizeF h1 = g.MeasureString(t1, f1, (int)largura), h2 = g.MeasureString(t2, f2, (int)largura);
                    SizeF h3 = g.MeasureString(Url, f3, (int)largura);
                    float total = h1.Height + Px(8) + h2.Height + Px(12) + h3.Height;
                    if (total > q && tent < 11) continue;
                    float ty = y + Math.Max(0, (q - total) / 2);
                    using (SolidBrush b = new SolidBrush(Color.White)) g.DrawString(t1, f1, b, new RectangleF(tx, ty, largura, h1.Height + 2));
                    ty += h1.Height + Px(8);
                    using (SolidBrush b = new SolidBrush(Cores.Texto)) g.DrawString(t2, f2, b, new RectangleF(tx, ty, largura, h2.Height + 2));
                    ty += h2.Height + Px(12);
                    using (SolidBrush b = new SolidBrush(Cores.Laranja)) g.DrawString(Url, f3, b, new RectangleF(tx, ty, largura, h3.Height + 2));
                    return;
                }
            }
        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            ultimaTecla = DateTime.Now;
            if (keyData == Keys.F11 || keyData == Keys.Escape) { Close(); return true; }
            if (keyData == Keys.Right || keyData == Keys.PageDown) { Mudar(1); return true; }
            if (keyData == Keys.Left || keyData == Keys.PageUp) { Mudar(-1); return true; }
            if (keyData == Keys.Up || keyData == Keys.Down) return true; // setas ficam para os slides; conduzir com W A S D
            if (tecla(keyData, true)) return true;
            return base.ProcessCmdKey(ref msg, keyData);
        }
    }
}
