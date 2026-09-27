// Ligação Bluetooth Low Energy ao módulo BT24 (canal de dados FFE1), para a app funcionar sem cabo.
// Compila com o csc do .NET Framework 4 e os .winmd do Windows (sem SDK): as esperas são feitas à mão.
using System;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Storage.Streams;

namespace FirebotApp
{
    class LigacaoBle
    {
        public event Action<string> Recebido;   // chamado noutra thread
        public event Action Perdida;           // a ligação caiu

        BluetoothLEDevice dev;
        GattCharacteristic uart;               // guardado aqui para os eventos não se perderem
        public volatile bool Ligado;
        public string Nome = "";

        static Task<T> Esperar<T>(Windows.Foundation.IAsyncOperation<T> op)
        {
            var tcs = new TaskCompletionSource<T>();
            op.Completed = (info, estado) =>
            {
                if (estado == Windows.Foundation.AsyncStatus.Completed) tcs.TrySetResult(info.GetResults());
                else tcs.TrySetException(new Exception("Bluetooth: " + estado + (info.ErrorCode != null ? " " + info.ErrorCode.Message : "")));
            };
            return tcs.Task;
        }

        static byte[] Bytes(IBuffer b) { var r = DataReader.FromBuffer(b); var a = new byte[b.Length]; r.ReadBytes(a); return a; }
        static IBuffer ParaBuffer(byte[] a, int ini, int n) { var w = new DataWriter(); var p = new byte[n]; Array.Copy(a, ini, p, 0, n); w.WriteBytes(p); return w.DetachBuffer(); }

        // Procura o módulo pelo nome e liga-se. Devolve null se correu bem, ou a mensagem de erro.
        public async Task<string> Ligar(string nome)
        {
            Nome = nome;
            var achado = new TaskCompletionSource<ulong>();
            var w = new BluetoothLEAdvertisementWatcher();
            w.ScanningMode = BluetoothLEScanningMode.Active;
            w.Received += (s, e) => { if (e.Advertisement.LocalName == nome) achado.TrySetResult(e.BluetoothAddress); };
            w.Start();
            Task fim = await Task.WhenAny(achado.Task, Task.Delay(12000));
            w.Stop();
            if (fim != achado.Task) return "Não encontrei o '" + nome + "'. Está ligado (LED a piscar) e perto do PC?";

            dev = await Esperar(BluetoothLEDevice.FromBluetoothAddressAsync(achado.Task.Result));
            if (dev == null) return "Não consegui abrir o '" + nome + "'.";
            dev.ConnectionStatusChanged += (s, e) =>
            {
                if (s.ConnectionStatus == BluetoothConnectionStatus.Disconnected && Ligado)
                {
                    Ligado = false;
                    var p = Perdida; if (p != null) p();
                }
            };

            GattDeviceServicesResult sr = await Esperar(dev.GetGattServicesAsync(BluetoothCacheMode.Uncached));
            if (sr.Status != GattCommunicationStatus.Success) return "Não consegui ler os serviços do '" + nome + "' (" + sr.Status + ").";
            foreach (GattDeviceService s in sr.Services)
            {
                GattCharacteristicsResult cr = await Esperar(s.GetCharacteristicsAsync(BluetoothCacheMode.Uncached));
                foreach (GattCharacteristic c in cr.Characteristics)
                {
                    var p = c.CharacteristicProperties;
                    bool notifica = (p & GattCharacteristicProperties.Notify) != 0;
                    bool escreve = (p & (GattCharacteristicProperties.Write | GattCharacteristicProperties.WriteWithoutResponse)) != 0;
                    if (uart == null && notifica && escreve) uart = c;
                }
            }
            if (uart == null) return "O '" + nome + "' não tem canal de dados (notify + write).";

            uart.ValueChanged += (s, e) =>
            {
                var r = Recebido;
                if (r != null) r(Encoding.ASCII.GetString(Bytes(e.CharacteristicValue)));
            };
            var st = await Esperar(uart.WriteClientCharacteristicConfigurationDescriptorAsync(GattClientCharacteristicConfigurationDescriptorValue.Notify));
            if (st != GattCommunicationStatus.Success) return "O '" + nome + "' recusou enviar dados (" + st + ").";
            Ligado = true;
            return null;
        }

        // Envia texto em pedaços de 20 bytes (o tamanho seguro de um pacote BLE)
        public void Enviar(string txt)
        {
            if (!Ligado || uart == null) return;
            byte[] b = Encoding.ASCII.GetBytes(txt);
            for (int i = 0; i < b.Length; i += 20)
            {
                int n = Math.Min(20, b.Length - i);
                try { var op = uart.WriteValueAsync(ParaBuffer(b, i, n), GattWriteOption.WriteWithoutResponse); }
                catch { }
            }
        }

        public void Desligar()
        {
            Ligado = false;
            try { if (dev != null) dev.Dispose(); } catch { }
            dev = null; uart = null;
        }
    }
}
