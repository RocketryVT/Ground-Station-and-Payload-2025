<script lang="ts">
  import { onMount } from "svelte";
  import {
    SerialPort,
    ClearBuffer,
    DataBits,
    FlowControl,
    Parity,
    StopBits,
    type PortInfo,
  } from "tauri-plugin-serialplugin";

  interface PortSettings {
    name: string;
    baudRate: number;
    lineEnding: string;
    dataBits: DataBits;
    stopBits: StopBits;
    parity: Parity;
    flowControl: FlowControl;
    timeout: number;
    isConnected: boolean;
    receivedData: string;
    bytesToRead: number;
    bytesToWrite: number;
    rtsState: boolean;
    dtrState: boolean;
    ctsState: boolean;
    dsrState: boolean;
    riState: boolean;
    cdState: boolean;
    message: string;
  }

  let ports: { [key: string]: PortInfo } = $state({});
  let selectedPort = $state("");
  let connectedDevices: {
    [key: string]: { port: SerialPort; settings: PortSettings };
  } = $state({});

  let baudRate: number = $state(9600);
  let dataBits = $state(DataBits.Eight);
  let flowControl = $state(FlowControl.None);
  let parity = $state(Parity.None);
  let stopBits = $state(StopBits.One);
  let timeout: number = $state(1000);

  const dataBitsOptions: DataBits[] = [
    DataBits.Five,
    DataBits.Six,
    DataBits.Seven,
    DataBits.Eight,
  ];
  const flowControlOptions: FlowControl[] = [
    FlowControl.None,
    FlowControl.Software,
    FlowControl.Hardware,
  ];
  const parityOptions: Parity[] = [Parity.None, Parity.Odd, Parity.Even];
  const stopBitsOptions: StopBits[] = [StopBits.One, StopBits.Two];

  async function scanPorts() {
    try {
      ports = await SerialPort.available_ports();
      console.log("Available ports");
    } catch (err) {
      console.error("Failed to scan ports:", err);
    }
  }

  async function connect(settings: PortSettings) {
    try {
      const serialport = new SerialPort({
        path: settings.name,
        baudRate: settings.baudRate,
        dataBits: settings.dataBits,
        flowControl: settings.flowControl,
        parity: settings.parity,
        stopBits: settings.stopBits,
        timeout: settings.timeout,
      });

      await serialport.open();
      settings.isConnected = true;
      console.log("Connected to port:", settings.name);

      await serialport.startListening();

      // Start listening for data
      serialport.listen((data) => {
        console.log("listen", data);
        settings.receivedData += data;
        updatePortStatus(settings);
        console.log("Received data:", settings.receivedData);
        connectedDevices = { ...connectedDevices };
      });

      // Listen for disconnection
      serialport.disconnected(() => {
        settings.isConnected = false;
        console.log("Port disconnected:", settings.name);
        delete connectedDevices[settings.name];
        connectedDevices = { ...connectedDevices };
      });

      connectedDevices = { ...connectedDevices, [settings.name]: { port: serialport, settings } };
    } catch (err) {
      console.error("Failed to connect:", err);
    }
  }

  async function connectToRFD900x() {
    try {
      const portName = Object.keys(ports).find(
        (name) => ports[name].serial_number === "A10O9AMR"
      );
      if (!portName) {
        console.error("RFD900x not found");
        return;
      }

      const settings: PortSettings = {
        name: portName,
        baudRate: 57600,
        dataBits: DataBits.Eight,
        flowControl: FlowControl.None,
        parity: Parity.None,
        stopBits: StopBits.One,
        timeout: 1000,
        isConnected: false,
        receivedData: "",
        bytesToRead: 0,
        bytesToWrite: 0,
        rtsState: false,
        dtrState: false,
        ctsState: false,
        dsrState: false,
        riState: false,
        cdState: false,
        lineEnding: "",
        message: "",
      };

      await connect(settings);
    } catch (err) {
      console.error("Failed to connect to RFD900x:", err);
    }
  }

  async function disconnect(settings: PortSettings) {
    try {
      const device = connectedDevices[settings.name];
      if (device) {
        await device.port.close();
        settings.isConnected = false;
        console.log("Disconnected from port:", settings.name);
        delete connectedDevices[settings.name];
        connectedDevices = { ...connectedDevices };
      }
    } catch (err) {
      console.error("Failed to disconnect:", err);
    }
  }

  async function readData(settings: PortSettings) {
    try {
      const device = connectedDevices[settings.name];
      if (device) {
        const data = await device.port.read();
        console.log("Data read:", data);
        settings.receivedData += data;
        updatePortStatus(settings);
        connectedDevices = { ...connectedDevices };
      }
    } catch (err) {
      console.error("Failed to read data:", err);
    }
  }

  async function sendMessage(settings: PortSettings) {
    try {
      const device = connectedDevices[settings.name];
      if (device) {
        await device.port.write(settings.message);
        console.log("Message sent:", settings.message);
        settings.message = "";
      }
    } catch (err) {
      console.error("Failed to send message:", err);
    }
}

  async function toggleRTS(settings: PortSettings) {
    try {
      const device = connectedDevices[settings.name];
      if (device) {
        settings.rtsState = !settings.rtsState;
        await device.port.setRequestToSend(settings.rtsState);
        console.log("RTS toggled:", settings.rtsState);
      }
    } catch (err) {
      console.error("Failed to toggle RTS:", err);
    }
  }

  async function updatePortStatus(settings: PortSettings) {
    const device = connectedDevices[settings.name];
    if (device) {
      try {
        settings.bytesToRead = await device.port.bytesToRead();
        settings.bytesToWrite = await device.port.bytesToWrite();
        settings.ctsState = await device.port.readClearToSend();
        settings.dsrState = await device.port.readDataSetReady();
        settings.riState = await device.port.readRingIndicator();
        settings.cdState = await device.port.readCarrierDetect();
      } catch (err) {
        console.error("Failed to update port status:", err);
      }
    }
  }

  onMount(() => {
    scanPorts();
  });
</script>

<style>
    .port-list, .connected-list {
      list-style-type: none;
      padding: 0;
    }
  
    .port-list li, .connected-list li {
      margin: 5px 0;
    }
  
    .received-data {
      white-space: pre-wrap;
      /* background-color: #f0f0f0; */
      /* color: #000; */
      padding: 10px;
      border: 1px solid #ccc;
      margin-top: 10px;
    }
</style>

<div>
    <button onclick={scanPorts}>Scan Ports</button>
    <button onclick={connectToRFD900x}>Connect to RFD900x</button>
  
    <h2>Available Ports</h2>
    <ul class="port-list">
      {#each Object.keys(ports).sort() as portName}
        <li>
          <input type="radio" bind:group={selectedPort} value={portName} />
          {portName}
        </li>
      {/each}
    </ul>

    {#if selectedPort}
    <h3>Configure Port Settings</h3>
    <div>
      <label>
        Baud Rate:
        <input type="number" bind:value={baudRate} />
      </label>
    </div>
    <div>
      <label>
        Data Bits:
        <select bind:value={dataBits}>
          {#each dataBitsOptions as option}
            <option value={option}>{option}</option>
          {/each}
        </select>
      </label>
    </div>
    <div>
      <label>
        Flow Control:
        <select bind:value={flowControl}>
          {#each flowControlOptions as option}
            <option value={option}>{option}</option>
          {/each}
        </select>
      </label>
    </div>
    <div>
      <label>
        Parity:
        <select bind:value={parity}>
          {#each parityOptions as option}
            <option value={option}>{option}</option>
          {/each}
        </select>
      </label>
    </div>
    <div>
        <label>
            Stop Bits:
            <select bind:value={stopBits}>
                {#each stopBitsOptions as option}
                <option value={option}>{option}</option>
                {/each}
            </select>
            </label>
        </div>
        <div>
            <label>
            Timeout:
            <input type="number" bind:value={timeout} />
            </label>
        </div>
        <button onclick={() => connect({
          name: selectedPort,
          baudRate,
          dataBits,
          flowControl,
          parity,
          stopBits,
          timeout,
          isConnected: false,
          receivedData: "",
          bytesToRead: 0,
          bytesToWrite: 0,
          rtsState: false,
          dtrState: false,
          ctsState: false,
          dsrState: false,
          riState: false,
          cdState: false,
          lineEnding: "",
          message: ""
        })} disabled={connectedDevices[selectedPort]?.settings?.isConnected}>Connect</button>
        <button onclick={() => disconnect(connectedDevices[selectedPort]?.settings)} disabled={!connectedDevices[selectedPort]?.settings?.isConnected}>Disconnect</button>
        {/if}
  
  <h2>Connected Devices</h2>
  <ul class="connected-list">
    {#each Object.keys(connectedDevices) as deviceName}
      <li>
        <div>
          <strong>{deviceName}</strong>
          <input type="text" bind:value={connectedDevices[deviceName].settings.message} placeholder="Enter message" />
          <button onclick={() => sendMessage(connectedDevices[deviceName].settings)}>Send Message</button>
        </div>
        <div class="received-data">{connectedDevices[deviceName].settings.receivedData}</div>
      </li>
    {/each}
  </ul>
</div>