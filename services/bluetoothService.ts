import { SensorData, LogEntry } from '../types';

// --- Web Bluetooth Type Definitions ---

interface BluetoothCharacteristicProperties {
  broadcast: boolean;
  read: boolean;
  writeWithoutResponse: boolean;
  write: boolean;
  notify: boolean;
  indicate: boolean;
  authenticatedSignedWrites: boolean;
  reliableWrite: boolean;
  writableAuxiliaries: boolean;
}

interface BluetoothRemoteGATTDescriptor {
  characteristic: BluetoothRemoteGATTCharacteristic;
  uuid: string;
  value?: DataView;
  readValue(): Promise<DataView>;
  writeValue(value: BufferSource): Promise<void>;
}

interface BluetoothRemoteGATTCharacteristic extends EventTarget {
  service: BluetoothRemoteGATTService;
  uuid: string;
  properties: BluetoothCharacteristicProperties;
  value?: DataView;
  getDescriptor(descriptor: string | number): Promise<BluetoothRemoteGATTDescriptor>;
  readValue(): Promise<DataView>;
  writeValue(value: BufferSource): Promise<void>;
  startNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
  stopNotifications(): Promise<BluetoothRemoteGATTCharacteristic>;
  addEventListener(type: string, listener: EventListenerOrEventListenerObject, options?: boolean | AddEventListenerOptions): void;
}

interface BluetoothRemoteGATTService extends EventTarget {
  uuid: string;
  device: BluetoothDevice;
  isPrimary: boolean;
  getCharacteristic(characteristic: string | number): Promise<BluetoothRemoteGATTCharacteristic>;
  getCharacteristics(characteristic?: string | number): Promise<BluetoothRemoteGATTCharacteristic[]>;
  getIncludedService(service: string | number): Promise<BluetoothRemoteGATTService>;
  getIncludedServices(service?: string | number): Promise<BluetoothRemoteGATTService[]>;
}

interface BluetoothRemoteGATTServer {
  device: BluetoothDevice;
  connected: boolean;
  connect(): Promise<BluetoothRemoteGATTServer>;
  disconnect(): void;
  getPrimaryService(service: string | number): Promise<BluetoothRemoteGATTService>;
  getPrimaryServices(service?: string | number): Promise<BluetoothRemoteGATTService[]>;
}

interface BluetoothDevice extends EventTarget {
  id: string;
  name?: string;
  gatt?: BluetoothRemoteGATTServer;
  watchAdvertisements(): Promise<void>;
  unwatchAdvertisements(): void;
  readonly watchingAdvertisements: boolean;
  addEventListener(type: string, listener: EventListenerOrEventListenerObject, options?: boolean | AddEventListenerOptions): void;
}

interface RequestDeviceOptions {
  filters?: any[];
  optionalServices?: (string | number)[];
  acceptAllDevices?: boolean;
}

interface Bluetooth extends EventTarget {
  getAvailability(): Promise<boolean>;
  requestDevice(options?: RequestDeviceOptions): Promise<BluetoothDevice>;
}

declare global {
  interface Navigator {
    bluetooth: Bluetooth;
  }
}

// --------------------------------------

type DataCallback = (data: SensorData) => void;
type LogCallback = (entry: LogEntry) => void;

class BluetoothService {
  private device: BluetoothDevice | null = null;
  private server: BluetoothRemoteGATTServer | null = null;
  private characteristic: BluetoothRemoteGATTCharacteristic | null = null;
  
  private dataCallback: DataCallback | null = null;
  private disconnectCallback: (() => void) | null = null;
  private logCallbacks: LogCallback[] = [];

  private isSimulating: boolean = false;
  private simulationTimeout: number | null = null;
  private simulationInterval: number | null = null;

  private decoder = new TextDecoder('utf-8');
  private lastLogTime = 0;

  // Default UUIDs (HM-10 / MLT-BT05 style 0xFFE0 standard)
  // Full UUID: 0000ffe0-0000-1000-8000-00805f9b34fb
  private serviceUUID: string = '0000ffe0-0000-1000-8000-00805f9b34fb';
  private characteristicUUID: string = '0000ffe1-0000-1000-8000-00805f9b34fb';

  // For simulation
  private mockX = 0;
  private mockY = 0;
  private mockZ = 0;

  private normalizeUUID(uuid: string): string {
    // If it's a short 16-bit UUID (e.g., "FFE0" or "0xFFE0"), expand it
    if (uuid.length <= 8) {
        const clean = uuid.replace('0x', '').toLowerCase();
        return `0000${clean}-0000-1000-8000-00805f9b34fb`;
    }
    return uuid.toLowerCase();
  }

  public getTargetServiceUUID(): string {
    return this.serviceUUID;
  }

  public setUUIDs(service: string, characteristic: string) {
    this.serviceUUID = this.normalizeUUID(service);
    this.characteristicUUID = this.normalizeUUID(characteristic);
  }

  public onLog(callback: LogCallback) {
    this.logCallbacks.push(callback);
  }

  public onDisconnect(callback: () => void) {
    this.disconnectCallback = callback;
  }

  private log(message: string, type: 'info' | 'error' | 'success' | 'warning' = 'info') {
    const entry: LogEntry = { timestamp: Date.now(), message, type };
    console.log(`[BLE][${type}] ${message}`);
    this.logCallbacks.forEach(cb => cb(entry));
  }

  private wait(ms: number) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  public async connect(): Promise<boolean> {
    try {
      if (this.isSimulating) this.stopSimulation();

      // Check if running in iframe (like VS Code Simple Browser)
      if (window.self !== window.top) {
          this.log("Running in iframe/embedded view. Web Bluetooth might be blocked.", 'warning');
          alert("Please open this app in a separate browser window (Chrome/Edge). VS Code's preview blocks Bluetooth.");
      }

      if (!navigator.bluetooth) {
        this.log("Web Bluetooth not supported.", 'error');
        alert("Web Bluetooth is not supported in this browser.");
        return false;
      }

      this.log("Scanning for devices...", 'info');
      
      // Robust filter configuration
      // We ask for the specific service in optionalServices to ensure access
      this.device = await navigator.bluetooth.requestDevice({
        filters: [
          { name: 'BUNKERBOX' }
        ],
        optionalServices: [0xFFE0, '0000ffe0-0000-1000-8000-00805f9b34fb']
      });

      if (!this.device) {
        this.log("No device selected.", 'warning');
        return false;
      }

      this.log(`Selected: ${this.device.name || 'Unknown Device'}`, 'success');

      if (!this.device.gatt) {
        this.log('Device has no GATT server.', 'error');
        throw new Error('Device has no GATT server');
      }

      this.device.addEventListener('gattserverdisconnected', this.onDisconnected);

      this.log('Connecting to GATT Server...', 'info');
      
      let connected = false;
      let attempts = 0;
      while (!connected && attempts < 3) {
        try {
            attempts++;
            this.server = await this.device.gatt.connect();
            connected = true;
            this.log(`GATT Connected! (Attempt ${attempts})`, 'success');
        } catch (connErr: any) {
            this.log(`Connection attempt ${attempts} failed: ${connErr.message}`, 'warning');
            if (attempts >= 3) throw connErr;
            await this.wait(1000);
        }
      }

      await this.setupGatt();

      return true;

    } catch (error: any) {
      if (error.name === 'NotFoundError' || error.message.includes('User cancelled')) {
        this.log('User cancelled selection.', 'warning');
      } else if (error.name === 'SecurityError' || error.name === 'NotSupportedError') {
        this.log(`Browser blocked request: ${error.message}`, 'error');
      } else {
        this.log(`Connection Error: ${error.message}`, 'error');
      }
      return false; 
    }
  }

  public async retryServiceConnection() {
    if (!this.server || !this.server.connected) {
        this.log("Cannot retry: Not connected.", 'error');
        return false;
    }
    this.log("Retrying sensor discovery...", 'info');
    await this.setupGatt();
  }

  private async setupGatt() {
    if (!this.server) return;
    this.log('Waiting 1.5s for stabilization...', 'info');
    await this.wait(1500);

    try {
      this.log(`Starting Service Discovery...`, 'info');
      
      let service: BluetoothRemoteGATTService | null = null;
      let services: BluetoothRemoteGATTService[] = [];

      try {
          services = await this.server.getPrimaryServices();
      } catch (e) {
          this.log("Generic scan failed, retrying...", 'warning');
          await this.wait(1000);
          try { services = await this.server.getPrimaryServices(); } catch(e2) {}
      }

      if (services.length > 0) {
          services.forEach(s => this.log(`> Found: ${s.uuid}`, 'info'));
          const target = this.serviceUUID.toLowerCase();
          const targetShort = target.substring(4, 8); 
          
          service = services.find(s => {
            const uuid = s.uuid.toLowerCase();
            return uuid === target || uuid.includes(targetShort) || uuid === '0000ffe0-0000-1000-8000-00805f9b34fb';
          }) || null;
      }

      if (!service) {
         this.log("Service not in list. Attempting direct access...", 'warning');
         try {
             service = await this.server.getPrimaryService(0xFFE0);
             this.log("Success via Short Alias (0xFFE0)!", 'success');
         } catch (e1) {
             try {
                service = await this.server.getPrimaryService(this.serviceUUID);
                this.log("Success via Configured UUID!", 'success');
             } catch (e2) {
                 this.log("Direct access failed.", 'error');
             }
         }
      }

      if (!service) {
         throw new Error(`Could not find Service ${this.serviceUUID.substring(0,8)}...`);
      }
      
      this.log(`Service Attached: ${service.uuid}`, 'success');

      this.log('Fetching Characteristic...', 'info');
      let char: BluetoothRemoteGATTCharacteristic | undefined;

      try {
        char = await service.getCharacteristic(this.characteristicUUID);
      } catch (e) {
        this.log(`Specific char failed, trying standard 0xFFE1...`, 'warning');
        try {
            char = await service.getCharacteristic('0000ffe1-0000-1000-8000-00805f9b34fb');
        } catch (e2) {
            const chars = await service.getCharacteristics();
            chars.forEach(c => this.log(`> Found Char: ${c.uuid}`, 'info'));
            char = chars.find(c => c.properties.notify);
        }
      }

      if (!char) throw new Error("No suitable Characteristic found.");

      this.characteristic = char;
      this.log(`Characteristic attached: ${char.uuid.substring(0,8)}...`, 'success');

      this.log('Starting Notifications...', 'info');
      try {
          await this.characteristic.startNotifications();
      } catch (notifyErr: any) {
          this.log(`Notify failed (${notifyErr.message}). Retrying in 1s...`, 'warning');
          await this.wait(1000);
          await this.characteristic.startNotifications();
      }
      
      this.characteristic.addEventListener('characteristicvaluechanged', this.handleCharacteristicValueChanged);
      
      this.log('SENSORS ACTIVE. BINARY MODE.', 'success');

    } catch (e: any) {
      this.log(`Setup Error: ${e.message}`, 'error');
      throw e;
    }
  }

  private onDisconnected = () => {
    this.log('Device Disconnected.', 'warning');
    if (this.disconnectCallback) this.disconnectCallback();
  };

  public startDemoMode() {
    this.startSimulation();
    return true;
  }

  public disconnect() {
    if (this.device) {
      this.device.removeEventListener('gattserverdisconnected', this.onDisconnected);
      if (this.device.gatt && this.device.gatt.connected) {
        this.device.gatt.disconnect();
      }
    }
    if (this.characteristic) {
        try {
            this.characteristic.removeEventListener('characteristicvaluechanged', this.handleCharacteristicValueChanged);
        } catch (e) {
            // Ignore if already removed or invalid
        }
    }
    this.stopSimulation();
    this.device = null;
    this.server = null;
    this.characteristic = null;
    this.log('Disconnected cleanly.', 'info');
  }

  public onDataReceived(callback: DataCallback) {
    this.dataCallback = callback;
  }

  private handleCharacteristicValueChanged = (event: Event) => {
    const target = event.target as BluetoothRemoteGATTCharacteristic;
    const value = target.value;
    if (!value) return;

    let x = 0, y = 0, z = 0;
    let parsed = false;

    // Debug: Log raw packet size occasionally
    const now = Date.now();
    // if (now - this.lastLogTime > 5000) {
    //    console.log(`[BLE] Packet Size: ${value.byteLength}`);
    // }

    // OPTIMIZATION: Check for Binary Format First (Fixed 6 bytes)
    if (value.byteLength === 6) {
        const rawX = value.getInt16(0, true);
        const rawY = value.getInt16(2, true);
        const rawZ = value.getInt16(4, true);
        
        const LSB_PER_G = 2048.0;
        const GRAVITY_MS2 = 9.80665;
        const factor = GRAVITY_MS2 / LSB_PER_G;

        x = rawX * factor;
        y = rawY * factor;
        z = rawZ * factor;
        parsed = true;
    } else {
        // Fallback: Try Text Format
        try {
            const text = this.decoder.decode(value);
            if (text.includes('X:') && text.includes('Y:')) {
                const parts = text.split(' ');
                parts.forEach(p => {
                    if (p.startsWith('X:')) x = parseInt(p.substring(2));
                    if (p.startsWith('Y:')) y = parseInt(p.substring(2));
                    if (p.startsWith('Z:')) z = parseInt(p.substring(2));
                });
                parsed = true;
            }
        } catch (e) {
            // Ignore
        }
    }

    // Fallback: Binary with extra bytes
    if (!parsed && value.byteLength >= 6) {
        const rawX = value.getInt16(0, true);
        const rawY = value.getInt16(2, true);
        const rawZ = value.getInt16(4, true);
        
        const LSB_PER_G = 2048.0;
        const GRAVITY_MS2 = 9.80665;
        const factor = GRAVITY_MS2 / LSB_PER_G;

        x = rawX * factor;
        y = rawY * factor;
        z = rawZ * factor;
        parsed = true;
    }

    if (parsed) {
        this.emitData(x, y, z);
        
        if (now - this.lastLogTime > 2000) {
            console.log(`[BLE] Data: X=${x.toFixed(1)} Y=${y.toFixed(1)} Z=${z.toFixed(1)}`);
            this.lastLogTime = now;
        }
    } else {
        // Log unparsed data to help debug
        if (now - this.lastLogTime > 2000) {
            console.warn(`[BLE] Unparsed Data (${value.byteLength} bytes)`);
            this.lastLogTime = now;
        }
    }
  };

  private startSimulation() {
    this.isSimulating = true;
    this.log('Starting Demo Simulation (Varying punches)...', 'info');
    
    // Initial loop start
    this.scheduleNextSimPunch();
    
    // Start a fast interval just for sensor decay, separate from punch logic
    // This keeps the values settling back to 0 smoothly
    if (this.simulationInterval) clearInterval(this.simulationInterval);
    this.simulationInterval = window.setInterval(() => {
        if (!this.isSimulating) return;
        const noise = () => (Math.random() - 0.5) * 0.5;
        this.mockX *= 0.8;
        this.mockY *= 0.8;
        this.mockZ *= 0.8;
        this.emitData(this.mockX + noise(), this.mockY + noise(), this.mockZ + noise());
    }, 20);
  }
  
  private scheduleNextSimPunch() {
      if (!this.isSimulating) return;
      
      // Randomize delay between 6 and 10 seconds for organic feel
      const delay = 6000 + Math.random() * 4000;
      
      this.simulationTimeout = window.setTimeout(() => {
          if (!this.isSimulating) return;
          
          // Generate Varying Punch Strength
          const tier = Math.random();
          let baseForce = 0;
          let logMsg = "";
          
          if (tier < 0.3) {
             // WIMP / WEAK
             baseForce = 20 + Math.random() * 15;
             logMsg = "Sim: Weak Hit";
          } else if (tier < 0.6) {
             // FIGHTER / MEDIUM
             baseForce = 45 + Math.random() * 20;
             logMsg = "Sim: Medium Hit";
          } else if (tier < 0.9) {
             // BRUISER / STRONG
             baseForce = 75 + Math.random() * 25;
             logMsg = "Sim: Heavy Hit";
          } else {
             // LEGEND / CRITICAL
             baseForce = 110 + Math.random() * 30;
             logMsg = "Sim: CRITICAL HIT";
          }
          
          // Apply force to mock axes
          this.mockX = baseForce * 0.6;
          this.mockY = baseForce * 0.6;
          this.mockZ = baseForce * 0.5;
          
          // console.log(logMsg, baseForce.toFixed(1));
          
          // Schedule next
          this.scheduleNextSimPunch();
      }, delay);
  }

  private stopSimulation() {
    if (this.simulationTimeout) {
      clearTimeout(this.simulationTimeout);
      this.simulationTimeout = null;
    }
    if (this.simulationInterval) {
      clearInterval(this.simulationInterval);
      this.simulationInterval = null;
    }
    this.isSimulating = false;
  }

  private emitData(x: number, y: number, z: number) {
    const mag = Math.sqrt(x*x + y*y + z*z);
    if (this.dataCallback) {
      this.dataCallback({ x, y, z, mag });
    } else {
      // console.warn("[BLE] No data callback registered!");
    }
  }
}

export const bluetoothService = new BluetoothService();
