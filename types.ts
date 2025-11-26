export interface SensorData {
  x: number;
  y: number;
  z: number;
  mag: number;
}

export type ScreenState = 'setup' | 'game' | 'settings';

export interface GameSettings {
  sensitivity: number; // 1 to 100
  threshold: number; // minimum G-force to trigger punch
  calibration: number; // Baseline force to subtract (e.g. 9.8 or 90)
  serviceUUID: string;
  characteristicUUID: string;
  sfxVolume: number; // 0 to 100
  musicVolume: number; // 0 to 100
}

export interface LogEntry {
  timestamp: number;
  message: string;
  type: 'info' | 'error' | 'success' | 'warning';
}