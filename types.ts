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
  serviceUUID: string;
  characteristicUUID: string;
}

export interface LogEntry {
  timestamp: number;
  message: string;
  type: 'info' | 'error' | 'success' | 'warning';
}