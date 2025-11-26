import React, { useState, useEffect, useCallback } from 'react';
import { NeonBackground } from './components/NeonBackground';
import { SetupScreen } from './components/SetupScreen';
import { GameScreen } from './components/GameScreen';
import { SettingsScreen } from './components/SettingsScreen';
import { BackgroundMusic } from './components/BackgroundMusic';
import { bluetoothService } from './services/bluetoothService';
import { setVolumes, playConnectionSound } from './services/audioService';
import { ScreenState, SensorData, GameSettings } from './types';

// Default to standard HM-10 / generic BLE module UUIDs as requested (0xFFE0 / 0xFFE1)
const DEFAULT_SETTINGS: GameSettings = {
  sensitivity: 50,
  threshold: 20,
  calibration: 9.8,
  serviceUUID: '0000ffe0-0000-1000-8000-00805f9b34fb',
  characteristicUUID: '0000ffe1-0000-1000-8000-00805f9b34fb',
  sfxVolume: 50,
  musicVolume: 50
};

const App: React.FC = () => {
  const [currentScreen, setCurrentScreen] = useState<ScreenState>('setup');
  const [isConnected, setIsConnected] = useState(false);
  const [sensorData, setSensorData] = useState<SensorData>({ x: 0, y: 0, z: 0, mag: 0 });
  const lastDataTimeRef = React.useRef<number>(Date.now());
  
  // Load settings from localStorage or use defaults
  const [settings, setSettings] = useState<GameSettings>(() => {
    try {
      const saved = localStorage.getItem('bunker_settings');
      return saved ? { ...DEFAULT_SETTINGS, ...JSON.parse(saved) } : DEFAULT_SETTINGS;
    } catch (e) {
      return DEFAULT_SETTINGS;
    }
  });

  // Update Bluetooth Service and Audio when settings change
  useEffect(() => {
    bluetoothService.setUUIDs(settings.serviceUUID, settings.characteristicUUID);
    setVolumes(settings.sfxVolume ?? 50, settings.musicVolume ?? 50);
    localStorage.setItem('bunker_settings', JSON.stringify(settings));
  }, [settings]);

  // Setup Bluetooth Listener
  useEffect(() => {
    bluetoothService.onDataReceived((data) => {
      setSensorData(data);
      lastDataTimeRef.current = Date.now();
    });

    bluetoothService.onDisconnect(() => {
        setIsConnected(false);
        setCurrentScreen('setup');
        playConnectionSound(false);
    });
    
    return () => {
      bluetoothService.disconnect();
    };
  }, []);

  // Connection Watchdog: Disconnect if no data for 5 seconds
  useEffect(() => {
      const watchdog = setInterval(() => {
          if (isConnected && Date.now() - lastDataTimeRef.current > 5000) {
              console.warn("Connection Watchdog Timeout: No data for 5s");
              bluetoothService.disconnect(); 
          }
      }, 1000);
      return () => clearInterval(watchdog);
  }, [isConnected]);

  const handleConnect = async () => {
    const success = await bluetoothService.connect();
    setIsConnected(success);
    if (success) {
        lastDataTimeRef.current = Date.now();
    }
    playConnectionSound(success);
    return success;
  };

  const handleStartGame = () => {
    setCurrentScreen('game');
  };

  const handleOpenSettings = () => {
    setCurrentScreen('settings');
  };

  const handleBackToGame = () => {
    setCurrentScreen('game');
  };

  const updateSettings = useCallback((newSettings: Partial<GameSettings>) => {
    setSettings(prev => ({ ...prev, ...newSettings }));
  }, []);

  const handleResetSettings = useCallback(() => {
    if (window.confirm("Are you sure you want to reset all settings to default?")) {
        setSettings(DEFAULT_SETTINGS);
    }
  }, []);

  return (
    <div className="relative w-full h-screen bg-slate-900 text-white font-sans overflow-hidden select-none">
      <NeonBackground />
      <BackgroundMusic />
      
      <main className="relative w-full h-full">
        {currentScreen === 'setup' && (
          <SetupScreen 
            onConnect={handleConnect} 
            onStartGame={handleStartGame} 
            onOpenSettings={handleOpenSettings}
            isConnected={isConnected}
          />
        )}
        
        {currentScreen === 'game' && (
          <GameScreen 
            sensorData={sensorData} 
            onOpenSettings={handleOpenSettings}
            sensitivity={settings.sensitivity}
            calibration={settings.calibration}
            threshold={settings.threshold}
          />
        )}

        {currentScreen === 'settings' && (
          <SettingsScreen 
            onBack={handleBackToGame} 
            sensorData={sensorData} 
            settings={settings}
            onUpdateSettings={updateSettings}
            onReset={handleResetSettings}
          />
        )}
      </main>
    </div>
  );
};

export default App;