import React, { useState, useEffect, useCallback } from 'react';
import { NeonBackground } from './components/NeonBackground';
import { SetupScreen } from './components/SetupScreen';
import { GameScreen } from './components/GameScreen';
import { SettingsScreen } from './components/SettingsScreen';
import { bluetoothService } from './services/bluetoothService';
import { ScreenState, SensorData, GameSettings } from './types';

// Default to standard HM-10 / generic BLE module UUIDs as requested (0xFFE0 / 0xFFE1)
const DEFAULT_SETTINGS: GameSettings = {
  sensitivity: 50,
  threshold: 15,
  serviceUUID: '0000ffe0-0000-1000-8000-00805f9b34fb',
  characteristicUUID: '0000ffe1-0000-1000-8000-00805f9b34fb'
};

const App: React.FC = () => {
  const [currentScreen, setCurrentScreen] = useState<ScreenState>('setup');
  const [isConnected, setIsConnected] = useState(false);
  const [sensorData, setSensorData] = useState<SensorData>({ x: 0, y: 0, z: 0, mag: 0 });
  
  // Load settings from localStorage or use defaults
  const [settings, setSettings] = useState<GameSettings>(() => {
    try {
      const saved = localStorage.getItem('bunker_settings');
      return saved ? { ...DEFAULT_SETTINGS, ...JSON.parse(saved) } : DEFAULT_SETTINGS;
    } catch (e) {
      return DEFAULT_SETTINGS;
    }
  });

  // Update Bluetooth Service when settings change
  useEffect(() => {
    bluetoothService.setUUIDs(settings.serviceUUID, settings.characteristicUUID);
    localStorage.setItem('bunker_settings', JSON.stringify(settings));
  }, [settings]);

  // Setup Bluetooth Listener
  useEffect(() => {
    bluetoothService.onDataReceived((data) => {
      setSensorData(data);
    });
    
    return () => {
      bluetoothService.disconnect();
    };
  }, []);

  const handleConnect = async () => {
    const success = await bluetoothService.connect();
    setIsConnected(success);
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

  return (
    <div className="relative w-full h-screen bg-slate-900 text-white font-sans overflow-hidden select-none">
      <NeonBackground />
      
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
          />
        )}

        {currentScreen === 'settings' && (
          <SettingsScreen 
            onBack={handleBackToGame} 
            sensorData={sensorData} 
            settings={settings}
            onUpdateSettings={updateSettings}
          />
        )}
      </main>
    </div>
  );
};

export default App;