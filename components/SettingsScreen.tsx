import React, { useState } from 'react';
import { ArrowLeft, Copy, Code, Check } from 'lucide-react';
import { NeonButton } from './NeonButton';
import { SensorData, GameSettings } from '../types';

interface SettingsScreenProps {
  onBack: () => void;
  sensorData: SensorData;
  settings: GameSettings;
  onUpdateSettings: (newSettings: Partial<GameSettings>) => void;
}

export const SettingsScreen: React.FC<SettingsScreenProps> = ({ 
  onBack, 
  sensorData, 
  settings, 
  onUpdateSettings 
}) => {
  const [showCode, setShowCode] = useState(false);
  const [copied, setCopied] = useState(false);
  
  const handleSensitivityChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    onUpdateSettings({ sensitivity: parseInt(e.target.value, 10) });
  };

  const handleUUIDChange = (key: 'serviceUUID' | 'characteristicUUID', value: string) => {
    onUpdateSettings({ [key]: value.trim() });
  };

  return (
    <div className="relative z-10 flex flex-col h-screen w-full p-8 bg-white overflow-y-auto text-black font-sans">
      <div className="max-w-4xl mx-auto w-full flex-1 flex flex-col">
        
        {/* Comic Header */}
        <div className="flex items-center justify-between mb-8 border-b-4 border-black pb-4">
          <h1 className="text-4xl font-black uppercase italic tracking-tighter drop-shadow-md">
            Machine Setup
          </h1>
          <button onClick={() => setShowCode(!showCode)} className="bg-cartoon-cyan border-2 border-black px-4 py-2 font-bold uppercase shadow-hard hover:translate-y-1 hover:shadow-none transition-all flex gap-2">
            <Code size={20} />
            <span>Firmware</span>
          </button>
        </div>

        {/* Code Block Panel */}
        {showCode && (
          <div className="mb-8 border-4 border-black bg-gray-900 p-4 shadow-hard text-green-400 font-mono text-xs overflow-auto h-64 relative">
             <div className="absolute top-4 right-4">
                <button onClick={() => { navigator.clipboard.writeText("See previous turn for full code"); setCopied(true); setTimeout(() => setCopied(false), 2000); }} className="bg-white text-black px-2 py-1 font-bold uppercase text-xs">
                    {copied ? 'Copied!' : 'Copy'}
                </button>
             </div>
             <p>// ESP32 Code Hidden for brevity in this view...</p>
          </div>
        )}

        <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
            
            {/* Left Col: Inputs */}
            <div className="space-y-8">
              {/* Sensitivity Panel */}
              <div className="bg-white p-6 border-4 border-black shadow-hard transform rotate-1">
                <label className="block bg-black text-white px-2 py-1 w-max font-black uppercase text-lg mb-4 transform -rotate-1">
                  Punch Gain
                </label>
                <div className="flex items-center gap-4">
                  <input 
                    type="range" 
                    min="1" 
                    max="100" 
                    value={settings.sensitivity} 
                    onChange={handleSensitivityChange}
                    className="w-full h-4 bg-gray-200 appearance-none border-2 border-black accent-cartoon-yellow"
                  />
                  <span className="text-3xl font-black">{settings.sensitivity}</span>
                </div>
              </div>

              {/* UUID Panel */}
              <div className="bg-white p-6 border-4 border-black shadow-hard transform -rotate-1">
                <h3 className="font-black uppercase text-xl mb-4 border-b-2 border-black inline-block">Connection ID</h3>
                
                <div className="space-y-4">
                    <div>
                        <label className="block font-bold text-xs uppercase mb-1">Service UUID</label>
                        <input 
                            type="text" 
                            value={settings.serviceUUID}
                            onChange={(e) => handleUUIDChange('serviceUUID', e.target.value)}
                            className="w-full border-2 border-black p-2 font-mono text-sm focus:bg-yellow-50 focus:outline-none"
                        />
                    </div>
                    <div>
                        <label className="block font-bold text-xs uppercase mb-1">Char UUID</label>
                        <input 
                            type="text" 
                            value={settings.characteristicUUID}
                            onChange={(e) => handleUUIDChange('characteristicUUID', e.target.value)}
                            className="w-full border-2 border-black p-2 font-mono text-sm focus:bg-yellow-50 focus:outline-none"
                        />
                    </div>
                </div>
              </div>
            </div>

            {/* Right Col: Live Data */}
            <div className="bg-cartoon-bg border-4 border-black p-6 shadow-hard flex flex-col gap-4">
               <div className="bg-black text-white px-4 py-2 font-black uppercase text-center transform -skew-x-6 mb-4">
                   Live Telemetry
               </div>
               
               <div className="grid grid-cols-2 gap-4">
                  <DataCard label="X-Axis" value={sensorData.x} color="text-cartoon-red" />
                  <DataCard label="Y-Axis" value={sensorData.y} color="text-cartoon-cyan" />
                  <DataCard label="Z-Axis" value={sensorData.z} color="text-cartoon-purple" />
                  <DataCard label="Force" value={sensorData.mag} color="text-black" isLarge />
               </div>
            </div>
        </div>

        {/* Footer */}
        <div className="mt-8 pt-8 flex justify-center border-t-4 border-black">
          <NeonButton onClick={onBack} variant="yellow" className="w-full max-w-md">
            <div className="flex items-center justify-center gap-2">
              <ArrowLeft />
              Back to Ring
            </div>
          </NeonButton>
        </div>
      </div>
    </div>
  );
};

const DataCard: React.FC<{ label: string; value: number; color: string; isLarge?: boolean }> = ({ 
    label, value, color, isLarge 
}) => (
    <div className="bg-white border-2 border-black p-2 shadow-hard">
        <div className="text-xs font-black uppercase text-gray-400">{label}</div>
        <div className={`${color} ${isLarge ? 'text-4xl' : 'text-xl'} font-black tabular-nums`}>
            {value.toFixed(1)}
        </div>
    </div>
);