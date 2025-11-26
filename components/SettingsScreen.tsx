import React, { useState, useEffect, useRef } from 'react';
import { ArrowLeft, Save, RotateCcw } from 'lucide-react';
import { NeonButton } from './NeonButton';
import { SensorData, GameSettings } from '../types';

interface SettingsScreenProps {
  onBack: () => void;
  sensorData: SensorData;
  settings: GameSettings;
  onUpdateSettings: (newSettings: Partial<GameSettings>) => void;
  onReset: () => void;
}

export const SettingsScreen: React.FC<SettingsScreenProps> = ({ 
  onBack, 
  sensorData, 
  settings, 
  onUpdateSettings,
  onReset
}) => {
  const [showCode, setShowCode] = useState(false);
  const [graphData, setGraphData] = useState<number[]>(new Array(50).fill(0));
  
  // Calibration Testing Logic
  const [punchHistory, setPunchHistory] = useState<Array<{
      id: number;
      rawPeak: number;
      calibratedPeak: number;
      score: number;
      time: string;
  }>>([]);
  
  const isCollectingRef = useRef<boolean>(false);
  const currentPeakRef = useRef<number>(0);
  const collectionTimeoutRef = useRef<number | null>(null);
  const lastPunchTime = useRef<number>(0);

  // Cleanup timeout on unmount only
  useEffect(() => {
      return () => {
          if (collectionTimeoutRef.current) clearTimeout(collectionTimeoutRef.current);
      };
  }, []);

  useEffect(() => {
    // Graph Update
    setGraphData(prev => {
      const val = Math.max(0, sensorData.mag - settings.calibration);
      return [...prev.slice(1), val];
    });

    // Peak Detection for Calibration Table
    const now = Date.now();
    const rawForce = Math.max(0, sensorData.mag - settings.calibration);
    const MAX_FORCE = 80;

    if (isCollectingRef.current) {
        if (rawForce > currentPeakRef.current) {
            currentPeakRef.current = rawForce;
        }
    } else if (rawForce > settings.threshold && now - lastPunchTime.current > 1000) {
        isCollectingRef.current = true;
        currentPeakRef.current = rawForce;
        
        collectionTimeoutRef.current = window.setTimeout(() => {
            const peak = currentPeakRef.current;
            
            // Calculate Score
            // Dynamic Max Force: Sens 100 = 20g, Sens 1 = 160g (Sensor Max)
            const dynamicMaxForce = 20 + (100 - settings.sensitivity) * 1.4;
            
            const normalized = Math.min(peak / dynamicMaxForce, 1);
            const curved = Math.pow(normalized, 1.4);
            
            let score = Math.floor(curved * 9999);
            score = Math.min(score, 9999);
            
            const newRecord = {
                id: Date.now(),
                rawPeak: peak + settings.calibration, // Approx raw
                calibratedPeak: peak,
                score: score,
                time: new Date().toLocaleTimeString()
            };
            
            setPunchHistory(prev => [newRecord, ...prev].slice(0, 5)); // Keep last 5
            lastPunchTime.current = Date.now();
            isCollectingRef.current = false;
        }, 300);
    }
  }, [sensorData, settings.calibration, settings.threshold, settings.sensitivity]);
  
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
        </div>

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

              {/* Audio Panel */}
              <div className="bg-white p-6 border-4 border-black shadow-hard transform -rotate-1">
                <label className="block bg-black text-white px-2 py-1 w-max font-black uppercase text-lg mb-4 transform rotate-1">
                  Audio Mixer
                </label>
                
                <div className="space-y-4">
                    <div>
                        <div className="flex justify-between text-xs font-bold mb-1 uppercase">
                            <span>SFX Volume</span>
                            <span>{settings.sfxVolume ?? 50}%</span>
                        </div>
                        <input 
                            type="range" 
                            min="0" 
                            max="100" 
                            value={settings.sfxVolume ?? 50} 
                            onChange={(e) => onUpdateSettings({ sfxVolume: parseInt(e.target.value, 10) })}
                            className="w-full h-4 bg-gray-200 appearance-none border-2 border-black accent-cartoon-red"
                        />
                    </div>
                    <div>
                        <div className="flex justify-between text-xs font-bold mb-1 uppercase">
                            <span>Music Volume</span>
                            <span>{settings.musicVolume ?? 50}%</span>
                        </div>
                        <input 
                            type="range" 
                            min="0" 
                            max="100" 
                            value={settings.musicVolume ?? 50} 
                            onChange={(e) => onUpdateSettings({ musicVolume: parseInt(e.target.value, 10) })}
                            className="w-full h-4 bg-gray-200 appearance-none border-2 border-black accent-cartoon-purple"
                        />
                    </div>
                </div>
              </div>

              {/* Calibration Panel */}
              <div className="bg-white p-6 border-4 border-black shadow-hard transform -rotate-1">
                <label className="block bg-black text-white px-2 py-1 w-max font-black uppercase text-lg mb-4 transform rotate-1">
                  Zero Calibration
                </label>
                <div className="flex justify-between text-xs font-bold mb-2">
                    <span>Raw: {sensorData.mag.toFixed(1)}</span>
                    <span className="text-cartoon-purple">Calibrated: {Math.max(0, sensorData.mag - settings.calibration).toFixed(1)}</span>
                </div>
                <div className="flex items-center gap-4">
                  <input 
                    type="range" 
                    min="0" 
                    max="200" 
                    step="0.1"
                    value={settings.calibration} 
                    onChange={(e) => onUpdateSettings({ calibration: parseFloat(e.target.value) })}
                    className="w-full h-4 bg-gray-200 appearance-none border-2 border-black accent-cartoon-cyan"
                  />
                  <span className="text-xl font-black w-16 text-right">{settings.calibration.toFixed(1)}</span>
                </div>
                <button 
                    onClick={() => onUpdateSettings({ calibration: sensorData.mag })}
                    className="mt-4 w-full bg-cartoon-pink border-2 border-black py-2 font-bold uppercase hover:bg-pink-400"
                >
                    Set to Current (Zero)
                </button>
              </div>

              {/* Threshold Panel */}
              <div className="bg-white p-6 border-4 border-black shadow-hard transform rotate-1">
                <label className="block bg-black text-white px-2 py-1 w-max font-black uppercase text-lg mb-4 transform -rotate-1">
                  Trigger Threshold
                </label>
                <p className="text-xs font-bold mb-2">Minimum force spike to detect a punch</p>
                <div className="flex items-center gap-4">
                  <input 
                    type="range" 
                    min="5" 
                    max="100" 
                    value={settings.threshold} 
                    onChange={(e) => onUpdateSettings({ threshold: parseInt(e.target.value, 10) })}
                    className="w-full h-4 bg-gray-200 appearance-none border-2 border-black accent-cartoon-purple"
                  />
                  <span className="text-3xl font-black">{settings.threshold}</span>
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
                  <DataCard label="Raw X" value={sensorData.x} color="text-cartoon-red" />
                  <DataCard label="Raw Y" value={sensorData.y} color="text-cartoon-cyan" />
                  <DataCard label="Raw Z" value={sensorData.z} color="text-cartoon-purple" />
                  <DataCard 
                    label="Calibrated Force" 
                    value={Math.max(0, sensorData.mag - settings.calibration)} 
                    color="text-black" 
                    isLarge 
                  />
               </div>
               <div className="text-center text-xs font-bold mt-2">
                   * Game uses Calibrated Force
               </div>

               {/* Live Graph */}
               <div className="bg-white border-4 border-black h-40 w-full relative overflow-hidden mt-4 shadow-hard">
                  <div className="absolute top-0 left-0 text-[10px] font-black p-1 bg-black text-white z-10">FORCE GRAPH</div>
                  <svg className="w-full h-full" viewBox="0 0 100 100" preserveAspectRatio="none">
                    {/* Grid Lines */}
                    <line x1="0" y1="25" x2="100" y2="25" stroke="#eee" strokeWidth="0.5" />
                    <line x1="0" y1="50" x2="100" y2="50" stroke="#eee" strokeWidth="0.5" />
                    <line x1="0" y1="75" x2="100" y2="75" stroke="#eee" strokeWidth="0.5" />

                    {/* Threshold Line */}
                    <line 
                        x1="0" y1={100 - Math.min(settings.threshold, 100)} 
                        x2="100" y2={100 - Math.min(settings.threshold, 100)} 
                        stroke="red" 
                        strokeWidth="1" 
                        strokeDasharray="2" 
                    />
                    <text x="2" y={100 - Math.min(settings.threshold, 100) - 2} fill="red" fontSize="4" fontWeight="bold">THRESHOLD</text>

                    {/* Data Line */}
                    <polyline
                        fill="none"
                        stroke="black"
                        strokeWidth="1.5"
                        points={graphData.map((val, i) => {
                            const x = (i / (graphData.length - 1)) * 100;
                            const y = 100 - Math.min((val / 100) * 100, 100); 
                            return `${x},${y}`;
                        }).join(' ')}
                    />
                  </svg>
               </div>

               {/* Test Punch History Table */}
               <div className="mt-4 bg-white border-4 border-black p-4 shadow-hard">
                   <h4 className="font-black uppercase text-sm mb-2 border-b-2 border-black flex justify-between items-center">
                       <span>Test Punches</span>
                       <span className="text-[10px] text-gray-500 font-normal">Last 5 Hits</span>
                   </h4>
                   {punchHistory.length === 0 ? (
                       <div className="text-center text-xs text-gray-400 py-4 italic">
                           Punch the bag to see calibration data...
                       </div>
                   ) : (
                       <table className="w-full text-xs font-bold text-center">
                           <thead>
                               <tr className="text-[10px] uppercase text-gray-500 border-b border-gray-200">
                                   <th className="pb-1">Time</th>
                                   <th className="pb-1">Raw</th>
                                   <th className="pb-1">Calib</th>
                                   <th className="pb-1 text-cartoon-purple">Score</th>
                               </tr>
                           </thead>
                           <tbody>
                               {punchHistory.map((punch) => (
                                   <tr key={punch.id} className="border-b border-gray-100 last:border-0">
                                       <td className="py-2 text-gray-400 font-mono">{punch.time}</td>
                                       <td className="py-2 text-gray-600">{punch.rawPeak.toFixed(1)}</td>
                                       <td className="py-2">{punch.calibratedPeak.toFixed(1)}</td>
                                       <td className="py-2 text-cartoon-purple text-sm">{punch.score}</td>
                                   </tr>
                               ))}
                           </tbody>
                       </table>
                   )}
               </div>

               {/* Instructions Box */}
               <div className="mt-4 bg-yellow-50 border-4 border-black p-4 shadow-hard transform rotate-1">
                   <h4 className="font-black uppercase text-sm mb-4 border-b-2 border-black text-cartoon-purple">
                       Settings Guide
                   </h4>
                   
                   {/* Slider Definitions */}
                   <div className="mb-4 space-y-3">
                       <div>
                           <span className="text-xs font-black uppercase bg-cartoon-yellow px-1 border border-black shadow-sm">Punch Gain</span>
                           <p className="text-[10px] leading-tight mt-1 font-medium text-gray-600">
                               Controls score difficulty. <br/>
                               <span className="text-black">Higher</span> = Easier to get 9999. <span className="text-black">Lower</span> = Harder.
                           </p>
                       </div>
                       <div>
                           <span className="text-xs font-black uppercase bg-cartoon-cyan px-1 border border-black shadow-sm">Zero Calibration</span>
                           <p className="text-[10px] leading-tight mt-1 font-medium text-gray-600">
                               Removes gravity and sensor offset. <br/>
                               Adjust until <span className="text-black">Calibrated Force</span> reads <span className="font-mono">0.0</span> when the bag is still.
                           </p>
                       </div>
                       <div>
                           <span className="text-xs font-black uppercase bg-cartoon-purple text-white px-1 border border-black shadow-sm">Trigger Threshold</span>
                           <p className="text-[10px] leading-tight mt-1 font-medium text-gray-600">
                               Sensitivity to start a punch. <br/>
                               <span className="text-black">Raise</span> if you get "ghost hits". <span className="text-black">Lower</span> for light taps.
                           </p>
                       </div>
                   </div>

                   <h5 className="font-black uppercase text-xs mb-2 border-b border-gray-300">Quick Setup Steps</h5>
                   <ul className="text-[10px] font-bold space-y-1 text-gray-700 list-decimal pl-4">
                       <li>Keep bag completely still.</li>
                       <li>Click <span className="bg-cartoon-pink px-1 border border-black text-[8px] uppercase text-black">Set to Current</span> to auto-zero.</li>
                       <li>Check graph: The line should be flat and below the red Threshold line.</li>
                       <li>Punch! Adjust Gain to tune difficulty.</li>
                   </ul>
               </div>
            </div>
        </div>

        {/* Footer */}
        <div className="mt-8 pt-8 flex flex-col gap-4 items-center border-t-4 border-black">
          <NeonButton onClick={onReset} variant="pink" className="w-full max-w-md">
            <div className="flex items-center justify-center gap-2">
              <RotateCcw size={20} />
              Reset to Defaults
            </div>
          </NeonButton>

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