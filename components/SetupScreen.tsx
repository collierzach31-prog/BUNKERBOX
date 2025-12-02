import React, { useState, useEffect, useRef } from 'react';
import { NeonButton } from './NeonButton';
import { Settings, Bluetooth, Loader2, PlayCircle, AlertCircle, Terminal, RefreshCw, Zap } from 'lucide-react';
import { bluetoothService } from '../services/bluetoothService';
import { LogEntry } from '../types';

interface SetupScreenProps {
  onConnect: () => Promise<boolean>;
  onStartGame: () => void;
  onOpenSettings: () => void;
  isConnected: boolean;
}

export const SetupScreen: React.FC<SetupScreenProps> = ({ 
  onConnect, 
  onStartGame, 
  onOpenSettings,
  isConnected 
}) => {
  const [status, setStatus] = useState<'idle' | 'connecting' | 'error' | 'calibrating' | 'ready'>('idle');
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const logContainerRef = useRef<HTMLDivElement>(null);
  const [hasSensors, setHasSensors] = useState(false);

  useEffect(() => {
    bluetoothService.onLog((entry) => {
      setLogs(prev => [...prev, entry]);
      if (entry.message.includes('SENSORS ACTIVE')) {
        setHasSensors(true);
        setStatus('ready');
      }
    });
  }, []);

  useEffect(() => {
    if (logContainerRef.current) {
        logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs]);

  const handleConnect = async () => {
    setStatus('connecting');
    setHasSensors(false);
    const success = await onConnect();
    if (success) {
      // Connection successful and sensors active
      setStatus('ready');
    } else {
      setStatus('error');
      setTimeout(() => setStatus('idle'), 3000);
    }
  };

  const handleRetrySensors = async () => {
    setStatus('calibrating');
    await bluetoothService.retryServiceConnection();
  };

  const handleDemoMode = () => {
    bluetoothService.startDemoMode();
    onStartGame();
  };

  return (
    <div className="relative z-10 flex flex-col items-center justify-between h-[100dvh] w-full p-4 md:p-8 text-center font-sans overflow-hidden">
      {/* Settings Gear */}
      <button 
        onClick={onOpenSettings}
        className="absolute top-4 right-4 md:top-6 md:right-6 bg-white border-2 md:border-4 border-black p-2 md:p-3 shadow-hard hover:shadow-none hover:translate-x-1 hover:translate-y-1 transition-all z-50 text-black"
      >
        <Settings size={24} className="md:w-7 md:h-7" />
      </button>

      {/* Header Logo */}
      <div className="mt-4 md:mt-8 flex flex-col items-center animate-pop-in shrink-0">
        <div className="bg-black text-white px-3 py-1 font-black uppercase tracking-widest text-xs md:text-sm mb-2 transform -rotate-2">
            Arcade Edition
        </div>
        <h1 className="text-6xl sm:text-7xl md:text-9xl font-black italic uppercase leading-none transform -skew-x-6 drop-shadow-lg">
            <span className="text-cartoon-cyan text-stroke-black" style={{ WebkitTextStroke: '2px black' }}>BUNKER</span><br/>
            <span className="text-cartoon-yellow text-stroke-black" style={{ WebkitTextStroke: '2px black' }}>BOX</span>
        </h1>
      </div>

      {/* Main Action Area */}
      <div className="flex-1 flex flex-col justify-center items-center w-full max-w-2xl py-4">
        
        {/* IDLE */}
        {status === 'idle' && (
            <div className="flex flex-col gap-6 md:gap-8 items-center animate-pop-in w-full">
            <NeonButton onClick={handleConnect} variant="cyan" pulse className="text-xl md:text-2xl px-8 py-6 md:px-12 md:py-8 flex items-center gap-3 md:gap-4 w-auto max-w-[90%] justify-center">
                <Bluetooth size={24} className="md:w-8 md:h-8" />
                SYNC MACHINE
            </NeonButton>
            
            <div className="bg-white border-2 border-black px-3 py-1.5 md:px-4 md:py-2 rounded-full shadow-hard text-[10px] md:text-xs font-black uppercase tracking-widest flex items-center gap-2">
                <div className="w-2 h-2 md:w-3 md:h-3 bg-cartoon-red rounded-full animate-ping"></div>
                Insert Coin / Connect
            </div>
            </div>
        )}

        {/* CONNECTING */}
        {status === 'connecting' && (
            <div className="flex flex-col items-center gap-4">
            <div className="border-4 border-black p-3 md:p-4 rounded-full bg-white shadow-hard animate-spin">
                <Loader2 size={32} className="text-black md:w-12 md:h-12" />
            </div>
            <span className="text-xl md:text-3xl font-black uppercase italic bg-cartoon-yellow px-4 py-1.5 md:px-6 md:py-2 border-4 border-black shadow-hard transform -rotate-2">
                Connecting...
            </span>
            </div>
        )}

        {/* CALIBRATING */}
        {status === 'calibrating' && (
            <div className="flex flex-col items-center gap-4 md:gap-6">
            {!hasSensors ? (
                <>
                    <Loader2 className="animate-spin w-12 h-12 md:w-16 md:h-16 text-cartoon-cyan" />
                    <div className="bg-white border-4 border-black p-4 md:p-6 shadow-hard text-center">
                        <span className="text-xl md:text-2xl font-black uppercase block mb-1">Warming Up</span>
                        <span className="text-xs md:text-sm font-bold text-gray-500">Calibrating Sensors...</span>
                    </div>
                    <div className="mt-2 md:mt-4 flex gap-4">
                         <button onClick={handleRetrySensors} className="bg-cartoon-yellow border-2 border-black px-4 py-2 shadow-hard hover:shadow-none hover:translate-y-1 font-bold uppercase text-xs flex items-center gap-2 transition-all">
                            <RefreshCw size={14} /> Retry
                         </button>
                    </div>
                </>
            ) : (
                <>
                   <Zap className="w-16 h-16 md:w-20 md:h-20 text-cartoon-yellow fill-current animate-bounce" />
                   <span className="text-2xl md:text-4xl font-black uppercase italic text-black">Ready to Rumble</span>
                </>
            )}
            </div>
        )}

        {/* ERROR */}
        {status === 'error' && (
            <div className="flex flex-col items-center gap-4 animate-shake">
            <AlertCircle size={48} className="text-cartoon-red md:w-16 md:h-16" />
            <div className="bg-cartoon-red text-white border-4 border-black px-6 py-3 md:px-8 md:py-4 shadow-hard text-lg md:text-xl font-black uppercase">
                Sync Failed!
            </div>
            </div>
        )}

        {/* READY */}
        {status === 'ready' && (
            <div className="animate-pop-in flex flex-col items-center gap-6 md:gap-8 w-full">
            <NeonButton onClick={onStartGame} variant="pink" className="text-3xl md:text-5xl px-10 py-6 md:px-16 md:py-8 transform hover:scale-105 w-auto max-w-[90%]">
                FIGHT!
            </NeonButton>
            <div className="bg-green-400 border-4 border-black px-4 py-1.5 md:px-6 md:py-2 shadow-hard flex items-center gap-2 font-black uppercase text-sm md:text-base">
                <Zap size={16} className="md:w-5 md:h-5" fill="black" />
                Gloves Active
            </div>
            </div>
        )}
      </div>

      {/* Footer Area */}
      <div className="w-full max-w-4xl flex flex-col gap-2 md:gap-4 shrink-0">

          <div className="flex justify-between items-center">
            <div className="bg-black text-white px-3 py-1.5 md:px-4 md:py-2 font-bold uppercase text-[10px] md:text-xs tracking-wider border-2 border-black shadow-hard transform skew-x-[-10deg]">
               STATUS: {isConnected ? <span className="text-green-400">ONLINE</span> : <span className="text-red-500">OFFLINE</span>}
            </div>

            {status === 'idle' && (
                <button onClick={handleDemoMode} className="bg-white hover:bg-gray-100 text-black border-2 border-black px-3 py-1.5 md:px-4 md:py-2 shadow-hard text-[10px] md:text-xs font-black uppercase flex items-center gap-2 transition-all active:shadow-none active:translate-y-1">
                    <PlayCircle size={12} className="md:w-3.5 md:h-3.5" />
                    Demo Mode
                </button>
            )}
          </div>
      </div>
    </div>
  );
};