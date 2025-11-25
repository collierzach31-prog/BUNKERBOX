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
      setStatus('calibrating'); 
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
    <div className="relative z-10 flex flex-col items-center justify-between h-screen w-full p-8 text-center font-sans">
      {/* Settings Gear */}
      <button 
        onClick={onOpenSettings}
        className="absolute top-6 right-6 bg-white border-4 border-black p-3 shadow-hard hover:shadow-none hover:translate-x-1 hover:translate-y-1 transition-all z-50 text-black"
      >
        <Settings size={28} />
      </button>

      {/* Header Logo */}
      <div className="mt-8 flex flex-col items-center animate-pop-in">
        <div className="bg-black text-white px-4 py-1 font-black uppercase tracking-widest text-sm mb-2 transform -rotate-2">
            Arcade Edition
        </div>
        <h1 className="text-8xl md:text-9xl font-black italic uppercase leading-none transform -skew-x-6 drop-shadow-lg">
            <span className="text-cartoon-cyan text-stroke-black" style={{ WebkitTextStroke: '3px black' }}>BUNKER</span><br/>
            <span className="text-cartoon-yellow text-stroke-black" style={{ WebkitTextStroke: '3px black' }}>BOX</span>
        </h1>
      </div>

      {/* Main Action Area */}
      <div className="flex-1 flex flex-col justify-center items-center w-full max-w-2xl">
        
        {/* IDLE */}
        {status === 'idle' && (
            <div className="flex flex-col gap-8 items-center animate-pop-in">
            <NeonButton onClick={handleConnect} variant="cyan" pulse className="text-2xl px-12 py-8 flex items-center gap-4">
                <Bluetooth size={32} />
                SYNC MACHINE
            </NeonButton>
            
            <div className="bg-white border-2 border-black px-4 py-2 rounded-full shadow-hard text-xs font-black uppercase tracking-widest flex items-center gap-2">
                <div className="w-3 h-3 bg-cartoon-red rounded-full animate-ping"></div>
                Insert Coin / Connect
            </div>
            </div>
        )}

        {/* CONNECTING */}
        {status === 'connecting' && (
            <div className="flex flex-col items-center gap-4">
            <div className="border-4 border-black p-4 rounded-full bg-white shadow-hard animate-spin">
                <Loader2 size={48} className="text-black" />
            </div>
            <span className="text-3xl font-black uppercase italic bg-cartoon-yellow px-6 py-2 border-4 border-black shadow-hard transform -rotate-2">
                Connecting...
            </span>
            </div>
        )}

        {/* CALIBRATING */}
        {status === 'calibrating' && (
            <div className="flex flex-col items-center gap-6">
            {!hasSensors ? (
                <>
                    <Loader2 className="animate-spin w-16 h-16 text-cartoon-cyan" />
                    <div className="bg-white border-4 border-black p-6 shadow-hard text-center">
                        <span className="text-2xl font-black uppercase block mb-1">Warming Up</span>
                        <span className="text-sm font-bold text-gray-500">Calibrating Sensors...</span>
                    </div>
                    <div className="mt-4 flex gap-4">
                         <button onClick={handleRetrySensors} className="bg-cartoon-yellow border-2 border-black px-4 py-2 shadow-hard hover:shadow-none hover:translate-y-1 font-bold uppercase text-xs flex items-center gap-2 transition-all">
                            <RefreshCw size={14} /> Retry
                         </button>
                    </div>
                </>
            ) : (
                <>
                   <Zap className="w-20 h-20 text-cartoon-yellow fill-current animate-bounce" />
                   <span className="text-4xl font-black uppercase italic text-black">Ready to Rumble</span>
                </>
            )}
            </div>
        )}

        {/* ERROR */}
        {status === 'error' && (
            <div className="flex flex-col items-center gap-4 animate-shake">
            <AlertCircle size={64} className="text-cartoon-red" />
            <div className="bg-cartoon-red text-white border-4 border-black px-8 py-4 shadow-hard text-xl font-black uppercase">
                Sync Failed!
            </div>
            </div>
        )}

        {/* READY */}
        {status === 'ready' && (
            <div className="animate-pop-in flex flex-col items-center gap-8">
            <NeonButton onClick={onStartGame} variant="pink" className="text-5xl px-16 py-8 transform hover:scale-105">
                FIGHT!
            </NeonButton>
            <div className="bg-green-400 border-4 border-black px-6 py-2 shadow-hard flex items-center gap-2 font-black uppercase">
                <Zap size={20} fill="black" />
                Gloves Active
            </div>
            </div>
        )}
      </div>

      {/* Footer Area */}
      <div className="w-full max-w-4xl flex flex-col gap-4">
          
          {/* Comic Style Log Box */}
          <div className="w-full bg-white border-4 border-black shadow-hard flex flex-col">
             <div className="bg-cartoon-yellow border-b-4 border-black px-4 py-2 flex items-center justify-between">
                 <div className="flex items-center gap-2 text-xs font-black uppercase">
                    <Terminal size={14} />
                    <span>Arena Log</span>
                 </div>
                 {isConnected && <div className="w-3 h-3 bg-green-500 border-2 border-black rounded-full animate-pulse"></div>}
             </div>
             <div 
                ref={logContainerRef}
                className="h-24 overflow-y-auto p-4 font-mono text-xs space-y-1 bg-white text-left"
             >
                {logs.length === 0 && <span className="text-gray-400 italic font-bold">Waiting for input...</span>}
                {logs.map((log, i) => (
                    <div key={i} className={`font-bold ${
                        log.type === 'error' ? 'text-red-600' : 
                        log.type === 'success' ? 'text-green-600' : 
                        log.type === 'warning' ? 'text-orange-600' : 'text-gray-600'
                    }`}>
                        <span className="mr-2 opacity-50">[{new Date(log.timestamp).toLocaleTimeString().split(' ')[0]}]</span>
                        {log.message}
                    </div>
                ))}
             </div>
          </div>

          <div className="flex justify-between items-center">
            <div className="bg-black text-white px-4 py-2 font-bold uppercase text-xs tracking-wider border-2 border-black shadow-hard transform skew-x-[-10deg]">
               STATUS: {isConnected ? <span className="text-green-400">ONLINE</span> : <span className="text-red-500">OFFLINE</span>}
            </div>

            {status === 'idle' && (
                <button onClick={handleDemoMode} className="bg-white hover:bg-gray-100 text-black border-2 border-black px-4 py-2 shadow-hard text-xs font-black uppercase flex items-center gap-2 transition-all active:shadow-none active:translate-y-1">
                    <PlayCircle size={14} />
                    Demo Mode
                </button>
            )}
          </div>
      </div>
    </div>
  );
};