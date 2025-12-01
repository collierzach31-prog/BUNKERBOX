import React, { useEffect, useState, useRef } from 'react';
import { Settings, Zap, Crown, Flame, Volume2, VolumeX } from 'lucide-react';
import { SensorData } from '../types';
import { playArcadeSound, setVolumes } from '../services/audioService';

interface GameScreenProps {
  sensorData: SensorData;
  onOpenSettings: () => void;
  sensitivity: number;
  calibration: number;
  threshold: number;
  sfxVolume: number;
  musicVolume: number;
}

// --------------------

export const GameScreen: React.FC<GameScreenProps> = ({ 
  sensorData, 
  onOpenSettings, 
  sensitivity,
  calibration,
  threshold,
  sfxVolume,
  musicVolume
}) => {
  const [targetScore, setTargetScore] = useState<number>(0);
  const [displayScore, setDisplayScore] = useState<number>(0);
  const [highScore, setHighScore] = useState<number>(0);
  const [isMuted, setIsMuted] = useState<boolean>(false);
  
  // Gauge States
  const [visualGauge, setVisualGauge] = useState(0); 
  const [isGaugeFrozen, setIsGaugeFrozen] = useState(false);
  const [isImpact, setIsImpact] = useState(false);
  const [isNewRecord, setIsNewRecord] = useState(false);
  const [history, setHistory] = useState<number[]>([]);
  
  const lastPunchTime = useRef<number>(0);
  const scoreanimRef = useRef<number>(0);
  const startDelayRef = useRef<number | null>(null);
  const snapTimeoutRef = useRef<number | null>(null);
  const lastAudioTickRef = useRef<number>(0);
  const accumulatedScoreDeltaRef = useRef<number>(0);
  const lastFrameScoreRef = useRef<number>(0);
  
  const MAX_FORCE = 80; 
  
  // Peak Detection Refs
  const isCollectingRef = useRef<boolean>(false);
  const currentPeakRef = useRef<number>(0);
  const collectionTimeoutRef = useRef<number | null>(null);
  
  // Mute Toggle Handler
  const handleMuteToggle = () => {
    setIsMuted(prev => {
      const newMuted = !prev;
      if (newMuted) {
        setVolumes(0, 0);
      } else {
        setVolumes(sfxVolume, musicVolume);
      }
      return newMuted;
    });
  };
  
  // Cleanup
  useEffect(() => {
     return () => {
         if (collectionTimeoutRef.current) clearTimeout(collectionTimeoutRef.current);
         if (snapTimeoutRef.current) clearTimeout(snapTimeoutRef.current);
         if (startDelayRef.current) clearTimeout(startDelayRef.current);
         if (scoreanimRef.current) cancelAnimationFrame(scoreanimRef.current);
     };
  }, []);

  useEffect(() => {
    const now = Date.now();
    // Use user-defined calibration instead of hardcoded GRAVITY
    const rawForce = Math.max(0, sensorData.mag - calibration);
    
    // Dynamic Difficulty Range: 
    // Sensitivity 100 (Easy) -> 20 units (~2g)
    // Sensitivity 1 (Hard) -> 160 units (~16g, Sensor Max)
    const dynamicMaxForce = 20 + (100 - sensitivity) * 1.4;

    const currentForcePercent = Math.min((rawForce / dynamicMaxForce) * 100, 100);

    // If frozen, do NOT update visualGauge
    if (!isGaugeFrozen) {
        setVisualGauge(prev => {
            // If collecting peak, hold the value (don't decay)
            if (isCollectingRef.current) {
                return Math.max(prev, currentForcePercent);
            }

            if (currentForcePercent > prev) return currentForcePercent; 
            return Math.max(0, prev - 2.5); // Fast decay for snappy feel
        });
    }

    // PEAK DETECTION LOGIC
    if (isCollectingRef.current) {
        // We are currently in a "punch window". Track the max force.
        if (rawForce > currentPeakRef.current) {
            currentPeakRef.current = rawForce;
        }
    } else if (!isGaugeFrozen && rawForce > threshold && now - lastPunchTime.current > 1000) {
        // START LISTENING
        isCollectingRef.current = true;
        currentPeakRef.current = rawForce;
        
        // Wait 300ms to find the true peak
        collectionTimeoutRef.current = window.setTimeout(() => {
            const peak = currentPeakRef.current;
            
            // Dynamic Difficulty: Sensitivity determines the "Max Force" needed for 9999
            const dynamicMaxForce = 20 + (100 - sensitivity) * 1.4;

            const normalizedForce = Math.min(peak / dynamicMaxForce, 1);
            const curvedForce = Math.pow(normalizedForce, 1.4); 
            
            let calculatedScore = Math.floor(curvedForce * 9999);
            if (calculatedScore > 9999) calculatedScore = 9999;
            if (calculatedScore < 100) calculatedScore = 100 + Math.floor(Math.random() * 50);

            // Freeze gauge at the PEAK level (Visual only, relative to dynamic max)
            const hitPeakPercent = Math.min((peak / dynamicMaxForce) * 100, 100);
            setVisualGauge(hitPeakPercent); 
            setIsGaugeFrozen(true);

            triggerPunch(calculatedScore, normalizedForce);
            lastPunchTime.current = Date.now();
            
            // Reset
            isCollectingRef.current = false;
            collectionTimeoutRef.current = null;
        }, 300);
    }
  }, [sensorData, sensitivity, isGaugeFrozen, calibration]);

  const triggerPunch = (finalScore: number, intensity: number) => {
    setTargetScore(finalScore);
    setIsImpact(true);
    
    // ALWAYS PLAY MAX IMPACT SOUND
    try {
        playArcadeSound('impact', 1.0); 
    } catch (e) {
        console.warn("Audio impact failed", e);
    }
    
    setTimeout(() => setIsImpact(false), 200);

    if (scoreanimRef.current) cancelAnimationFrame(scoreanimRef.current);
    if (startDelayRef.current) clearTimeout(startDelayRef.current);
    if (snapTimeoutRef.current) clearTimeout(snapTimeoutRef.current);

    // Delay before rolling score
    startDelayRef.current = window.setTimeout(() => {
        const startTime = performance.now();
        
        const startScore = 0; 
        const duration = 3500; // Longer duration for dramatic effect
        
        // Audio Sync: Throttle ticks to prevent buzzing, but ensure 1:1 sync at low speeds
        let lastTickTime = 0;
        let lastAudioScore = 0;
        const MIN_TICK_INTERVAL = 40; // ~25Hz max rate

        const animateScore = (time: number) => {
            const elapsed = time - startTime;
            const progress = Math.min(elapsed / duration, 1);
            
            // Quartic Out Easing (Smoother deceleration from the middle)
            const ease = 1 - Math.pow(1 - progress, 4);
            
            let current = Math.floor(startScore + (finalScore - startScore) * ease);
            let jitter = 0;
            if (progress < 0.5) jitter = Math.floor(Math.random() * (finalScore * 0.02));

            setDisplayScore(current + jitter);

            // Sync audio: Tick if value changed AND (enough time passed OR we are moving slowly)
            // This ensures a steady rattle at high speeds, and perfect 1:1 sync as it slows down
            if (current !== lastAudioScore) {
                const timeSinceLastTick = elapsed - lastTickTime;
                
                // If we are slow enough (changes taking > 40ms), always tick.
                // If we are fast, only tick every 40ms.
                if (timeSinceLastTick >= MIN_TICK_INTERVAL) {
                    try {
                        playArcadeSound('tick');
                    } catch (e) {
                        // Ignore audio errors to prevent animation crash
                    }
                    lastTickTime = elapsed;
                    lastAudioScore = current;
                }
            }

            if (progress < 1) {
                scoreanimRef.current = requestAnimationFrame(animateScore);
            } else {
                setDisplayScore(finalScore);
                
                let resetDelay = 200;

                // Score Locked: Update records
                if (finalScore > highScore) {
                    setHighScore(finalScore);
                    setIsNewRecord(true);
                    
                    // Safely play audio
                    try {
                        playArcadeSound('newRecord');
                    } catch (e) {
                        console.error("Audio playback failed", e);
                    }

                    setTimeout(() => setIsNewRecord(false), 3000);
                    resetDelay = 3200; // Keep gauge frozen during celebration
                } else {
                    try {
                        playArcadeSound('win');
                    } catch (e) {
                        console.error("Audio playback failed", e);
                    }
                }
                
                setHistory(prev => [finalScore, ...prev].slice(0, 5));

                // Snap Gauge Back
                snapTimeoutRef.current = window.setTimeout(() => {
                    setIsGaugeFrozen(false);
                    setVisualGauge(0);
                }, resetDelay); 
            }
        };
        scoreanimRef.current = requestAnimationFrame(animateScore);
    }, 250);
  };
  
  // --- CARTOON GAUGE RENDERER ---
  const renderGaugeSegments = () => {
    const SEGMENTS = 24;
    return [...Array(SEGMENTS)].map((_, i) => {
      const threshold = (i / SEGMENTS) * 100;
      const isActive = visualGauge >= threshold;
      
      // Neon Gradient Logic
      let colorClass = 'bg-gray-800 border-gray-700';
      if (isActive) {
          if (i < 4) colorClass = 'bg-cartoon-cyan shadow-[0_0_10px_#22d3ee] border-black';
          else if (i < 8) colorClass = 'bg-cartoon-lime shadow-[0_0_10px_#a3e635] border-black';
          else if (i < 12) colorClass = 'bg-cartoon-yellow shadow-[0_0_10px_#facc15] border-black';
          else if (i < 16) colorClass = 'bg-orange-500 shadow-[0_0_10px_#f97316] border-black';
          else if (i < 20) colorClass = 'bg-cartoon-red shadow-[0_0_10px_#ef4444] border-black';
          else colorClass = 'bg-cartoon-purple shadow-[0_0_10px_#a855f7] border-black';
      }

      return (
        <div key={i} className="flex-1 h-full px-[1px] group relative">
             <div 
                className={`w-full h-full transform skew-x-[-12deg] transition-all duration-75 border-2 ${colorClass} ${isActive ? 'scale-y-100' : 'scale-y-[0.3] opacity-30'}`}
             ></div>
        </div>
      );
    });
  };

  const containerShake = isImpact ? 'animate-shake-hard' : '';

  return (
    <div className={`relative z-10 flex flex-col h-screen w-full overflow-hidden transition-colors duration-200 bg-white ${containerShake}`}>
      
      {/* Header / Stats */}
      <div className="absolute top-0 left-0 w-full p-4 flex justify-between items-start z-20">
        
        {/* Left Column: Volume + Champ & History */}
        <div className="flex flex-col gap-2 items-start">
            {/* Volume Button - Top Left for Phone */}
            <button 
                onClick={handleMuteToggle}
                className={`bg-white border-4 border-black p-3 shadow-hard hover:shadow-none hover:translate-y-1 transition-all ${isMuted ? 'text-gray-400' : 'text-black'}`}
            >
                {isMuted ? <VolumeX size={24} /> : <Volume2 size={24} />}
            </button>

            {/* Champ Box */}
            <div className="bg-white border-4 border-black p-3 shadow-hard transform -rotate-1">
            <div className="flex items-center gap-2 text-black mb-1">
                <Crown size={18} fill="#facc15" stroke="black" strokeWidth={3} />
                <span className="text-xs font-black uppercase tracking-widest">Champ</span>
            </div>
            <span className="text-black text-4xl font-black tracking-tighter">
                {highScore.toString().padStart(4, '0')}
            </span>
            </div>

            {/* Recent History Box */}
            {history.length > 0 && (
                <div className="bg-white border-4 border-black p-2 shadow-hard transform rotate-1 w-full animate-pop-in">
                    <div className="text-[10px] font-black uppercase tracking-widest border-b-2 border-black mb-1 text-black">
                        Last 5
                    </div>
                    <div className="flex flex-col gap-0.5">
                        {history.map((score, i) => (
                            <div key={i} className={`flex justify-between font-mono font-black text-xs ${i === 0 ? 'text-black' : 'text-gray-400'}`}>
                                <span>#{i + 1}</span>
                                <span>{score.toString().padStart(4, '0')}</span>
                            </div>
                        ))}
                    </div>
                </div>
            )}
        </div>

        {/* Right Buttons */}
        <div className="flex gap-4">
            {/* Settings Button */}
            <button 
                onClick={onOpenSettings}
                className="bg-white border-4 border-black p-3 shadow-hard hover:shadow-none hover:translate-y-1 transition-all text-black"
            >
                <Settings size={24} />
            </button>
        </div>
      </div>

      {/* Main Score Area */}
      <div className="flex-1 flex flex-col items-center justify-center relative">
        
        {/* NEW RECORD OVERLAY */}
        {isNewRecord && (
            <div className="absolute inset-0 z-50 flex items-center justify-center pointer-events-none bg-white/80 backdrop-blur-md">
                <div className="bg-cartoon-yellow border-8 border-black p-12 md:p-20 shadow-hard-xl transform -rotate-6 animate-shake-hard">
                    <h1 className="text-[15vw] md:text-9xl font-black italic text-black uppercase tracking-tighter drop-shadow-md animate-pulse text-center leading-none">
                        NEW<br/>RECORD!
                    </h1>
                </div>
            </div>
        )}

        <div className="relative z-10 flex flex-col items-center">
          
          {/* Badge */}
          <div className="bg-cartoon-cyan border-4 border-black px-6 py-2 shadow-hard transform rotate-2 mb-8 animate-bounce-slow">
             <span className="font-black uppercase tracking-widest text-black">Smash Rating</span>
          </div>
          
          {/* Comic Score */}
          <div className="relative">
             {/* Black Extrusion/Shadow Text */}
             <div className="absolute top-2 left-2 text-black font-black text-[25vw] md:text-[18rem] leading-none tracking-tighter select-none z-0">
                {displayScore.toString().padStart(4, '0')}
             </div>
             
             {/* Main Text */}
             <div className="relative z-10 font-black text-[25vw] md:text-[18rem] leading-none tracking-tighter tabular-nums text-cartoon-yellow text-stroke-black" 
                  style={{ WebkitTextStroke: '6px black' }}>
                {displayScore.toString().padStart(4, '0')}
             </div>
          </div>
        </div>
      </div>

      {/* CARTOON FORCE GAUGE */}
      <div className="w-full px-4 pb-8 max-w-5xl mx-auto">
        
        {/* Gauge Header */}
        <div className="flex justify-between items-end mb-2 px-2">
           <div className="bg-black text-white px-3 py-1 font-black uppercase text-sm transform skew-x-[-12deg] shadow-hard">
               <span className="block transform skew-x-[12deg]">Power Meter</span>
           </div>
           
           <div className="font-black text-3xl text-black">
               {Math.round(visualGauge > 5 ? visualGauge : 0)}<span className="text-sm text-gray-500">%</span>
           </div>
        </div>
        
        {/* The Gauge Container */}
        <div className="p-3 bg-gray-200 border-4 border-black shadow-hard-lg relative">
            {/* Background Stripes */}
            <div className="absolute inset-0 opacity-10" style={{ backgroundImage: 'repeating-linear-gradient(45deg, #000 0, #000 10px, transparent 10px, transparent 20px)' }}></div>
            
            {/* Inner Well */}
            <div className="relative bg-white border-2 border-black h-20 w-full flex items-end justify-between p-1 gap-[2px]">
                {renderGaugeSegments()}
            </div>
        </div>
        
        {/* Tier Labels */}
        <div className="flex justify-between text-[10px] md:text-xs uppercase font-black mt-4 px-2 tracking-widest text-black">
            <span>Wimp</span>
            <span>Loser</span>
            <span>Average</span>
            <span className="text-cartoon-purple">Pro</span>
        </div>
      </div>
    </div>
  );
};