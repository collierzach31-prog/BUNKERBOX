import React, { useEffect, useState, useRef } from 'react';
import { Settings, Zap, Crown, Flame, Volume2, VolumeX } from 'lucide-react';
import { SensorData } from '../types';

interface GameScreenProps {
  sensorData: SensorData;
  onOpenSettings: () => void;
  sensitivity: number;
}

// --- AUDIO ENGINE ---
const audioCtx = typeof window !== 'undefined' ? new (window.AudioContext || (window as any).webkitAudioContext)() : null;

// SYNTHESIZED ARCADE SFX
const playArcadeSound = (type: 'impact' | 'tick' | 'win' | 'lock', intensity: number = 0.5) => {
  if (!audioCtx) return;
  if (audioCtx.state === 'suspended') audioCtx.resume();
  
  const now = audioCtx.currentTime;
  const gain = audioCtx.createGain();
  gain.connect(audioCtx.destination);

  if (type === 'impact') {
    // 8-BIT CRUNCH IMPACT (Square Wave Sweep)
    const osc = audioCtx.createOscillator();
    osc.type = 'square'; // Classic Arcade Grit
    osc.connect(gain);
    
    // Quick pitch drop
    osc.frequency.setValueAtTime(150, now);
    osc.frequency.exponentialRampToValueAtTime(40, now + 0.3);
    
    // Punchy Envelope
    gain.gain.setValueAtTime(0.8, now);
    gain.gain.exponentialRampToValueAtTime(0.01, now + 0.3);
    
    osc.start(now);
    osc.stop(now + 0.3);

    // White Noise Burst (The "Smack")
    const bufferSize = audioCtx.sampleRate * 0.1; // 100ms
    const buffer = audioCtx.createBuffer(1, bufferSize, audioCtx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < bufferSize; i++) data[i] = (Math.random() * 2 - 1) * 0.5;
    
    const noise = audioCtx.createBufferSource();
    noise.buffer = buffer;
    const noiseGain = audioCtx.createGain();
    noise.connect(noiseGain);
    noiseGain.connect(audioCtx.destination);
    
    noiseGain.gain.setValueAtTime(0.6, now);
    noiseGain.gain.exponentialRampToValueAtTime(0.01, now + 0.1);
    noise.start(now);

  } else if (type === 'tick') {
    // 8-BIT COIN BLIP
    const osc = audioCtx.createOscillator();
    osc.type = 'square';
    osc.connect(gain);
    
    // Pentatonic scale notes for musicality
    const notes = [523.25, 659.25, 783.99, 1046.50, 1318.51, 1567.98];
    const freq = notes[Math.floor(Math.random() * notes.length)];
    
    osc.frequency.setValueAtTime(freq, now);
    
    // Very short, very staccato
    gain.gain.setValueAtTime(0.3, now); 
    gain.gain.exponentialRampToValueAtTime(0.01, now + 0.05);
    
    osc.start(now);
    osc.stop(now + 0.05);

  } else if (type === 'win') {
    // LEVEL UP FANFARE (Fast Arpeggio)
    const notes = [523.25, 659.25, 783.99, 1046.50, 1318.51, 1567.98, 2093.00];
    
    notes.forEach((freq, i) => {
        const osc = audioCtx.createOscillator();
        const noteGain = audioCtx.createGain();
        osc.type = 'square'; // NES Style
        osc.frequency.value = freq;
        
        osc.connect(noteGain);
        noteGain.connect(audioCtx.destination);
        
        const startTime = now + (i * 0.08); 
        
        noteGain.gain.setValueAtTime(0.15, startTime);
        noteGain.gain.exponentialRampToValueAtTime(0.01, startTime + 0.2);
        
        osc.start(startTime);
        osc.stop(startTime + 0.2);
    });
  }
};

// BOOM BAP DRUM MACHINE
const playDrum = (type: 'kick' | 'snare' | 'hat' | 'bass', time: number) => {
    if (!audioCtx) return;
    const gain = audioCtx.createGain();
    gain.connect(audioCtx.destination);

    if (type === 'kick') {
        const osc = audioCtx.createOscillator();
        osc.type = 'sine';
        osc.connect(gain);
        osc.frequency.setValueAtTime(120, time);
        osc.frequency.exponentialRampToValueAtTime(50, time + 0.1);
        gain.gain.setValueAtTime(1.0, time);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.4);
        osc.start(time);
        osc.stop(time + 0.4);
    } else if (type === 'snare') {
        const osc = audioCtx.createOscillator();
        osc.type = 'triangle';
        osc.connect(gain);
        osc.frequency.setValueAtTime(200, time);
        gain.gain.setValueAtTime(0.4, time);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.1);
        osc.start(time);
        osc.stop(time + 0.1);

        // Snare Noise
        const bSize = audioCtx.sampleRate * 0.2;
        const buff = audioCtx.createBuffer(1, bSize, audioCtx.sampleRate);
        const dat = buff.getChannelData(0);
        for(let i=0; i<bSize; i++) dat[i] = (Math.random()*2-1);
        const noise = audioCtx.createBufferSource();
        noise.buffer = buff;
        const nGain = audioCtx.createGain();
        noise.connect(nGain);
        nGain.connect(audioCtx.destination);
        
        // Filter noise to sound crisp
        const filter = audioCtx.createBiquadFilter();
        filter.type = 'highpass';
        filter.frequency.value = 800;
        nGain.disconnect();
        noise.disconnect();
        noise.connect(filter);
        filter.connect(nGain);
        nGain.connect(audioCtx.destination);

        nGain.gain.setValueAtTime(0.4, time);
        nGain.gain.exponentialRampToValueAtTime(0.01, time + 0.2);
        noise.start(time);
    } else if (type === 'hat') {
        const bSize = audioCtx.sampleRate * 0.05;
        const buff = audioCtx.createBuffer(1, bSize, audioCtx.sampleRate);
        const dat = buff.getChannelData(0);
        for(let i=0; i<bSize; i++) dat[i] = (Math.random()*2-1); // white noise
        const noise = audioCtx.createBufferSource();
        noise.buffer = buff;
        
        const filter = audioCtx.createBiquadFilter();
        filter.type = 'highpass';
        filter.frequency.value = 5000;
        
        noise.connect(filter);
        filter.connect(gain);
        
        gain.gain.setValueAtTime(0.15, time);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.05);
        noise.start(time);
    } else if (type === 'bass') {
        const osc = audioCtx.createOscillator();
        osc.type = 'sawtooth'; // Gritty synth bass
        osc.connect(gain);
        
        // Simple bassline note (F1)
        osc.frequency.setValueAtTime(43.65, time); 
        
        // Lowpass filter for "underwater" feel
        const filter = audioCtx.createBiquadFilter();
        filter.type = 'lowpass';
        filter.frequency.value = 400;
        osc.disconnect();
        osc.connect(filter);
        filter.connect(gain);

        gain.gain.setValueAtTime(0.3, time);
        gain.gain.linearRampToValueAtTime(0.3, time + 0.2);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.6);
        osc.start(time);
        osc.stop(time + 0.6);
    }
}

// --------------------

export const GameScreen: React.FC<GameScreenProps> = ({ 
  sensorData, 
  onOpenSettings, 
  sensitivity 
}) => {
  const [targetScore, setTargetScore] = useState<number>(0);
  const [displayScore, setDisplayScore] = useState<number>(0);
  const [highScore, setHighScore] = useState<number>(0);
  
  // Gauge States
  const [visualGauge, setVisualGauge] = useState(0); 
  const [isGaugeFrozen, setIsGaugeFrozen] = useState(false);
  const [isImpact, setIsImpact] = useState(false);
  const [isNewRecord, setIsNewRecord] = useState(false);
  const [history, setHistory] = useState<number[]>([]);
  
  const [isMusicPlaying, setIsMusicPlaying] = useState(false);

  const lastPunchTime = useRef<number>(0);
  const scoreanimRef = useRef<number>(0);
  const startDelayRef = useRef<number | null>(null);
  const snapTimeoutRef = useRef<number | null>(null);
  const lastAudioTickRef = useRef<number>(0);
  const musicIntervalRef = useRef<number | null>(null);
  const beatStepRef = useRef<number>(0);
  
  const GRAVITY = 9.8; 
  const MAX_FORCE = 80; 
  const THRESHOLD = 12; 
  
  // Start Music Automatically on Mount (if desired, or user toggles)
  useEffect(() => {
     // Default off, user enables
     return () => {
         if (musicIntervalRef.current) clearInterval(musicIntervalRef.current);
     };
  }, []);

  const toggleMusic = () => {
      if (audioCtx && audioCtx.state === 'suspended') audioCtx.resume();
      
      if (isMusicPlaying) {
          if (musicIntervalRef.current) clearInterval(musicIntervalRef.current);
          setIsMusicPlaying(false);
      } else {
          setIsMusicPlaying(true);
          // 92 BPM = ~652ms per beat. 16th notes = ~163ms
          const stepTime = 163;
          beatStepRef.current = 0;
          
          musicIntervalRef.current = window.setInterval(() => {
              if (!audioCtx) return;
              const now = audioCtx.currentTime;
              const step = beatStepRef.current % 16;
              
              // Classic Boom Bap Pattern
              // Kick: 0, 4, 10
              // Snare: 4, 12
              // Hat: Every even step (8th notes)
              
              if (step === 0 || step === 10) playDrum('kick', now);
              if (step === 4 || step === 12) playDrum('snare', now);
              if (step % 2 === 0) playDrum('hat', now);
              
              // Simple Bass on the 'One'
              if (step === 0) playDrum('bass', now);

              beatStepRef.current++;
          }, stepTime);
      }
  };

  useEffect(() => {
    const now = Date.now();
    const rawForce = Math.max(0, sensorData.mag - GRAVITY);
    const currentForcePercent = Math.min((rawForce / MAX_FORCE) * 100, 100);

    // If frozen, do NOT update visualGauge
    if (!isGaugeFrozen) {
        setVisualGauge(prev => {
            if (currentForcePercent > prev) return currentForcePercent; 
            return Math.max(0, prev - 2.5); // Fast decay for snappy feel
        });
    }

    if (rawForce > THRESHOLD && now - lastPunchTime.current > 1000) {
      const normalizedForce = Math.min(rawForce / MAX_FORCE, 1);
      const curvedForce = Math.pow(normalizedForce, 1.4); 
      const sensitivityMultiplier = sensitivity / 50;
      
      let calculatedScore = Math.floor(curvedForce * 9999 * sensitivityMultiplier);
      if (calculatedScore > 9999) calculatedScore = 9999;
      if (calculatedScore < 100) calculatedScore = 100 + Math.floor(Math.random() * 50);

      // Instant freeze at peak (simulated peak based on current hit)
      const hitPeak = currentForcePercent;
      setVisualGauge(hitPeak); 
      setIsGaugeFrozen(true);

      triggerPunch(calculatedScore, normalizedForce);
      lastPunchTime.current = now;
    }
  }, [sensorData, sensitivity, isGaugeFrozen]);

  const triggerPunch = (finalScore: number, intensity: number) => {
    setTargetScore(finalScore);
    setIsImpact(true);
    
    // ALWAYS PLAY MAX IMPACT SOUND
    playArcadeSound('impact', 1.0); 
    
    setTimeout(() => setIsImpact(false), 200);

    if (scoreanimRef.current) cancelAnimationFrame(scoreanimRef.current);
    if (startDelayRef.current) clearTimeout(startDelayRef.current);
    if (snapTimeoutRef.current) clearTimeout(snapTimeoutRef.current);

    // Delay before rolling score
    startDelayRef.current = window.setTimeout(() => {
        const startTime = performance.now();
        
        // RESET TICK TIMER FOR NEW ANIMATION
        lastAudioTickRef.current = -100;

        const startScore = 0; 
        const duration = 2000; 

        const animateScore = (time: number) => {
            const elapsed = time - startTime;
            const progress = Math.min(elapsed / duration, 1);
            
            // Quintic Out Easing
            const ease = 1 - Math.pow(1 - progress, 5);
            
            let current = Math.floor(startScore + (finalScore - startScore) * ease);
            let jitter = 0;
            if (progress < 0.5) jitter = Math.floor(Math.random() * (finalScore * 0.05));

            setDisplayScore(current + jitter);

            // Audio: Slot Machine Ticks (every 30ms for faster, more intense rolling sound)
            if (elapsed - lastAudioTickRef.current > 30 && progress < 0.98) {
                playArcadeSound('tick');
                lastAudioTickRef.current = elapsed;
            }

            if (progress < 1) {
                scoreanimRef.current = requestAnimationFrame(animateScore);
            } else {
                setDisplayScore(finalScore);
                
                // Play final flourish for EVERY punch
                playArcadeSound('win');
                
                // Score Locked: Update records
                if (finalScore > highScore) {
                    setHighScore(finalScore);
                    setIsNewRecord(true);
                    setTimeout(() => setIsNewRecord(false), 3000);
                }
                
                setHistory(prev => [finalScore, ...prev].slice(0, 5));

                // Snap Gauge Back
                snapTimeoutRef.current = window.setTimeout(() => {
                    setIsGaugeFrozen(false);
                    setVisualGauge(0);
                }, 200); 
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
      
      {/* NEW RECORD OVERLAY */}
      {isNewRecord && (
          <div className="absolute inset-0 z-50 flex items-center justify-center pointer-events-none animate-pop-in">
              <div className="bg-cartoon-yellow border-4 border-black p-12 shadow-hard-xl transform -rotate-3">
                  <h1 className="text-8xl font-black italic text-black uppercase tracking-tighter drop-shadow-md">
                      NEW RECORD!
                  </h1>
              </div>
          </div>
      )}

      {/* Header / Stats */}
      <div className="absolute top-0 left-0 w-full p-4 flex justify-between items-start z-20">
        
        {/* Left Column: Champ & History */}
        <div className="flex flex-col gap-2 items-start">
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
            {/* Music Toggle */}
            <button 
                onClick={toggleMusic}
                className={`bg-white border-4 border-black p-3 shadow-hard hover:shadow-none hover:translate-y-1 transition-all ${isMusicPlaying ? 'text-cartoon-purple' : 'text-gray-400'}`}
            >
                {isMusicPlaying ? <Volume2 size={24} /> : <VolumeX size={24} />}
            </button>

            {/* Settings Btn */}
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
               {Math.round(visualGauge)}<span className="text-sm text-gray-500">%</span>
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
            <span>Average</span>
            <span>Fighter</span>
            <span className="text-cartoon-purple">Legend</span>
        </div>
      </div>
    </div>
  );
};