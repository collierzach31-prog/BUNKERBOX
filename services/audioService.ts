// --- AUDIO ENGINE ---
export const audioCtx = typeof window !== 'undefined' ? new (window.AudioContext || (window as any).webkitAudioContext)() : null;

let sfxVolume = 0.5;
let musicVolume = 0.5;

export const setVolumes = (sfx: number, music: number) => {
    sfxVolume = sfx / 100;
    musicVolume = music / 100;
};

export const playConnectionSound = (success: boolean) => {
    if (!audioCtx) return;
    if (audioCtx.state === 'suspended') {
        audioCtx.resume().catch(e => console.warn("Audio resume failed:", e));
    }
    const now = audioCtx.currentTime;
    const gain = audioCtx.createGain();
    gain.connect(audioCtx.destination);
    
    if (success) {
        // Happy "Ding-Dong"
        const osc = audioCtx.createOscillator();
        osc.type = 'sine';
        osc.connect(gain);
        
        osc.frequency.setValueAtTime(523.25, now); // C5
        osc.frequency.setValueAtTime(783.99, now + 0.15); // G5
        
        gain.gain.setValueAtTime(0.5 * sfxVolume, now);
        gain.gain.linearRampToValueAtTime(0, now + 0.6);
        
        osc.start(now);
        osc.stop(now + 0.6);
    } else {
        // Sad "Bwomp"
        const osc = audioCtx.createOscillator();
        osc.type = 'sawtooth';
        osc.connect(gain);
        
        osc.frequency.setValueAtTime(150, now);
        osc.frequency.exponentialRampToValueAtTime(50, now + 0.4);
        
        gain.gain.setValueAtTime(0.5 * sfxVolume, now);
        gain.gain.linearRampToValueAtTime(0, now + 0.4);
        
        osc.start(now);
        osc.stop(now + 0.4);
    }
};

// SYNTHESIZED ARCADE SFX
export const playArcadeSound = (type: 'impact' | 'tick' | 'win' | 'lock' | 'newRecord', intensity: number = 0.5) => {
  if (!audioCtx) return;
  if (audioCtx.state === 'suspended') {
      audioCtx.resume().catch(e => console.warn("Audio resume failed:", e));
  }
  
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
    gain.gain.setValueAtTime(0.8 * sfxVolume, now);
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
    
    noiseGain.gain.setValueAtTime(0.6 * sfxVolume, now);
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
    gain.gain.setValueAtTime(0.3 * sfxVolume, now); 
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
        
        noteGain.gain.setValueAtTime(0.15 * sfxVolume, startTime);
        noteGain.gain.exponentialRampToValueAtTime(0.01, startTime + 0.2);
        
        osc.start(startTime);
        osc.stop(startTime + 0.2);
    });
  } else if (type === 'newRecord') {
    // MAJOR VICTORY FANFARE (Triplet feel)
    // C4, E4, G4, C5, G4, C5, E5, C6
    const notes = [
        { f: 523.25, t: 0.0 },  // C5
        { f: 523.25, t: 0.1 },  // C5
        { f: 523.25, t: 0.2 },  // C5
        { f: 659.25, t: 0.3 },  // E5
        { f: 783.99, t: 0.45 }, // G5
        { f: 1046.50, t: 0.6 }, // C6
        { f: 783.99, t: 0.75 }, // G5
        { f: 1046.50, t: 0.9 }  // C6 (Long)
    ];
    
    notes.forEach((note, i) => {
        const osc = audioCtx.createOscillator();
        const noteGain = audioCtx.createGain();
        
        // Mix of Square and Sawtooth for "Full" sound
        osc.type = i % 2 === 0 ? 'square' : 'sawtooth'; 
        osc.frequency.value = note.f;
        
        osc.connect(noteGain);
        noteGain.connect(audioCtx.destination);
        
        const startTime = now + note.t;
        const duration = i === notes.length - 1 ? 0.8 : 0.15;
        
        noteGain.gain.setValueAtTime(0.3 * sfxVolume, startTime);
        noteGain.gain.exponentialRampToValueAtTime(0.01, startTime + duration);
        
        osc.start(startTime);
        osc.stop(startTime + duration);
    });
  }
};

// DARK TRAP DRUM MACHINE
export const playDrum = (type: 'kick' | 'snare' | 'hat' | 'bass', time: number) => {
    if (!audioCtx) return;
    const gain = audioCtx.createGain();
    gain.connect(audioCtx.destination);

    if (type === 'kick') {
        // Heavy 808 kick - deep and punchy
        const osc = audioCtx.createOscillator();
        osc.type = 'sine';
        osc.connect(gain);
        osc.frequency.setValueAtTime(120, time);
        osc.frequency.exponentialRampToValueAtTime(30, time + 0.2);
        gain.gain.setValueAtTime(1.0 * musicVolume, time);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.5);
        osc.start(time);
        osc.stop(time + 0.5);
        
    } else if (type === 'snare') {
        // Hard clap/snare
        const osc = audioCtx.createOscillator();
        osc.type = 'triangle';
        osc.connect(gain);
        osc.frequency.setValueAtTime(180, time);
        gain.gain.setValueAtTime(0.3 * musicVolume, time);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.08);
        osc.start(time);
        osc.stop(time + 0.08);

        // Snare noise - crispy
        const bSize = audioCtx.sampleRate * 0.2;
        const buff = audioCtx.createBuffer(1, bSize, audioCtx.sampleRate);
        const dat = buff.getChannelData(0);
        for(let i=0; i<bSize; i++) dat[i] = (Math.random()*2-1);
        const noise = audioCtx.createBufferSource();
        noise.buffer = buff;
        
        const filter = audioCtx.createBiquadFilter();
        filter.type = 'highpass';
        filter.frequency.value = 1200;
        
        const nGain = audioCtx.createGain();
        noise.connect(filter);
        filter.connect(nGain);
        nGain.connect(audioCtx.destination);

        nGain.gain.setValueAtTime(0.45 * musicVolume, time);
        nGain.gain.exponentialRampToValueAtTime(0.01, time + 0.2);
        noise.start(time);
        
    } else if (type === 'hat') {
        // Dark hi-hat
        const bSize = audioCtx.sampleRate * 0.05;
        const buff = audioCtx.createBuffer(1, bSize, audioCtx.sampleRate);
        const dat = buff.getChannelData(0);
        for(let i=0; i<bSize; i++) dat[i] = (Math.random()*2-1);
        const noise = audioCtx.createBufferSource();
        noise.buffer = buff;
        
        const filter = audioCtx.createBiquadFilter();
        filter.type = 'highpass';
        filter.frequency.value = 5000;
        
        noise.connect(filter);
        filter.connect(gain);
        
        gain.gain.setValueAtTime(0.12 * musicVolume, time);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 0.05);
        noise.start(time);
        
    } else if (type === 'bass') {
        // Deep sliding 808 - long sustain
        const osc = audioCtx.createOscillator();
        osc.type = 'sine';
        osc.connect(gain);
        
        // Slide down for that dark feel
        osc.frequency.setValueAtTime(50, time);
        osc.frequency.exponentialRampToValueAtTime(32, time + 0.3);
        
        gain.gain.setValueAtTime(0.7 * musicVolume, time);
        gain.gain.setValueAtTime(0.65 * musicVolume, time + 0.5);
        gain.gain.exponentialRampToValueAtTime(0.01, time + 1.2);
        osc.start(time);
        osc.stop(time + 1.2);
    }
};
