import React, { useEffect, useRef, useState } from 'react';
import { Volume2, VolumeX } from 'lucide-react';
import { audioCtx, playDrum } from '../services/audioService';

export const BackgroundMusic = React.memo(() => {
  const [isMusicPlaying, setIsMusicPlaying] = useState(true);
  const musicIntervalRef = useRef<number | null>(null);
  const beatStepRef = useRef<number>(0);

  // Attempt to resume audio context on any user interaction
  useEffect(() => {
      const handleInteraction = () => {
          if (audioCtx && audioCtx.state === 'suspended') {
              audioCtx.resume().then(() => {
                  console.log("Audio Context Resumed by User Interaction");
              }).catch(e => console.error(e));
          }
      };

      window.addEventListener('click', handleInteraction);
      window.addEventListener('touchstart', handleInteraction);
      window.addEventListener('keydown', handleInteraction);

      return () => {
          window.removeEventListener('click', handleInteraction);
          window.removeEventListener('touchstart', handleInteraction);
          window.removeEventListener('keydown', handleInteraction);
      };
  }, []);

  useEffect(() => {
    if (isMusicPlaying) {
        // Try to resume immediately (works if policy allows)
        if (audioCtx && audioCtx.state === 'suspended') {
            audioCtx.resume().catch(() => {});
        }

        // ~140 BPM trap tempo (428ms per beat, ~107ms per 16th note)
        const stepTime = 107;
        beatStepRef.current = 0;
        
        musicIntervalRef.current = window.setInterval(() => {
            if (!audioCtx) return;
            
            // If context is still suspended, we can't hear anything, but we keep the loop running
            // so it picks up immediately when resumed.
            if (audioCtx.state === 'suspended') return;

            const now = audioCtx.currentTime;
            const step = beatStepRef.current % 32; // 2-bar loop
            
            // TRAP PATTERN
            // Kick on 1, and syncopated hits
            if (step === 0 || step === 6 || step === 16 || step === 22) {
                playDrum('kick', now);
            }
            
            // Snare on 2 and 4 (steps 8 and 24)
            if (step === 8 || step === 24) {
                playDrum('snare', now);
            }
            
            // Hi-hats - rapid fire trap style
            if (step % 2 === 0) {
                playDrum('hat', now);
            }
            
            // Hi-hat rolls before the snare (signature trap sound)
            if (step === 7 || step === 23) {
                playDrum('hatRoll', now);
            }
            
            // Extra hat triplets for that bounce
            if (step === 3 || step === 11 || step === 19 || step === 27) {
                playDrum('hat', now);
            }
            
            // Sliding 808 bass on the 1
            if (step === 0 || step === 16) {
                playDrum('808slide', now);
            }

            beatStepRef.current++;
        }, stepTime);
    } else {
        if (musicIntervalRef.current) clearInterval(musicIntervalRef.current);
    }

    return () => {
        if (musicIntervalRef.current) clearInterval(musicIntervalRef.current);
    };
  }, [isMusicPlaying]);

  return (
    <button 
        onClick={() => setIsMusicPlaying(!isMusicPlaying)}
        className="fixed bottom-4 left-4 z-50 bg-black text-white p-2 border-2 border-white shadow-hard hover:scale-110 transition-transform"
    >
        {isMusicPlaying ? <Volume2 size={24} /> : <VolumeX size={24} />}
    </button>
  );
});
