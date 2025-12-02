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
        if (audioCtx && audioCtx.state === 'suspended') {
            audioCtx.resume().catch(() => {});
        }

        // ~120 BPM (60 BPM Half-time) - Slower, Heavier Miami Bounce
        const stepTime = 125;
        beatStepRef.current = 0;
        
        musicIntervalRef.current = window.setInterval(() => {
            if (!audioCtx) return;
            if (audioCtx.state === 'suspended') return;

            const now = audioCtx.currentTime;
            const step = beatStepRef.current % 32; // 2 Bar Loop
            
            // Kick: Spaced out Miami Pattern
            // Bar 1: 1, 3& (Steps 0, 10)
            // Bar 2: 1, 2&, 4 (Steps 16, 22, 28)
            if (step === 0 || step === 10 || step === 16 || step === 22 || step === 28) {
                playDrum('kick', now);
            }
            
            // Snare: Hard on 3 (Half-time feel) -> Steps 8 and 24
            if (step === 8 || step === 24) {
                playDrum('snare', now);
            }
            
            // Hi-hats: Slower flow (8th notes instead of 16ths)
            // Steps: 0, 2, 4, 6... 
            if (step % 2 === 0) {
                playDrum('hat', now);
            }
            // Occasional triplet roll at the end of the loop
            if (step === 30 || step === 31) {
                playDrum('hat', now);
            }
            
            // Melody: "Cartoony" Plucks (Syncopated)
            // Slightly simplified pattern
            if (step === 0 || step === 3 || step === 6 || step === 14 || 
                step === 16 || step === 19 || step === 27) {
                // @ts-ignore
                playDrum('melody', now);
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
