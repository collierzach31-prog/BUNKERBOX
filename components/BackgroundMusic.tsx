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

        // ~95 BPM - punchy hip-hop tempo
        const stepTime = 158;
        beatStepRef.current = 0;
        
        musicIntervalRef.current = window.setInterval(() => {
            if (!audioCtx) return;
            if (audioCtx.state === 'suspended') return;

            const now = audioCtx.currentTime;
            const step = beatStepRef.current % 16;
            
            // Punchy kick pattern
            if (step === 0 || step === 10) {
                playDrum('kick', now);
            }
            
            // Snare on 2 and 4
            if (step === 4 || step === 12) {
                playDrum('snare', now);
            }
            
            // Hi-hats - steady groove
            if (step % 2 === 0) {
                playDrum('hat', now);
            }
            
            // Bass on 1 and 3
            if (step === 0 || step === 8) {
                playDrum('bass', now);
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
