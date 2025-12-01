import React from 'react';

export const NeonBackground = React.memo(() => {
  return (
    <div className="fixed inset-0 z-0 bg-white overflow-hidden pointer-events-none">
      
      {/* Comic Halftone Pattern */}
      <div 
        className="absolute inset-0 opacity-10"
        style={{
          backgroundImage: 'radial-gradient(#000 20%, transparent 20%)',
          backgroundPosition: '0 0',
          backgroundSize: '16px 16px'
        }}
      ></div>

      {/* Diagonal Stripes Accent (Bottom Right) */}
      <div className="absolute bottom-0 right-0 w-1/2 h-1/2 opacity-5"
         style={{
           backgroundImage: 'repeating-linear-gradient(-45deg, #000, #000 2px, transparent 2px, transparent 10px)'
         }}
      ></div>

      {/* === FLOATING SHAPES === */}
      
      {/* Large Cyan Circle - Top Left */}
      <div className="absolute top-10 left-10 w-24 h-24 border-4 border-cartoon-cyan rounded-full opacity-30 animate-float"></div>
      
      {/* Magenta Square - Bottom Right */}
      <div className="absolute bottom-20 right-20 w-32 h-32 border-4 border-cartoon-magenta rotate-12 opacity-30 animate-float-delayed"></div>
      
      {/* Yellow Circle - Top Right */}
      <div className="absolute top-20 right-32 w-16 h-16 bg-cartoon-yellow border-4 border-black rounded-full opacity-20 animate-bounce-slow"></div>
      
      {/* Small Pink Square - Left Center */}
      <div className="absolute top-1/2 left-8 w-12 h-12 bg-cartoon-pink border-3 border-black rotate-45 opacity-25 animate-spin-slow"></div>
      
      {/* Cyan Diamond - Bottom Left */}
      <div className="absolute bottom-32 left-24 w-20 h-20 border-4 border-cartoon-cyan rotate-45 opacity-20 animate-pulse"></div>
      
      {/* Purple Circle - Center Right */}
      <div className="absolute top-1/3 right-12 w-14 h-14 bg-cartoon-purple border-4 border-black rounded-full opacity-20 animate-float"></div>
      
      {/* Large Outline Square - Top Center */}
      <div className="absolute top-16 left-1/2 -translate-x-1/2 w-28 h-28 border-4 border-black rotate-6 opacity-10 animate-float-delayed"></div>
      
      {/* Small Yellow Diamond - Bottom Center */}
      <div className="absolute bottom-16 left-1/2 w-10 h-10 bg-cartoon-yellow border-3 border-black rotate-45 opacity-30 animate-bounce-slow" style={{ animationDelay: '0.5s' }}></div>
      
      {/* Lime Circle - Mid Left */}
      <div className="absolute top-2/3 left-16 w-8 h-8 bg-cartoon-lime border-2 border-black rounded-full opacity-25 animate-ping-slow"></div>
      
      {/* Red Square - Mid Right */}
      <div className="absolute top-1/2 right-24 w-6 h-6 bg-cartoon-red border-2 border-black opacity-20 animate-spin-slow" style={{ animationDelay: '1s' }}></div>
      
      {/* Large Outline Circle - Bottom */}
      <div className="absolute bottom-10 left-1/3 w-36 h-36 border-4 border-cartoon-purple rounded-full opacity-10 animate-float"></div>
      
      {/* Small Cyan Squares scattered */}
      <div className="absolute top-1/4 left-1/4 w-4 h-4 bg-cartoon-cyan border-2 border-black opacity-30 animate-bounce-slow" style={{ animationDelay: '0.2s' }}></div>
      <div className="absolute top-3/4 right-1/4 w-5 h-5 bg-cartoon-cyan border-2 border-black rotate-12 opacity-25 animate-bounce-slow" style={{ animationDelay: '0.8s' }}></div>
      
      {/* Floating Stars (using rotated squares) */}
      <div className="absolute top-1/5 right-1/3 w-6 h-6 bg-cartoon-yellow border-2 border-black rotate-45 opacity-30 animate-twinkle"></div>
      <div className="absolute bottom-1/4 left-1/3 w-4 h-4 bg-cartoon-pink border-2 border-black rotate-45 opacity-25 animate-twinkle" style={{ animationDelay: '0.5s' }}></div>
      
    </div>
  );
});