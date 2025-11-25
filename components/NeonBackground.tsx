import React from 'react';

export const NeonBackground: React.FC = () => {
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

      {/* Floating Shapes for Energy */}
      <div className="absolute top-10 left-10 w-24 h-24 border-4 border-cartoon-cyan rounded-full opacity-20 animate-bounce-slow"></div>
      <div className="absolute bottom-20 right-20 w-32 h-32 border-4 border-cartoon-magenta rotate-12 opacity-20 animate-bounce-slow" style={{ animationDelay: '1s' }}></div>
      
    </div>
  );
};