import React from 'react';

interface NeonButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'cyan' | 'pink' | 'yellow'; 
  pulse?: boolean;
}

export const NeonButton: React.FC<NeonButtonProps> = ({ 
  children, 
  variant = 'cyan', 
  pulse = false,
  className = '', 
  ...props 
}) => {
  // Cartoon Arcade Styles
  const styles = {
    // Action Button (Cyan)
    cyan: 'bg-cartoon-cyan text-black hover:bg-cyan-300',
    
    // Danger/Fight Button (Red/Pink)
    pink: 'bg-cartoon-red text-white hover:bg-red-500',
    
    // Secondary/Menu (Yellow)
    yellow: 'bg-cartoon-yellow text-black hover:bg-yellow-300',
  };

  const baseStyles = "relative px-8 py-4 text-xl font-black uppercase tracking-widest border-4 border-black shadow-hard transition-all duration-75 active:shadow-none active:translate-x-1 active:translate-y-1 transform";
  const pulseClass = pulse ? "animate-pulse" : "";
  const selectedStyle = styles[variant];

  return (
    <button 
      className={`${baseStyles} ${selectedStyle} ${pulseClass} ${className}`}
      {...props}
    >
      {children}
    </button>
  );
};