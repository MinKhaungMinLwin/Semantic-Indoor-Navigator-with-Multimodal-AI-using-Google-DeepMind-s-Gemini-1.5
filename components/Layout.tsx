import React from 'react';
import { Cpu, Lightbulb, Activity, Map, Github } from 'lucide-react';
import { AppMode } from '../types';

interface LayoutProps {
  children: React.ReactNode;
  currentMode: AppMode;
  onModeChange: (mode: AppMode) => void;
}

export const Layout: React.FC<LayoutProps> = ({ children, currentMode, onModeChange }) => {
  return (
    <div className="min-h-screen bg-slate-950 text-slate-200 flex flex-col">
      {/* Header */}
      <header className="border-b border-slate-800 bg-slate-900/50 backdrop-blur-md sticky top-0 z-50">
        <div className="max-w-7xl mx-auto px-4 h-16 flex items-center justify-between">
          <div className="flex items-center space-x-3">
            <div className="w-10 h-10 bg-blue-600 rounded-lg flex items-center justify-center shadow-[0_0_15px_rgba(37,99,235,0.5)]">
              <Cpu className="text-white w-6 h-6" />
            </div>
            <div>
              <h1 className="text-lg font-bold tracking-wider text-white">GEMINI CORTEX</h1>
              <p className="text-xs text-blue-400 font-mono tracking-widest">ROBOTICS 1.5.0</p>
            </div>
          </div>
          
          <nav className="flex items-center space-x-1 bg-slate-800/50 p-1 rounded-full border border-slate-700 overflow-x-auto max-w-[200px] md:max-w-none">
            <button 
              onClick={() => onModeChange(AppMode.IDEATION)}
              className={`px-3 md:px-4 py-2 rounded-full text-sm font-medium transition-all duration-200 flex items-center space-x-2 whitespace-nowrap ${currentMode === AppMode.IDEATION ? 'bg-blue-600 text-white shadow-md' : 'text-slate-400 hover:text-white hover:bg-slate-700'}`}
            >
              <Lightbulb className="w-4 h-4" />
              <span className="hidden md:inline">Ideas</span>
            </button>
            <button 
              onClick={() => onModeChange(AppMode.SIMULATION)}
              className={`px-3 md:px-4 py-2 rounded-full text-sm font-medium transition-all duration-200 flex items-center space-x-2 whitespace-nowrap ${currentMode === AppMode.SIMULATION ? 'bg-indigo-600 text-white shadow-md' : 'text-slate-400 hover:text-white hover:bg-slate-700'}`}
            >
              <Activity className="w-4 h-4" />
              <span className="hidden md:inline">Simulation</span>
            </button>
            <button 
              onClick={() => onModeChange(AppMode.NAVIGATION)}
              className={`px-3 md:px-4 py-2 rounded-full text-sm font-medium transition-all duration-200 flex items-center space-x-2 whitespace-nowrap ${currentMode === AppMode.NAVIGATION ? 'bg-emerald-600 text-white shadow-md' : 'text-slate-400 hover:text-white hover:bg-slate-700'}`}
            >
              <Map className="w-4 h-4" />
              <span className="hidden md:inline">Navigator</span>
            </button>
          </nav>

          <div className="hidden md:flex items-center text-xs text-slate-500 font-mono">
            <span className="w-2 h-2 rounded-full bg-green-500 animate-pulse mr-2"></span>
            SYSTEM ONLINE
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="flex-grow max-w-7xl mx-auto w-full p-4 md:p-6">
        {children}
      </main>

      {/* Footer */}
      <footer className="border-t border-slate-800 py-6 text-center text-slate-500 text-sm">
        <p>Powered by Google DeepMind Gemini 2.5 Flash</p>
        <p className="text-xs mt-1">This is a simulation interface. No physical robots are harmed in the process.</p>
      </footer>
    </div>
  );
};
