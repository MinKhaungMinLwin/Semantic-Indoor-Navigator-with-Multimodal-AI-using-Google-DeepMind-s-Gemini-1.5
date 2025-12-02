import React, { useState, useRef, useEffect } from 'react';
import { generateNavigationPlan } from '../services/gemini';
import { NavigationPlan } from '../types';
import { Map as MapIcon, Target, Navigation, Upload, Play, RefreshCw, Loader2, AlertCircle, Eye } from 'lucide-react';

export const SemanticNavigator: React.FC = () => {
  const [image, setImage] = useState<File | null>(null);
  const [preview, setPreview] = useState<string | null>(null);
  const [instruction, setInstruction] = useState('');
  const [plan, setPlan] = useState<NavigationPlan | null>(null);
  const [loading, setLoading] = useState(false);
  const [animationStep, setAnimationStep] = useState(0);
  
  const fileInputRef = useRef<HTMLInputElement>(null);
  const animationRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    return () => {
      if (animationRef.current) clearInterval(animationRef.current);
    };
  }, []);

  const handleImageUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (e.target.files && e.target.files[0]) {
      const file = e.target.files[0];
      setImage(file);
      const reader = new FileReader();
      reader.onloadend = () => setPreview(reader.result as string);
      reader.readAsDataURL(file);
      setPlan(null); // Reset plan on new image
    }
  };

  const handleNavigate = async () => {
    if (!image || !instruction) return;
    setLoading(true);
    setPlan(null);
    try {
      const result = await generateNavigationPlan(image, instruction);
      setPlan(result);
      startAnimation(result.path.length);
    } catch (error) {
      console.error(error);
      alert("Navigation planning failed. See console.");
    } finally {
      setLoading(false);
    }
  };

  const startAnimation = (totalSteps: number) => {
    setAnimationStep(0);
    if (animationRef.current) clearInterval(animationRef.current);
    
    let step = 0;
    animationRef.current = setInterval(() => {
      step += 1;
      setAnimationStep(step);
      if (step >= totalSteps - 1) {
        if (animationRef.current) clearInterval(animationRef.current);
      }
    }, 800);
  };

  const reset = () => {
    setImage(null);
    setPreview(null);
    setInstruction('');
    setPlan(null);
    setAnimationStep(0);
  };

  return (
    <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 h-[calc(100vh-140px)] animate-in fade-in duration-500">
      {/* Control Panel */}
      <div className="lg:col-span-4 flex flex-col gap-4 overflow-y-auto">
        <div className="bg-slate-900 border border-slate-800 rounded-xl p-5 shadow-xl">
          <div className="flex items-center mb-4">
            <div className="p-2 bg-emerald-900/30 rounded-lg mr-3">
              <Navigation className="w-5 h-5 text-emerald-400" />
            </div>
            <div>
              <h2 className="text-lg font-bold text-white">Semantic Navigator</h2>
              <p className="text-xs text-slate-400">Context-aware path planning</p>
            </div>
          </div>
          
          <div className="space-y-4">
            <div>
              <label className="block text-xs font-mono text-slate-500 mb-2 uppercase">1. Environment Map / View</label>
              <div 
                onClick={() => fileInputRef.current?.click()}
                className="border-2 border-dashed border-slate-700 rounded-lg p-4 hover:bg-slate-800/50 hover:border-emerald-500/50 transition-colors cursor-pointer text-center group"
              >
                {preview ? (
                   <div className="relative">
                     <img src={preview} alt="Environment" className="w-full h-32 object-cover rounded opacity-60 group-hover:opacity-100 transition-opacity" />
                     <div className="absolute inset-0 flex items-center justify-center">
                       <RefreshCw className="w-6 h-6 text-white opacity-0 group-hover:opacity-100 drop-shadow-md" />
                     </div>
                   </div>
                ) : (
                  <div className="py-4">
                    <Upload className="w-8 h-8 text-slate-600 mx-auto mb-2 group-hover:text-emerald-400 transition-colors" />
                    <p className="text-xs text-slate-400">Upload floor plan or room photo</p>
                  </div>
                )}
                <input ref={fileInputRef} type="file" className="hidden" accept="image/*" onChange={handleImageUpload} />
              </div>
            </div>

            <div>
              <label className="block text-xs font-mono text-slate-500 mb-2 uppercase">2. Navigation Command</label>
              <textarea
                value={instruction}
                onChange={(e) => setInstruction(e.target.value)}
                placeholder="e.g. 'Navigate to the couch avoiding the coffee table'"
                className="w-full bg-slate-950 border border-slate-700 rounded-lg p-3 text-sm text-emerald-400 placeholder-slate-700 focus:ring-1 focus:ring-emerald-500 outline-none resize-none h-24 font-mono"
              />
            </div>

            <button
              onClick={handleNavigate}
              disabled={loading || !image || !instruction}
              className="w-full bg-emerald-600 hover:bg-emerald-500 text-white font-bold py-3 rounded-lg shadow-lg shadow-emerald-900/20 transition-all flex items-center justify-center space-x-2 disabled:opacity-50 disabled:cursor-not-allowed"
            >
              {loading ? (
                <Loader2 className="w-4 h-4 animate-spin" />
              ) : (
                <>
                  <Play className="w-4 h-4 fill-current" />
                  <span>GENERATE PATH</span>
                </>
              )}
            </button>
            
            {plan && (
              <button
                onClick={reset}
                className="w-full bg-slate-800 hover:bg-slate-700 text-slate-400 hover:text-white py-2 rounded-lg text-xs font-mono transition-colors"
              >
                RESET SYSTEM
              </button>
            )}
          </div>
        </div>

        {plan && (
          <div className="bg-slate-900 border border-slate-800 rounded-xl p-5 flex-grow overflow-y-auto">
            <h3 className="text-xs font-bold text-slate-400 uppercase mb-3 flex items-center">
              <Eye className="w-3 h-3 mr-2" /> Analysis
            </h3>
            <p className="text-sm text-slate-300 mb-4 leading-relaxed">{plan.analysis}</p>
            
            <h3 className="text-xs font-bold text-slate-400 uppercase mb-3 flex items-center">
              <Target className="w-3 h-3 mr-2" /> Landmarks Detected
            </h3>
            <div className="space-y-2">
              {plan.landmarks.map((lm, idx) => (
                <div key={idx} className="flex items-center justify-between bg-slate-950 p-2 rounded border border-slate-800">
                  <span className="text-xs font-mono text-emerald-400">{lm.name}</span>
                  <span className={`text-[10px] px-2 py-0.5 rounded-full uppercase font-bold ${
                    lm.type === 'obstacle' ? 'bg-red-900/30 text-red-400' :
                    lm.type === 'target' ? 'bg-emerald-900/30 text-emerald-400' :
                    'bg-blue-900/30 text-blue-400'
                  }`}>
                    {lm.type}
                  </span>
                </div>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Visualization Panel */}
      <div className="lg:col-span-8 bg-black rounded-xl border border-slate-800 relative overflow-hidden flex items-center justify-center group shadow-2xl">
        {!preview && (
          <div className="text-center text-slate-700">
            <MapIcon className="w-20 h-20 mx-auto mb-4 opacity-20" />
            <p className="font-mono text-sm">NO VISUAL DATA SOURCE</p>
          </div>
        )}

        {preview && (
          <div className="relative w-full h-full flex items-center justify-center bg-slate-950">
            {/* The Image Container */}
            <div className="relative w-full h-full max-h-full aspect-auto md:p-4">
               <img 
                 src={preview} 
                 alt="Map" 
                 className="w-full h-full object-contain opacity-80" 
               />
               
               {/* SVG Overlay for Vector Graphics */}
               {plan && (
                 <div className="absolute inset-0 md:m-4 pointer-events-none">
                    {/* Render Landmarks */}
                    {plan.landmarks.map((lm, idx) => (
                      <div
                        key={`lm-${idx}`}
                        className="absolute w-4 h-4 -ml-2 -mt-2 flex items-center justify-center"
                        style={{ left: `${lm.x}%`, top: `${lm.y}%` }}
                      >
                         <div className={`w-3 h-3 rounded-full animate-ping absolute opacity-75 ${
                           lm.type === 'target' ? 'bg-emerald-500' : 
                           lm.type === 'obstacle' ? 'bg-red-500' : 'bg-blue-500'
                         }`} />
                         <div className={`w-2 h-2 rounded-full relative z-10 ${
                           lm.type === 'target' ? 'bg-emerald-400 shadow-[0_0_10px_#34d399]' : 
                           lm.type === 'obstacle' ? 'bg-red-400 shadow-[0_0_10px_#f87171]' : 'bg-blue-400'
                         }`} />
                         {/* Tooltip-like label */}
                         <div className="absolute top-4 left-1/2 -translate-x-1/2 bg-black/80 text-[10px] text-white px-2 py-1 rounded whitespace-nowrap border border-slate-700 backdrop-blur-sm hidden group-hover:block z-20">
                           {lm.name}
                         </div>
                      </div>
                    ))}

                    {/* Render Path Lines */}
                    <svg className="absolute inset-0 w-full h-full overflow-visible">
                      <polyline
                        points={plan.path.map(p => `${p.x}%,${p.y}%`).join(' ')}
                        fill="none"
                        stroke="#10b981"
                        strokeWidth="2"
                        strokeDasharray="4 4"
                        className="opacity-50"
                      />
                      <polyline
                        points={plan.path.slice(0, animationStep + 1).map(p => `${p.x}%,${p.y}%`).join(' ')}
                        fill="none"
                        stroke="#34d399"
                        strokeWidth="3"
                        filter="drop-shadow(0 0 4px #10b981)"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        className="transition-all duration-500 ease-linear"
                      />
                    </svg>
                    
                    {/* Robot Avatar */}
                    {plan.path.length > 0 && (
                      <div 
                        className="absolute w-6 h-6 -ml-3 -mt-3 text-emerald-400 transition-all duration-700 ease-linear z-30"
                        style={{ 
                          left: `${plan.path[Math.min(animationStep, plan.path.length - 1)].x}%`, 
                          top: `${plan.path[Math.min(animationStep, plan.path.length - 1)].y}%` 
                        }}
                      >
                        <div className="absolute inset-0 bg-emerald-500/20 rounded-full animate-pulse scale-150"></div>
                        <Navigation className="w-full h-full fill-current drop-shadow-[0_0_8px_rgba(52,211,153,0.8)]" style={{ transform: 'rotate(45deg)' }} />
                        <div className="absolute -top-6 left-1/2 -translate-x-1/2 bg-emerald-900/90 text-[9px] text-emerald-200 px-2 py-0.5 rounded border border-emerald-500/30 whitespace-nowrap">
                          {plan.path[Math.min(animationStep, plan.path.length - 1)].action}
                        </div>
                      </div>
                    )}
                 </div>
               )}
               
               {/* Scanning Overlay when loading */}
               {loading && (
                 <div className="absolute inset-0 bg-emerald-500/5 animate-pulse z-10">
                   <div className="w-full h-1 bg-emerald-400/50 absolute top-0 animate-scan shadow-[0_0_15px_#34d399]"></div>
                   <div className="absolute bottom-4 right-4 font-mono text-emerald-400 text-xs bg-black/50 px-3 py-1 rounded border border-emerald-500/30">
                     MAPPING TERRAIN...
                   </div>
                 </div>
               )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};