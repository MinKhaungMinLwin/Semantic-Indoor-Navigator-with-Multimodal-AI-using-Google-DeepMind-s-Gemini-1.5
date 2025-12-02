import React, { useState, useRef } from 'react';
import { simulateRobotTask } from '../services/gemini';
import { SimulationResult } from '../types';
import { 
  Camera, Play, RotateCcw, Code, List, Terminal, 
  Upload, CheckCircle, Clock, Zap, AlertTriangle 
} from 'lucide-react';
import { BarChart, Bar, XAxis, YAxis, Tooltip, ResponsiveContainer, Cell } from 'recharts';

export const SimulationLab: React.FC = () => {
  const [image, setImage] = useState<File | null>(null);
  const [preview, setPreview] = useState<string | null>(null);
  const [instruction, setInstruction] = useState('');
  const [result, setResult] = useState<SimulationResult | null>(null);
  const [loading, setLoading] = useState(false);
  const [activeTab, setActiveTab] = useState<'plan' | 'code'>('plan');
  
  const fileInputRef = useRef<HTMLInputElement>(null);

  const handleImageUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (e.target.files && e.target.files[0]) {
      const file = e.target.files[0];
      setImage(file);
      const reader = new FileReader();
      reader.onloadend = () => setPreview(reader.result as string);
      reader.readAsDataURL(file);
    }
  };

  const handleSimulate = async () => {
    if (!instruction) return;
    setLoading(true);
    setResult(null);
    try {
      const res = await simulateRobotTask(image, instruction);
      setResult(res);
    } catch (e) {
      console.error(e);
      alert('Simulation failed. Check API key and console.');
    } finally {
      setLoading(false);
    }
  };

  const clearAll = () => {
    setImage(null);
    setPreview(null);
    setInstruction('');
    setResult(null);
  };

  return (
    <div className="grid grid-cols-1 lg:grid-cols-12 gap-6 h-[calc(100vh-140px)]">
      
      {/* LEFT COLUMN: Controls & Input */}
      <div className="lg:col-span-4 flex flex-col gap-6 h-full overflow-y-auto pr-2">
        {/* Visual Input Feed */}
        <div className="bg-slate-900 border border-slate-800 rounded-xl p-1 overflow-hidden relative group shadow-2xl">
           <div className="absolute top-3 left-3 z-10 bg-black/60 backdrop-blur-sm px-2 py-1 rounded text-xs font-mono text-blue-400 border border-blue-500/30 flex items-center">
             <Camera className="w-3 h-3 mr-2" /> VISION FEED_01
           </div>
           
           <div 
             onClick={() => fileInputRef.current?.click()}
             className="w-full aspect-video bg-black rounded-lg border border-slate-800 flex flex-col items-center justify-center cursor-pointer hover:bg-slate-950 transition-colors relative overflow-hidden"
           >
             {preview ? (
               <img src={preview} alt="Robot vision" className="w-full h-full object-cover" />
             ) : (
               <div className="text-center text-slate-600">
                 <Upload className="w-8 h-8 mx-auto mb-2 opacity-50" />
                 <p className="text-sm font-mono">UPLOAD SOURCE IMAGE</p>
                 <p className="text-xs opacity-50 mt-1">Simulate Robot Eye View</p>
               </div>
             )}
             
             {/* Scanning effect overlay */}
             {loading && (
               <div className="absolute inset-0 bg-gradient-to-b from-transparent via-blue-500/10 to-transparent w-full h-full animate-scan" style={{backgroundSize: '100% 20%'}}></div>
             )}
           </div>
           <input 
             type="file" 
             ref={fileInputRef} 
             className="hidden" 
             accept="image/*" 
             onChange={handleImageUpload}
           />
        </div>

        {/* Command Center */}
        <div className="bg-slate-900 border border-slate-800 rounded-xl p-6 shadow-xl flex flex-col flex-grow">
          <h3 className="text-sm font-bold text-slate-400 mb-4 uppercase tracking-wider flex items-center">
            <Terminal className="w-4 h-4 mr-2" /> Operator Command
          </h3>
          
          <textarea
            value={instruction}
            onChange={(e) => setInstruction(e.target.value)}
            placeholder="Enter natural language command (e.g., 'Locate the red cube and place it in the bin')..."
            className="w-full flex-grow bg-slate-950 border border-slate-700 rounded-lg p-4 font-mono text-sm text-green-400 placeholder-slate-700 focus:ring-1 focus:ring-green-500 outline-none resize-none mb-4"
          />

          <div className="flex gap-3">
            <button
              onClick={handleSimulate}
              disabled={loading || !instruction}
              className="flex-1 bg-blue-600 hover:bg-blue-500 text-white font-bold py-3 rounded-lg shadow-lg shadow-blue-900/20 transition-all flex items-center justify-center space-x-2 disabled:opacity-50 disabled:grayscale"
            >
              {loading ? <div className="animate-spin w-4 h-4 border-2 border-white border-t-transparent rounded-full" /> : <Play className="w-4 h-4" />}
              <span>EXECUTE PLAN</span>
            </button>
            <button
              onClick={clearAll}
              className="px-4 bg-slate-800 hover:bg-slate-700 text-slate-300 rounded-lg transition-colors border border-slate-700"
            >
              <RotateCcw className="w-4 h-4" />
            </button>
          </div>
        </div>
      </div>

      {/* RIGHT COLUMN: Output & Visualization */}
      <div className="lg:col-span-8 bg-slate-900 border border-slate-800 rounded-xl overflow-hidden flex flex-col shadow-2xl h-full">
        {/* Output Header */}
        <div className="border-b border-slate-800 bg-slate-950/50 p-2 flex items-center justify-between">
          <div className="flex space-x-1">
            <button
              onClick={() => setActiveTab('plan')}
              className={`px-4 py-2 text-sm font-medium rounded-md transition-colors flex items-center ${activeTab === 'plan' ? 'bg-slate-800 text-blue-400' : 'text-slate-500 hover:text-slate-300'}`}
            >
              <List className="w-4 h-4 mr-2" /> Logic Chain
            </button>
            <button
              onClick={() => setActiveTab('code')}
              className={`px-4 py-2 text-sm font-medium rounded-md transition-colors flex items-center ${activeTab === 'code' ? 'bg-slate-800 text-purple-400' : 'text-slate-500 hover:text-slate-300'}`}
            >
              <Code className="w-4 h-4 mr-2" /> Generated Code
            </button>
          </div>
          {result && (
            <div className="flex items-center space-x-4 px-4">
              <span className="text-xs text-slate-500 font-mono flex items-center">
                <Clock className="w-3 h-3 mr-1" /> {result.steps.reduce((acc, s) => acc + s.estimatedDuration, 0)}s EST
              </span>
              <span className="text-xs text-green-500 font-mono flex items-center">
                <CheckCircle className="w-3 h-3 mr-1" /> VALID
              </span>
            </div>
          )}
        </div>

        {/* Content Area */}
        <div className="flex-grow overflow-y-auto p-0 relative bg-slate-950/30">
          {!result && !loading && (
            <div className="absolute inset-0 flex flex-col items-center justify-center text-slate-700">
              <Zap className="w-16 h-16 mb-4 opacity-10" />
              <p className="font-mono text-sm">AWAITING INPUT STREAM</p>
            </div>
          )}

          {loading && (
            <div className="absolute inset-0 flex flex-col items-center justify-center bg-slate-950/80 z-20 backdrop-blur-sm">
              <div className="w-64 bg-slate-800 rounded-full h-1 overflow-hidden mb-4">
                <div className="h-full bg-blue-500 animate-loading-bar"></div>
              </div>
              <p className="font-mono text-blue-400 text-xs animate-pulse">ANALYZING SPATIAL DATA...</p>
              <p className="font-mono text-slate-500 text-[10px] mt-2">GENERATING KINEMATIC SOLUTION</p>
            </div>
          )}

          {result && activeTab === 'plan' && (
            <div className="p-6 space-y-6">
              <div className="bg-slate-800/50 border border-slate-700/50 p-4 rounded-lg">
                <h4 className="text-sm font-bold text-slate-300 mb-2">MISSION SUMMARY</h4>
                <p className="text-slate-400 text-sm leading-relaxed">{result.summary}</p>
              </div>

              <div className="space-y-4">
                 {result.steps.map((step, idx) => (
                   <div key={idx} className="relative flex items-start group">
                     {/* Connector Line */}
                     {idx !== result.steps.length - 1 && (
                       <div className="absolute left-[19px] top-10 bottom-[-24px] w-[2px] bg-slate-800 group-hover:bg-blue-900/50 transition-colors"></div>
                     )}
                     
                     <div className="w-10 h-10 rounded-full bg-slate-900 border-2 border-slate-700 flex items-center justify-center text-xs font-mono text-slate-500 flex-shrink-0 z-10 group-hover:border-blue-500 group-hover:text-blue-400 transition-colors shadow-lg">
                       {idx + 1}
                     </div>
                     
                     <div className="ml-4 bg-slate-900 border border-slate-800 p-4 rounded-lg flex-grow hover:border-slate-600 transition-all shadow-md">
                       <div className="flex justify-between items-start mb-2">
                         <span className="inline-block px-2 py-1 bg-blue-900/20 text-blue-400 text-[10px] font-bold tracking-wider rounded border border-blue-900/30">
                           {step.action.toUpperCase()}
                         </span>
                         <span className="text-xs font-mono text-slate-600">{step.estimatedDuration}s</span>
                       </div>
                       <h5 className="text-slate-200 font-medium text-sm mb-1">Target: <span className="text-indigo-400">{step.target}</span></h5>
                       <p className="text-xs text-slate-500 mb-3">{step.reasoning}</p>
                       <div className="bg-black/40 p-2 rounded border border-slate-800/50 font-mono text-[10px] text-green-400/80 truncate">
                         {step.codeSnippet}
                       </div>
                     </div>
                   </div>
                 ))}
              </div>

              {/* Simple Chart for time distribution */}
              <div className="h-48 mt-8 bg-slate-900/50 rounded-xl p-4 border border-slate-800">
                <h4 className="text-xs font-bold text-slate-500 mb-4 uppercase">Task Duration Analysis</h4>
                <ResponsiveContainer width="100%" height="100%">
                  <BarChart data={result.steps}>
                    <XAxis dataKey="action" tick={{fontSize: 10, fill: '#64748b'}} interval={0} />
                    <YAxis hide />
                    <Tooltip 
                      contentStyle={{backgroundColor: '#0f172a', borderColor: '#334155', color: '#f1f5f9'}}
                      itemStyle={{color: '#94a3b8'}}
                      cursor={{fill: '#1e293b'}}
                    />
                    <Bar dataKey="estimatedDuration" radius={[4, 4, 0, 0]}>
                      {result.steps.map((entry, index) => (
                        <Cell key={`cell-${index}`} fill={index % 2 === 0 ? '#3b82f6' : '#6366f1'} />
                      ))}
                    </Bar>
                  </BarChart>
                </ResponsiveContainer>
              </div>
            </div>
          )}

          {result && activeTab === 'code' && (
            <div className="p-0 h-full">
              <div className="bg-[#0d1117] h-full p-6 overflow-auto font-mono text-sm leading-6">
                <pre className="text-slate-300">
                  <code>{result.fullCode}</code>
                </pre>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};
