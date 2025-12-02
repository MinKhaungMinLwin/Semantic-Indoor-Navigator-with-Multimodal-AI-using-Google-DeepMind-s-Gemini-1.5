import React, { useState } from 'react';
import { ProjectIdea } from '../types';
import { generateProjectIdeas } from '../services/gemini';
import { Sparkles, Wrench, HardDrive, Code, Loader2, ArrowRight } from 'lucide-react';

export const IdeaGenerator: React.FC = () => {
  const [interests, setInterests] = useState('');
  const [equipment, setEquipment] = useState('');
  const [ideas, setIdeas] = useState<ProjectIdea[]>([]);
  const [loading, setLoading] = useState(false);

  const handleGenerate = async () => {
    setLoading(true);
    try {
      const result = await generateProjectIdeas(interests, equipment);
      setIdeas(result);
    } catch (e) {
      console.error(e);
      alert("Failed to generate ideas. Please check console.");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="space-y-8 animate-in fade-in slide-in-from-bottom-4 duration-500">
      <div className="text-center space-y-4 mb-12">
        <h2 className="text-3xl font-bold text-white">What can I build with Gemini Robotics?</h2>
        <p className="text-slate-400 max-w-2xl mx-auto">
          Gemini 1.5's massive context window and multimodal capabilities enable robots to understand complex environments, follow vague instructions, and reason about the physical world. Tell us what gear you have, and we'll design a mission.
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        {/* Input Panel */}
        <div className="lg:col-span-1 space-y-6">
          <div className="bg-slate-900 rounded-xl p-6 border border-slate-800 shadow-xl">
            <h3 className="text-lg font-semibold text-white mb-4 flex items-center">
              <Wrench className="w-5 h-5 mr-2 text-blue-400" /> Parameters
            </h3>
            
            <div className="space-y-4">
              <div>
                <label className="block text-sm text-slate-400 mb-1">Your Interests</label>
                <textarea 
                  className="w-full bg-slate-800 border border-slate-700 rounded-lg p-3 text-sm text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent outline-none resize-none h-24"
                  placeholder="e.g., Home automation, sorting LEGOs, drone navigation..."
                  value={interests}
                  onChange={(e) => setInterests(e.target.value)}
                />
              </div>
              
              <div>
                <label className="block text-sm text-slate-400 mb-1">Available Hardware</label>
                <textarea 
                  className="w-full bg-slate-800 border border-slate-700 rounded-lg p-3 text-sm text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent outline-none resize-none h-24"
                  placeholder="e.g., Raspberry Pi 4, Logitech Webcam, 2 Servo Motors..."
                  value={equipment}
                  onChange={(e) => setEquipment(e.target.value)}
                />
              </div>

              <button
                onClick={handleGenerate}
                disabled={loading}
                className="w-full bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 text-white font-bold py-3 rounded-lg shadow-lg shadow-blue-900/20 transition-all active:scale-95 disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center space-x-2"
              >
                {loading ? (
                  <Loader2 className="w-5 h-5 animate-spin" />
                ) : (
                  <>
                    <Sparkles className="w-5 h-5" />
                    <span>Generate Projects</span>
                  </>
                )}
              </button>
            </div>
          </div>
        </div>

        {/* Results Panel */}
        <div className="lg:col-span-2">
          {ideas.length === 0 && !loading && (
            <div className="h-full flex flex-col items-center justify-center border-2 border-dashed border-slate-800 rounded-xl p-12 text-slate-600">
              <Sparkles className="w-16 h-16 mb-4 opacity-20" />
              <p>Configure parameters to receive mission directives.</p>
            </div>
          )}

          <div className="space-y-6">
            {ideas.map((idea, idx) => (
              <div 
                key={idx} 
                className="bg-slate-900 rounded-xl border border-slate-800 overflow-hidden hover:border-blue-500/50 transition-all duration-300 group"
              >
                <div className="p-6">
                  <div className="flex justify-between items-start mb-4">
                    <h3 className="text-xl font-bold text-white group-hover:text-blue-400 transition-colors">
                      {idea.title}
                    </h3>
                    <span className={`px-3 py-1 rounded-full text-xs font-bold border ${
                      idea.difficulty === 'Advanced' ? 'bg-red-900/20 border-red-800 text-red-400' :
                      idea.difficulty === 'Intermediate' ? 'bg-yellow-900/20 border-yellow-800 text-yellow-400' :
                      'bg-green-900/20 border-green-800 text-green-400'
                    }`}>
                      {idea.difficulty}
                    </span>
                  </div>
                  
                  <p className="text-slate-300 mb-6 leading-relaxed">{idea.description}</p>
                  
                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div className="bg-slate-950/50 p-4 rounded-lg">
                      <h4 className="text-xs font-bold text-slate-500 uppercase tracking-wider mb-2 flex items-center">
                        <HardDrive className="w-3 h-3 mr-2" /> Hardware
                      </h4>
                      <div className="flex flex-wrap gap-2">
                        {idea.hardware.map((hw, i) => (
                          <span key={i} className="text-xs bg-slate-800 text-slate-300 px-2 py-1 rounded border border-slate-700">
                            {hw}
                          </span>
                        ))}
                      </div>
                    </div>
                    
                    <div className="bg-slate-950/50 p-4 rounded-lg">
                      <h4 className="text-xs font-bold text-slate-500 uppercase tracking-wider mb-2 flex items-center">
                        <Code className="w-3 h-3 mr-2" /> Software Stack
                      </h4>
                      <div className="flex flex-wrap gap-2">
                        {idea.software.map((sw, i) => (
                          <span key={i} className="text-xs bg-slate-800 text-blue-300 px-2 py-1 rounded border border-slate-700">
                            {sw}
                          </span>
                        ))}
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};
