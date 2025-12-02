import React, { useState } from 'react';
import { Layout } from './components/Layout';
import { IdeaGenerator } from './components/IdeaGenerator';
import { SimulationLab } from './components/SimulationLab';
import { SemanticNavigator } from './components/SemanticNavigator';
import { AppMode } from './types';

function App() {
  const [mode, setMode] = useState<AppMode>(AppMode.IDEATION);

  return (
    <Layout currentMode={mode} onModeChange={setMode}>
      {mode === AppMode.IDEATION && <IdeaGenerator />}
      {mode === AppMode.SIMULATION && <SimulationLab />}
      {mode === AppMode.NAVIGATION && <SemanticNavigator />}
    </Layout>
  );
}

export default App;
