export enum AppMode {
  IDEATION = 'IDEATION',
  SIMULATION = 'SIMULATION',
  NAVIGATION = 'NAVIGATION',
}

export interface RobotPlanStep {
  id: string;
  action: string;
  target: string;
  reasoning: string;
  codeSnippet: string;
  estimatedDuration: number;
}

export interface SimulationResult {
  summary: string;
  steps: RobotPlanStep[];
  fullCode: string;
}

export interface ProjectIdea {
  title: string;
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
  description: string;
  hardware: string[];
  software: string[];
}

export interface ChatMessage {
  role: 'user' | 'model';
  text: string;
  timestamp: number;
}

// Navigation specific types
export interface Landmark {
  name: string;
  x: number; // Percentage 0-100
  y: number; // Percentage 0-100
  type: 'obstacle' | 'target' | 'reference' | 'start';
  description: string;
}

export interface NavWaypoint {
  id: number;
  x: number; // Percentage 0-100
  y: number; // Percentage 0-100
  action: string;
}

export interface NavigationPlan {
  analysis: string;
  landmarks: Landmark[];
  path: NavWaypoint[];
}
