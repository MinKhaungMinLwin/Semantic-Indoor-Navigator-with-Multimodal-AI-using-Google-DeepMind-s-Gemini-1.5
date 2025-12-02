import { GoogleGenAI, Type, Schema } from "@google/genai";
import { ProjectIdea, RobotPlanStep, SimulationResult, NavigationPlan } from "../types";

const apiKey = process.env.API_KEY || '';
const ai = new GoogleGenAI({ apiKey });

// Helper to convert blob to base64
export const blobToBase64 = (blob: Blob): Promise<string> => {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.onloadend = () => {
      const base64String = reader.result as string;
      // Remove data url prefix (e.g. "data:image/jpeg;base64,")
      const base64Data = base64String.split(',')[1];
      resolve(base64Data);
    };
    reader.onerror = reject;
    reader.readAsDataURL(blob);
  });
};

export const generateProjectIdeas = async (interests: string, equipment: string): Promise<ProjectIdea[]> => {
  try {
    const model = 'gemini-2.5-flash';
    
    const responseSchema: Schema = {
      type: Type.ARRAY,
      items: {
        type: Type.OBJECT,
        properties: {
          title: { type: Type.STRING },
          difficulty: { type: Type.STRING, enum: ['Beginner', 'Intermediate', 'Advanced'] },
          description: { type: Type.STRING },
          hardware: { type: Type.ARRAY, items: { type: Type.STRING } },
          software: { type: Type.ARRAY, items: { type: Type.STRING } }
        },
        required: ['title', 'difficulty', 'description', 'hardware', 'software']
      }
    };

    const prompt = `
      The user is asking "What kind of project can I do with Gemini Robotics capabilities?".
      Generate 3 distinct, creative, and feasible robotics project ideas that utilize Multimodal LLMs (Gemini).
      
      User's Interests/Context: ${interests || "General robotics, home automation, or computer vision"}
      User's Equipment: ${equipment || "Standard laptop, maybe a webcam, Raspberry Pi"}
      
      Focus on projects where Gemini analyzes images/video/audio to control hardware.
    `;

    const result = await ai.models.generateContent({
      model,
      contents: prompt,
      config: {
        responseMimeType: 'application/json',
        responseSchema: responseSchema,
        systemInstruction: "You are a creative robotics engineer mentor. Suggest practical yet impressive projects.",
      }
    });

    if (result.text) {
      return JSON.parse(result.text) as ProjectIdea[];
    }
    return [];
  } catch (error) {
    console.error("Error generating ideas:", error);
    throw error;
  }
};

export const simulateRobotTask = async (imageBlob: Blob | null, instruction: string): Promise<SimulationResult> => {
  try {
    const model = 'gemini-2.5-flash'; // Good balance for this task

    const responseSchema: Schema = {
      type: Type.OBJECT,
      properties: {
        summary: { type: Type.STRING, description: "High level summary of the plan" },
        steps: {
          type: Type.ARRAY,
          items: {
            type: Type.OBJECT,
            properties: {
              id: { type: Type.STRING },
              action: { type: Type.STRING, description: "The primitive action (e.g., MOVE, GRIP)" },
              target: { type: Type.STRING, description: "The object or location interacting with" },
              reasoning: { type: Type.STRING, description: "Why this step is needed based on the image" },
              codeSnippet: { type: Type.STRING, description: "A single line of Python code for this step" },
              estimatedDuration: { type: Type.NUMBER, description: "Estimated seconds" }
            },
            required: ['id', 'action', 'target', 'reasoning', 'codeSnippet', 'estimatedDuration']
          }
        },
        fullCode: { type: Type.STRING, description: "Complete executable Python script using a mock robot library" }
      },
      required: ['summary', 'steps', 'fullCode']
    };

    const parts: any[] = [{ text: `Instruction: ${instruction}` }];
    
    if (imageBlob) {
      const base64Data = await blobToBase64(imageBlob);
      parts.push({
        inlineData: {
          mimeType: imageBlob.type,
          data: base64Data
        }
      });
    }

    const result = await ai.models.generateContent({
      model,
      contents: { parts },
      config: {
        responseMimeType: 'application/json',
        responseSchema: responseSchema,
        systemInstruction: `
          You are the 'Cortex' of an advanced robot. 
          Analyze the visual input (if provided) and the text instruction. 
          Break down the task into robotic primitives (Move, Grasp, Inspect, Release, Navigate).
          Generate a structured plan and a full Python script using a fictional library 'gemini_robotics'.
          Assume the robot has a manipulator arm and a mobile base.
          If no image is provided, hallucinate a plausible scenario based on the instruction.
        `
      }
    });

    if (result.text) {
      return JSON.parse(result.text) as SimulationResult;
    }
    throw new Error("No response generated");

  } catch (error) {
    console.error("Error simulating task:", error);
    throw error;
  }
};

export const generateNavigationPlan = async (imageBlob: Blob, instruction: string): Promise<NavigationPlan> => {
  try {
    const model = 'gemini-2.5-flash';
    const base64Data = await blobToBase64(imageBlob);

    const responseSchema: Schema = {
      type: Type.OBJECT,
      properties: {
        analysis: { type: Type.STRING, description: "Semantic understanding of the environment and task" },
        landmarks: {
          type: Type.ARRAY,
          items: {
            type: Type.OBJECT,
            properties: {
              name: { type: Type.STRING },
              x: { type: Type.NUMBER, description: "X coordinate percentage (0-100) from left" },
              y: { type: Type.NUMBER, description: "Y coordinate percentage (0-100) from top" },
              type: { type: Type.STRING, enum: ['obstacle', 'target', 'reference', 'start'] },
              description: { type: Type.STRING }
            },
            required: ['name', 'x', 'y', 'type', 'description']
          }
        },
        path: {
          type: Type.ARRAY,
          items: {
            type: Type.OBJECT,
            properties: {
              id: { type: Type.INTEGER },
              x: { type: Type.NUMBER, description: "X coordinate percentage (0-100)" },
              y: { type: Type.NUMBER, description: "Y coordinate percentage (0-100)" },
              action: { type: Type.STRING, description: "Action at this waypoint (e.g. 'Turn Left', 'Arrive')" }
            },
            required: ['id', 'x', 'y', 'action']
          }
        }
      },
      required: ['analysis', 'landmarks', 'path']
    };

    const result = await ai.models.generateContent({
      model,
      contents: {
        parts: [
          { inlineData: { mimeType: imageBlob.type, data: base64Data } },
          { text: `
            Analyze this indoor environment image (map or perspective view) for robot navigation.
            Instruction: "${instruction}"
            
            1. Identify key landmarks, obstacles, and the target location mentioned or implied.
            2. Determine a logical starting point (e.g., a doorway or bottom center if not specified).
            3. Plan a path of waypoints (x,y in 0-100% coordinates) to the target, avoiding obstacles.
            4. If the image is a perspective view, approximate the ground plane coordinates.
          ` }
        ]
      },
      config: {
        responseMimeType: 'application/json',
        responseSchema: responseSchema,
        systemInstruction: "You are an advanced Semantic Navigation AI. You interpret visual scenes for autonomous robots. Precision in coordinate estimation (0-100 scale) is critical."
      }
    });

    if (result.text) {
      return JSON.parse(result.text) as NavigationPlan;
    }
    throw new Error("No navigation plan generated");
  } catch (error) {
    console.error("Navigation Error:", error);
    throw error;
  }
};
