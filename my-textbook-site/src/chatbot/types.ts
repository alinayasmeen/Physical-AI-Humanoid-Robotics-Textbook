// Define types for chatbot components here

export interface Message {
  text: string;
  sender: 'user' | 'bot';
  selectedText?: string;
}

export interface ChatRequest {
  query: string;
  selected_text: string | null;
}

export interface ChatResponse {
  response: string;
}