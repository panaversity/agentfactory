import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import AIDialog from '../components/AIDialog';
import * as AIService from '../services/ai_service';

// Mock the AIService
jest.mock('../services/ai_service', () => ({
  queryAI: jest.fn(),
  getConfigStatus: jest.fn(),
}));

describe('AIDialog Component', () => {
  const mockOnClose = jest.fn();
  const defaultProps = {
    initialText: 'Sample highlighted text',
    onClose: mockOnClose,
    position: { x: 100, y: 100 }
  };

  beforeEach(() => {
    jest.clearAllMocks();
    (AIService.getConfigStatus as jest.MockedFunction<any>).mockResolvedValue({
      is_configured: true,
      is_valid: true,
    });
  });

  it('renders correctly with initial text', () => {
    render(<AIDialog {...defaultProps} />);
    
    expect(screen.getByText('AI Assistant')).toBeInTheDocument();
    expect(screen.getByText('Sample highlighted text')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Ask the AI about the highlighted text...')).toBeInTheDocument();
  });

  it('closes when close button is clicked', () => {
    render(<AIDialog {...defaultProps} />);
    
    fireEvent.click(screen.getByText('×'));
    
    expect(mockOnClose).toHaveBeenCalledTimes(1);
  });

  it('submits query when form is submitted', async () => {
    const mockResponse = { response: 'Test AI response' };
    (AIService.queryAI as jest.MockedFunction<any>).mockResolvedValue(mockResponse);
    
    render(<AIDialog {...defaultProps} />);
    
    // Fill in query
    fireEvent.change(screen.getByPlaceholderText('Ask the AI about the highlighted text...'), {
      target: { value: 'What does this mean?' }
    });
    
    // Submit form
    fireEvent.click(screen.getByText('Ask AI'));
    
    await waitFor(() => {
      expect(AIService.queryAI).toHaveBeenCalledWith({
        highlighted_text: 'Sample highlighted text',
        query: 'What does this mean?'
      });
      expect(screen.getByText('AI Response:')).toBeInTheDocument();
      expect(screen.getByText('Test AI response')).toBeInTheDocument();
    });
  });

  it('shows error when query is submitted without text', async () => {
    render(<AIDialog {...defaultProps} />);
    
    // Submit empty query
    fireEvent.click(screen.getByText('Ask AI'));
    
    await waitFor(() => {
      expect(screen.getByText('Please enter a query')).toBeInTheDocument();
    });
  });

  it('shows error when AI is not configured', async () => {
    (AIService.getConfigStatus as jest.MockedFunction<any>).mockResolvedValue({
      is_configured: false,
    });
    
    render(<AIDialog {...defaultProps} />);
    
    // Wait for config status to be checked
    await waitFor(() => {
      expect(screen.getByText('⚠️ AI is not configured. Please set up your API key first.')).toBeInTheDocument();
    });
  });

  it('disables submit button when not configured', async () => {
    (AIService.getConfigStatus as jest.MockedFunction<any>).mockResolvedValue({
      is_configured: false,
    });
    
    render(<AIDialog {...defaultProps} />);
    
    // Wait for config status to be checked
    await waitFor(() => {
      const submitButton = screen.getByText('Ask AI');
      expect(submitButton).toBeDisabled();
    });
  });

  it('shows error when AI service fails', async () => {
    (AIService.queryAI as jest.MockedFunction<any>).mockRejectedValue(new Error('API Error'));
    
    render(<AIDialog {...defaultProps} />);
    
    // Fill in query
    fireEvent.change(screen.getByPlaceholderText('Ask the AI about the highlighted text...'), {
      target: { value: 'Test query' }
    });
    
    // Submit form
    fireEvent.click(screen.getByText('Ask AI'));
    
    await waitFor(() => {
      expect(screen.getByText('Error getting AI response')).toBeInTheDocument();
    });
  });
});