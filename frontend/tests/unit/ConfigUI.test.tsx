import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import ConfigUI from '../components/ConfigUI';
import * as AIService from '../services/ai_service';

// Mock the AIService
jest.mock('../services/ai_service', () => ({
  configureAPIKey: jest.fn(),
  getConfigStatus: jest.fn(),
}));

describe('ConfigUI Component', () => {
  const mockOnClose = jest.fn();
  const defaultProps = {
    onClose: mockOnClose
  };

  beforeEach(() => {
    jest.clearAllMocks();
    (AIService.getConfigStatus as jest.MockedFunction<any>).mockResolvedValue({
      is_configured: false,
      is_valid: null,
    });
  });

  it('renders correctly with initial state', () => {
    render(<ConfigUI {...defaultProps} />);
    
    expect(screen.getByText('AI Configuration')).toBeInTheDocument();
    expect(screen.getByLabelText('Gemini API Key:')).toBeInTheDocument();
    expect(screen.getByText('Close')).toBeInTheDocument();
  });

  it('closes when close button is clicked', () => {
    render(<ConfigUI {...defaultProps} />);
    
    fireEvent.click(screen.getByText('Close'));
    
    expect(mockOnClose).toHaveBeenCalledTimes(1);
  });

  it('closes when overlay close button is clicked', () => {
    render(<ConfigUI {...defaultProps} />);
    
    fireEvent.click(screen.getByText('×'));
    
    expect(mockOnClose).toHaveBeenCalledTimes(1);
  });

  it('validates and saves API key successfully', async () => {
    const configStatus = {
      is_configured: true,
      is_valid: true,
      message: null
    };
    (AIService.configureAPIKey as jest.MockedFunction<any>).mockResolvedValue({ status: 'success' });
    (AIService.getConfigStatus as jest.MockedFunction<any>).mockResolvedValue(configStatus);
    
    render(<ConfigUI {...defaultProps} />);
    
    // Enter API key
    fireEvent.change(screen.getByLabelText('Gemini API Key:'), {
      target: { value: 'test_api_key' }
    });
    
    // Click Save API Key
    fireEvent.click(screen.getByText('Save API Key'));
    
    await waitFor(() => {
      expect(AIService.configureAPIKey).toHaveBeenCalledWith({ api_key: 'test_api_key' });
      expect(AIService.getConfigStatus).toHaveBeenCalledTimes(2); // Once on mount, once after save
      expect(screen.getByText('API key saved and validated successfully!')).toBeInTheDocument();
    });
  });

  it('shows error for invalid API key', async () => {
    (AIService.configureAPIKey as jest.MockedFunction<any>).mockRejectedValue(new Error('Invalid API key'));
    
    render(<ConfigUI {...defaultProps} />);
    
    // Enter API key
    fireEvent.change(screen.getByLabelText('Gemini API Key:'), {
      target: { value: 'invalid_key' }
    });
    
    // Click Save API Key
    fireEvent.click(screen.getByText('Save API Key'));
    
    await waitFor(() => {
      expect(screen.getByText('Failed to save API key')).toBeInTheDocument();
    });
  });

  it('shows error when API key is empty', async () => {
    render(<ConfigUI {...defaultProps} />);
    
    // Click Save API Key without entering key
    fireEvent.click(screen.getByText('Save API Key'));
    
    await waitFor(() => {
      expect(screen.getByText('API key is required')).toBeInTheDocument();
    });
  });

  it('loads configuration status on mount', async () => {
    const configStatus = {
      is_configured: true,
      is_valid: true,
      message: null
    };
    (AIService.getConfigStatus as jest.MockedFunction<any>).mockResolvedValue(configStatus);
    
    render(<ConfigUI {...defaultProps} />);
    
    await waitFor(() => {
      expect(screen.getByText('API Key Configured: Yes')).toBeInTheDocument();
      expect(screen.getByText('API Key Valid: Yes')).toBeInTheDocument();
    });
  });

  it('shows loading state while saving', async () => {
    // Create a promise that doesn't resolve immediately to simulate loading
    const pendingPromise = new Promise(() => {});
    (AIService.configureAPIKey as jest.MockedFunction<any>).mockReturnValue(pendingPromise as any);
    
    render(<ConfigUI {...defaultProps} />);
    
    // Enter API key
    fireEvent.change(screen.getByLabelText('Gemini API Key:'), {
      target: { value: 'test_api_key' }
    });
    
    // Click Save API Key
    fireEvent.click(screen.getByText('Save API Key'));
    
    // Check that the button shows loading state
    expect(screen.getByText('Saving...')).toBeInTheDocument();
  });
});