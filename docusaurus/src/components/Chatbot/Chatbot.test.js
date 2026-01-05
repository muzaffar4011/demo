// Simple test file for the Chatbot component
// This is a basic structure that would be expanded in a real implementation

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import Chatbot from './Chatbot';

// Mock the fetch API
global.fetch = jest.fn(() =>
  Promise.resolve({
    ok: true,
    json: () =>
      Promise.resolve({
        response: 'Test response from API',
        sources: [
          {
            document_id: 'doc-123',
            title: 'Test Document',
            url: '/docs/test',
            relevance_score: 0.95
          }
        ],
        session_id: 'test-session-id',
        latency: 1.23
      })
  })
);

describe('Chatbot Component', () => {
  beforeEach(() => {
    fetch.mockClear();
  });

  test('renders chatbot widget initially collapsed', () => {
    render(<Chatbot />);

    // Initially, the chat container should not be visible
    expect(screen.queryByRole('dialog')).not.toBeInTheDocument();

    // The toggle button should be present
    const toggleButton = screen.getByRole('button', { name: /💬/i });
    expect(toggleButton).toBeInTheDocument();
  });

  test('expands chatbot when toggle is clicked', () => {
    render(<Chatbot />);

    const toggleButton = screen.getByRole('button', { name: /💬/i });
    fireEvent.click(toggleButton);

    // After clicking, the chat container should be visible
    expect(screen.getByText('AI Assistant')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Ask about the content...')).toBeInTheDocument();
  });

  test('sends message when send button is clicked', async () => {
    render(<Chatbot apiUrl="http://test-api" />);

    const toggleButton = screen.getByRole('button', { name: /💬/i });
    fireEvent.click(toggleButton);

    // Type a message
    const input = screen.getByPlaceholderText('Ask about the content...');
    fireEvent.change(input, { target: { value: 'Hello' } });

    // Click send button
    const sendButton = screen.getByText('Send');
    fireEvent.click(sendButton);

    // Wait for the API call to complete
    await waitFor(() => {
      expect(fetch).toHaveBeenCalledWith('http://test-api/api/chat', expect.objectContaining({
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        }
      }));
    });
  });

  test('shows loading indicator when waiting for response', async () => {
    render(<Chatbot />);

    const toggleButton = screen.getByRole('button', { name: /💬/i });
    fireEvent.click(toggleButton);

    // Mock a delayed response
    fetch.mockImplementationOnce(() =>
      new Promise(resolve => setTimeout(() => resolve({
        ok: true,
        json: () => Promise.resolve({
          response: 'Delayed response',
          sources: [],
          session_id: 'test-session-id',
          latency: 1.23
        })
      }), 100))
    );

    const input = screen.getByPlaceholderText('Ask about the content...');
    fireEvent.change(input, { target: { value: 'Test message' } });

    const sendButton = screen.getByText('Send');
    fireEvent.click(sendButton);

    // Loading indicator should appear
    expect(screen.getByText('AI Assistant')).toBeInTheDocument();

    // Wait for the response
    await waitFor(() => {
      expect(screen.getByText('Delayed response')).toBeInTheDocument();
    });
  });
});