// This file provides functionality for capturing selected text on the page
// It's included as a script to ensure compatibility with Docusaurus

(function() {
  // Store the currently selected text
  let currentSelectedText = '';

  // Function to get selected text
  function getSelectedText() {
    return window.getSelection ? window.getSelection().toString().trim() : '';
  }

  // Event listener for text selection
  document.addEventListener('mouseup', function() {
    currentSelectedText = getSelectedText();

    // Dispatch a custom event so the chatbot can react to text selection
    const event = new CustomEvent('textSelected', {
      detail: { text: currentSelectedText }
    });
    document.dispatchEvent(event);
  });

  // Store the selected text in a global variable for access by the chatbot
  window.ChatbotContext = {
    getSelectedText: function() {
      return currentSelectedText;
    },

    // Clear the selected text
    clearSelectedText: function() {
      currentSelectedText = '';
    }
  };
})();