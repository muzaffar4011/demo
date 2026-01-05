import React, {type ReactNode} from 'react';

type CodeBlockProps = {
  children: ReactNode;
  explanation?: string;
  language?: string;
  expectedOutput?: string;
  className?: string;
};

export default function CodeBlock({ children, explanation, expectedOutput, language, className = '' }: CodeBlockProps): ReactNode {
  return (
    <div className={`code-explanation ${className}`}>
      <div className="code-block-content">
        {children}
      </div>
      {explanation && (
        <div className="margin-top--md">
          <strong>Explanation:</strong> {explanation}
        </div>
      )}
      {expectedOutput && (
        <div className="margin-top--sm">
          <strong>Expected Output:</strong>
          <pre className="language-text">
            <code>{expectedOutput}</code>
          </pre>
        </div>
      )}
    </div>
  );
}