import React, {type ReactNode} from 'react';
import type {MermaidProps} from 'mdx-mermaid/lib/types';

// A wrapper component for diagrams that can include Mermaid diagrams or other visual elements
type DiagramProps = {
  children: ReactNode;
  title?: string;
  description?: string;
  className?: string;
};

export default function Diagram({ children, title, description, className = '' }: DiagramProps): ReactNode {
  return (
    <div className={`diagram-container ${className}`}>
      {title && <h4>{title}</h4>}
      <div>{children}</div>
      {description && <p className="text--center margin-top--sm"><em>{description}</em></p>}
    </div>
  );
}