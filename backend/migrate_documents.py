"""
Script to migrate all documentation files from docusaurus/docs into Qdrant.
This script will:
1. Find all .md and .mdx files in the docusaurus/docs directory
2. Parse and chunk them
3. Generate embeddings
4. Store them in Qdrant
"""
import asyncio
import sys
from pathlib import Path

# Add the backend directory to the path so we can import app modules
sys.path.insert(0, str(Path(__file__).parent))

from app.services.document_service import DocumentService
from app.core.config import settings


async def migrate_documents():
    """
    Migrate all documents from docusaurus/docs to Qdrant
    """
    print("🚀 Starting document migration to Qdrant...")
    print(f"📁 Collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"🔗 Qdrant URL: {settings.QDRANT_URL}")
    print()
    
    # Get the path to docusaurus/docs directory
    # This script is in backend/, so docusaurus is at ../docusaurus/docs
    backend_dir = Path(__file__).parent
    project_root = backend_dir.parent
    docs_path = project_root / "docusaurus" / "docs"
    
    if not docs_path.exists():
        print(f"❌ Error: Documentation directory not found at: {docs_path}")
        print("   Please make sure the docusaurus/docs directory exists.")
        return
    
    print(f"📂 Source directory: {docs_path}")
    print()
    
    try:
        print("🔌 Connecting to Qdrant and initializing services...")
        document_service = DocumentService()
        print("✅ Services initialized")
        print()
        
        # Ingest all documents
        print("📚 Starting document ingestion...")
        print("   This may take several minutes depending on the number of documents.")
        print("   Generating embeddings and uploading to Qdrant...")
        print()
        
        result = await document_service.ingest_documents(
            source=str(docs_path),
            recursive=True,
            filters=[".md", ".mdx"],
            chunk_size=1000,
            chunk_overlap=200
        )
        
        print()
        print("✅ Migration completed successfully!")
        print(f"   📄 Documents processed: {result['documents_processed']}")
        print(f"   📦 Chunks created: {result['chunks_created']}")
        print()
        print("🎉 Your documents are now available in Qdrant!")
        print("   You can now use the chat API to ask questions about your content.")
        
    except Exception as e:
        print()
        print(f"❌ Error during migration: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(migrate_documents())

