"""
Script to re-ingest documents into Qdrant with corrected URLs and titles.
This script will delete the existing collection and re-create it with proper metadata.
"""
import asyncio
import sys
from pathlib import Path
from app.services.document_service import DocumentService
from app.core.database import get_qdrant_client
from app.core.config import settings


async def main():
    print("Re-ingesting documents with corrected URLs and titles...")
    print()

    # Initialize services
    document_service = DocumentService()
    qdrant_client = get_qdrant_client()

    # Delete existing collection if it exists
    try:
        collections = await qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if settings.QDRANT_COLLECTION_NAME in collection_names:
            print(f"Deleting existing collection: {settings.QDRANT_COLLECTION_NAME}")
            await qdrant_client.delete_collection(settings.QDRANT_COLLECTION_NAME)
            print("Collection deleted successfully")
            print()
    except Exception as e:
        print(f"Warning: Could not delete collection: {e}")
        print()

    # Determine the docs path
    # Assuming script is run from backend directory and docs are in ../docusaurus/docs
    backend_path = Path(__file__).parent
    docs_path = backend_path.parent / "docusaurus" / "docs"

    if not docs_path.exists():
        print(f"Error: Docs directory not found at {docs_path}")
        print("Please provide the correct path to your documentation.")
        sys.exit(1)

    print(f"Source directory: {docs_path}")
    print()

    # Ingest documents
    try:
        result = await document_service.ingest_documents(
            source=str(docs_path),
            recursive=True,
            filters=[".md", ".mdx"],
            chunk_size=1000,
            chunk_overlap=200
        )

        print()
        print("=" * 60)
        print("INGESTION COMPLETED SUCCESSFULLY!")
        print("=" * 60)
        print(f"Documents processed: {result['documents_processed']}")
        print(f"Chunks created: {result['chunks_created']}")
        print()
        print("Your chatbot will now provide accurate source citations!")
        print()

    except Exception as e:
        print()
        print("=" * 60)
        print("INGESTION FAILED")
        print("=" * 60)
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
