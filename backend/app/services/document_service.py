import asyncio
import os
from typing import List, Dict, Any
from pathlib import Path
import hashlib
from app.core.config import settings
from app.core.database import get_qdrant_client
from app.services.embedding_service import EmbeddingService
from app.utils.document_parser import DocumentParser
from qdrant_client.http import models
import uuid


class DocumentService:
    def __init__(self):
        self.parser = DocumentParser()
        self.embedding_service = EmbeddingService()
        self.qdrant_client = get_qdrant_client()

    async def ingest_documents(
        self,
        source: str,
        recursive: bool = True,
        filters: List[str] = None,
        chunk_size: int = 1000,
        chunk_overlap: int = 200
    ) -> Dict[str, Any]:
        """
        Process documents from the source directory, chunk them, generate embeddings, and store in vector DB
        """
        if filters is None:
            filters = [".md", ".mdx"]

        # Ensure the collection exists before ingesting
        await self.create_collection_if_not_exists(vector_size=1536)  # text-embedding-ada-002 has 1536 dimensions

        # Get all document files from the source
        source_path = Path(source).resolve()  # Resolve to absolute path
        if not source_path.exists():
            raise ValueError(f"Source path does not exist: {source}")

        # Find all files matching the filters
        all_files = []
        if recursive:
            for ext in filters:
                all_files.extend(source_path.rglob(f"*{ext}"))
        else:
            for ext in filters:
                all_files.extend(source_path.glob(f"*{ext}"))

        documents_processed = 0
        chunks_created = 0
        total_files = len(all_files)

        print(f"Found {total_files} files to process")
        print()

        for idx, file_path in enumerate(all_files, 1):
            try:
                print(f"[{idx}/{total_files}] Processing: {file_path.name}...", end=" ", flush=True)
                
                # Parse the document
                content = self.parser.parse_document(file_path)

                # Extract title from frontmatter or use filename
                frontmatter = self.parser.extract_frontmatter(file_path)
                if frontmatter:
                    # Try to get title from frontmatter (various possible fields)
                    title = (frontmatter.get('title') or
                            frontmatter.get('sidebar_label') or
                            frontmatter.get('id') or
                            file_path.stem)
                else:
                    title = file_path.stem

                # Create document metadata
                doc_id = str(uuid.uuid4())
                # Get relative path from the source directory
                try:
                    source_path_str = str(file_path.resolve().relative_to(source_path))
                except ValueError:
                    # If relative path fails, use absolute path
                    source_path_str = str(file_path.resolve())

                # Chunk the document content
                chunks = self.parser.chunk_content(
                    content=content,
                    chunk_size=chunk_size,
                    chunk_overlap=chunk_overlap
                )

                file_chunks = 0
                # Process each chunk
                for i, chunk in enumerate(chunks):
                    try:
                        # Generate embedding for the chunk
                        embedding = await self.embedding_service.generate_embedding(chunk)

                        # Prepare payload for Qdrant
                        # Convert path to proper Docusaurus URL format
                        # Remove file extension and convert backslashes to forward slashes
                        doc_path = source_path_str.replace(chr(92), '/').replace('.md', '').replace('.mdx', '')
                        # If the file is index.md/index.mdx, remove it from the path
                        if doc_path.endswith('/index'):
                            doc_path = doc_path[:-6]  # Remove '/index'

                        # Ensure the URL includes the base URL and /docs/
                        # Base URL from Docusaurus config: /physical-ai-humanoid-robotics/
                        if not doc_path.startswith('docs/'):
                            doc_url = f"/physical-ai-humanoid-robotics/docs/{doc_path}/"
                        else:
                            doc_url = f"/physical-ai-humanoid-robotics/{doc_path}/"

                        payload = {
                            "document_id": doc_id,
                            "source_path": source_path_str,
                            "title": title,
                            "content": chunk,
                            "position": i,
                            "url": doc_url
                        }

                        # Upsert to Qdrant
                        await self.qdrant_client.upsert(
                            collection_name=settings.QDRANT_COLLECTION_NAME,
                            points=[
                                models.PointStruct(
                                    id=str(uuid.uuid4()),
                                    vector=embedding,
                                    payload=payload
                                )
                            ]
                        )
                        chunks_created += 1
                        file_chunks += 1
                    except Exception as e:
                        print(f"\n  Warning: Error processing chunk {i+1}: {str(e)}")
                        continue

                documents_processed += 1
                print(f"OK ({file_chunks} chunks)")
            except Exception as e:
                print(f"Error: {str(e)}")
                continue

        return {
            "documents_processed": documents_processed,
            "chunks_created": chunks_created
        }

    async def check_collection_exists(self) -> bool:
        """
        Check if the collection exists in Qdrant
        """
        try:
            collections = await self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]
            return settings.QDRANT_COLLECTION_NAME in collection_names
        except Exception:
            return False

    async def create_collection_if_not_exists(self, vector_size: int = 1536):
        """
        Create the collection in Qdrant if it doesn't exist
        """
        if not await self.check_collection_exists():
            await self.qdrant_client.create_collection(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )