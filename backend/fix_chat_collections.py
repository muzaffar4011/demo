"""
Script to fix chat collections by recreating them with proper indexes.
This will delete existing chat_messages and chat_sessions collections and recreate them.
"""
import asyncio
from app.core.database import get_qdrant_client
from qdrant_client.http import models


async def main():
    print("Fixing chat collections with proper indexes...")
    print()

    client = get_qdrant_client()

    try:
        # Get existing collections
        collections = await client.get_collections()
        collection_names = [col.name for col in collections.collections]

        # Delete old collections if they exist
        for collection_name in ["chat_sessions", "chat_messages"]:
            if collection_name in collection_names:
                print(f"Deleting existing collection: {collection_name}")
                await client.delete_collection(collection_name)
                print(f"  OK - Deleted {collection_name}")

        print()
        print("Creating new collections with proper configuration...")
        print()

        # Create sessions collection
        print("Creating chat_sessions collection...")
        await client.create_collection(
            collection_name="chat_sessions",
            vectors_config=models.VectorParams(size=1, distance=models.Distance.COSINE)
        )
        print("  OK - Created chat_sessions")

        # Create messages collection
        print("Creating chat_messages collection...")
        await client.create_collection(
            collection_name="chat_messages",
            vectors_config=models.VectorParams(size=1, distance=models.Distance.COSINE)
        )
        print("  OK - Created chat_messages")

        # Create index for session_id
        print("Creating index for session_id field...")
        await client.create_payload_index(
            collection_name="chat_messages",
            field_name="session_id",
            field_schema=models.PayloadSchemaType.KEYWORD
        )
        print("  OK - Created session_id index")

        print()
        print("=" * 60)
        print("SUCCESS! Chat collections are now properly configured.")
        print("=" * 60)
        print()
        print("You can now use the chatbot with persistent history!")
        print("Note: All previous chat history has been cleared.")
        print()

    except Exception as e:
        print()
        print("=" * 60)
        print("ERROR")
        print("=" * 60)
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
