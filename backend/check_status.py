"""Quick status check for chat system"""
import asyncio
from app.core.database import get_qdrant_client


async def check_status():
    print("=" * 60)
    print("Chat System Status Check")
    print("=" * 60)
    print()

    try:
        client = get_qdrant_client()

        # Get all collections
        collections = await client.get_collections()
        collection_names = [c.name for c in collections.collections]

        print("Collections in Qdrant:")
        for name in collection_names:
            info = await client.get_collection(name)
            print(f"  - {name}: {info.points_count} points")

        print()

        # Check required collections
        required = ["physical_ai_book", "chat_sessions", "chat_messages"]
        missing = [r for r in required if r not in collection_names]

        if missing:
            print("[ERROR] Missing collections:", ", ".join(missing))
            print("Run: uv run python fix_chat_collections.py")
        else:
            print("[OK] All required collections exist")

        print()

        # Check if there are any chat sessions
        if "chat_sessions" in collection_names:
            sessions_info = await client.get_collection("chat_sessions")
            print(f"Total chat sessions: {sessions_info.points_count}")

        if "chat_messages" in collection_names:
            messages_info = await client.get_collection("chat_messages")
            print(f"Total chat messages: {messages_info.points_count}")

        print()
        print("=" * 60)

    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(check_status())
